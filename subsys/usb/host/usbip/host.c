
#include <stdint.h>
#include <netinet/in.h>

#include <zephyr/kernel.h>
#include <zephyr/net/buf.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/sys/dlist.h>

#include "usbh_device.h"
#include "usbip.h"
#include "usbh_ch9.h"
#include "usbh_device.h"
#include "zephyr/drivers/usb/uhc.h"
#include "zephyr/sys/atomic.h"
#include <zephyr/drivers/usb/uhc.h>
#include <zephyr/drivers/usb/udc.h>

LOG_MODULE_REGISTER(usbip_host);

struct usbip_host_data {
	struct usb_device udev;
	struct k_thread urb_thread;
	struct k_thread rx_thread;
	int fd;
};

struct usbip_host_config {
	const struct device *usbd;
	const struct device *usbh;
	const uint8_t addr;
	struct k_mem_slab *urb_slab;
	struct net_buf_pool *xfr_pool;
	sys_dlist_t *urb_list;
	struct k_msgq *msgq;
	void (*start_urb_handler)(const struct device *dev);
	void (*start_rx_handler)(const struct device *dev);
};

enum usbip_host_evt_type {
	URB_SUBMIT,
	URB_COMPLETE,
	URB_UNLINK,
};

struct usbip_host_urb {
	/* USB/IP protocol data */
	uint32_t seqnum;
	uint32_t ep;
	uint32_t transfer_buffer_length;
	uint32_t interval;
	struct usb_setup_packet setup;
	struct net_buf *xfer;

	/* URB tracking data */
	atomic_t claimed;
	int32_t status;
	uint32_t unlink_seqnum;

	struct k_mutex mutex;
	sys_dnode_t _node;
};

struct usbip_host_evt {
	enum usbip_host_evt_type evt;
	struct usbip_host_urb *urb;
};

static int usbip_host_urb_alloc(const struct device *dev, const struct usbip_cmd *cmd,
				struct usbip_host_urb **urb)
{
	int rc = 0;
	const struct usbip_host_config *config = dev->config;
	struct usbip_host_urb *u;
	uint32_t direction;

	rc = k_mem_slab_alloc(config->urb_slab, (void **)urb, K_NO_WAIT);
	if (rc) {
		LOG_ERR("Could not allocate urb.");
		return -ENOMEM;
	}

	direction = ntohl(cmd->hdr.direction);
	u = *urb;

	u->seqnum = ntohl(cmd->hdr.seqnum);
	u->ep = ntohl(cmd->hdr.ep);
	u->ep |= direction == USBIP_DIR_OUT ? USB_EP_DIR_OUT : USB_EP_DIR_IN;

	/* copy setup */
	memcpy(&u->setup, cmd->submit.setup, sizeof(cmd->submit.setup));

	return 0;
}

static int usbip_host_urb_free(const struct device *dev, struct usbip_host_urb *urb)
{
	const struct usbip_host_config *config = dev->config;

	if (urb->xfer) {
		net_buf_unref(urb->xfer);
	}

	if (sys_dnode_is_linked(&urb->_node)) {
		sys_dlist_remove(&urb->_node);
	}

	k_mem_slab_free(config->urb_slab, urb);

	return 0;
}

static int usbip_host_tx_hdr(const struct device *dev, struct usbip_host_urb *urb, uint32_t cmd)
{
	struct usbip_host_data *priv = dev->data;
	uint32_t direction = USB_EP_DIR_IS_OUT(urb->ep) ? USBIP_DIR_OUT : USBIP_DIR_IN;
	uint32_t ep_idx = USB_EP_GET_IDX(urb->ep);

	struct usbip_header_basic hdr = {
		.command = htonl(cmd),
		.seqnum = htonl(urb->seqnum),
		.devid = htonl(1 << 16 | priv->udev.addr),
		.direction = htonl(direction),
		.ep = htonl(ep_idx),
	};

	return usbip_tx(priv->fd, &hdr, sizeof(hdr));
}

static int usbip_host_tx_unlink(const struct device *dev, struct usbip_host_urb *urb)
{
	struct usbip_host_data *priv = dev->data;
	struct usbip_ret_unlink ret = {
		.status = htonl(urb->status),
	};

	usbip_host_tx_hdr(dev, urb, USBIP_RET_UNLINK);
	usbip_tx(priv->fd, &ret, sizeof(ret));

	usbip_host_urb_free(dev, urb);

	return 0;
}

static int usbip_host_tx_submit(const struct device *dev, struct usbip_host_urb *urb)
{
	struct usbip_host_data *priv = dev->data;
	uint32_t actual_length = USB_EP_DIR_IS_OUT(urb->ep) ? 0 : urb->xfer->len;
	struct usbip_ret_submit ret = {
		.status = htonl(urb->status),
		.actual_length = htonl(actual_length),
	};

	usbip_host_tx_hdr(dev, urb, USBIP_RET_SUBMIT);
	usbip_tx(priv->fd, &ret, sizeof(ret));
	if (actual_length) {
		usbip_tx(priv->fd, urb->xfer->data, urb->xfer->len);
	}

	usbip_host_urb_free(dev, urb);

	return 0;
}

static int usbip_host_urb_ctrl_submit(const struct device *dev, struct usbip_host_urb *urb)
{
	int rc;
	struct usbip_host_data *priv = dev->data;
	const struct usbip_host_config *config = dev->config;
	struct usbip_host_evt evt = {
		.evt = URB_COMPLETE,
		.urb = urb,
	};

	urb->status =
		usbh_req_setup(&priv->udev, urb->setup.bmRequestType, urb->setup.bRequest,
			       urb->setup.wValue, urb->setup.wIndex, urb->setup.wLength, urb->xfer);

	/* we cannot block here as we are submitting an event to the current thread */
	rc = k_msgq_put(config->msgq, &evt, K_NO_WAIT);
	if (rc) {
		LOG_ERR("Could not put message");
		return rc;
	}

	return 0;
}
static int usbip_host_urb_data_submit_cb(struct usb_device *const udev,
					 struct uhc_transfer *const xfer, void *data)
{
	const struct device *dev = udev->owner;
	const struct usbip_host_config *config = dev->config;

	struct usbip_host_evt evt = {
		.evt = URB_COMPLETE,
		.urb = data,
	};

	evt.urb->status = xfer->err;

	k_msgq_put(config->msgq, &evt, K_FOREVER);

	usbh_xfer_free(udev, xfer);

	return 0;
}

static int usbip_host_urb_data_submit(const struct device *dev, struct usbip_host_urb *urb)
{
	int ret;
	struct usbip_host_data *priv = dev->data;

	struct uhc_transfer *xfer = usbh_xfer_alloc(&priv->udev, urb->ep, 0, 64, urb->interval,
			(void *)usbip_host_urb_data_submit_cb, urb);

	if (xfer == NULL) {
		return -ENOMEM;
	}

	if (urb->xfer) {
		ret = usbh_xfer_buf_add(&priv->udev, xfer, urb->xfer);
		if (ret) {
			return ret;
		}
	}

	return usbh_xfer_enqueue(&priv->udev, xfer);
}

static int usbip_host_urb_submit(const struct device *dev, struct usbip_host_urb *urb)
{
	if (USB_EP_GET_IDX(urb->ep) == 0) {
		return usbip_host_urb_ctrl_submit(dev, urb);
	}
	return usbip_host_urb_data_submit(dev, urb);
}

static int usbip_host_urb_unlink(const struct device *dev, struct usbip_host_urb *urb)
{
	struct usbip_host_urb *u, *s;
	const struct usbip_host_config *config = dev->config;

	urb->status = 0;

	SYS_DLIST_FOR_EACH_CONTAINER_SAFE(config->urb_list, u, s, _node) {
		if (u->seqnum != urb->unlink_seqnum) {
			continue;
		}

		if (!atomic_test_and_set_bit(&urb->claimed, 0)) {
			urb->status = -ECONNRESET;
			usbip_host_urb_free(dev, u);
		}
	}

	return usbip_host_tx_unlink(dev, urb);
}

static void usbip_host_urb_handler(const struct device *dev)
{
	const struct usbip_host_config *config = dev->config;
	struct usbip_host_evt evt;
	while (true) {
		k_msgq_get(config->msgq, &evt, K_FOREVER);

		switch (evt.evt) {
		case URB_SUBMIT:
			usbip_host_urb_submit(dev, evt.urb);
			break;
		case URB_UNLINK:
			usbip_host_urb_unlink(dev, evt.urb);
			break;
		case URB_COMPLETE:
			usbip_host_tx_submit(dev, evt.urb);
			break;
		default:
			LOG_ERR("Unknown evt %u", evt.evt);
			break;
		}
	}
}

static int usbip_host_rx_submit(const struct device *dev, const struct usbip_cmd *cmd)
{
	int rc = 0;
	int len;
	struct usbip_host_urb *urb;
	const struct usbip_host_config *config = dev->config;
	const struct usbip_host_data *priv = dev->data;

	if (cmd->submit.number_of_packets) {
		LOG_ERR("ISO transfers are not supported");
		return -ENOTSUP;
	}

	len = ntohl(cmd->submit.transfer_buffer_length);
	if (len > CONFIG_USBH_USBIP_MAX_URB_SIZE) {
		LOG_ERR("Transfer size %u is larger than max of %u", len,
			CONFIG_USBH_USBIP_MAX_URB_SIZE);
		return -ENOMEM;
	}

	rc = usbip_host_urb_alloc(dev, cmd, &urb);
	if (rc) {
		return rc;
	}

	/* submit specifics */
	urb->interval = ntohl(cmd->submit.interval);
	urb->transfer_buffer_length = len;
	if (len) {
		urb->xfer = net_buf_alloc(config->xfr_pool, K_NO_WAIT);
		if (urb->xfer == NULL) {
			LOG_ERR("Could not allocate xfr buffer.");
			goto exit;
		}
	}

	if (USB_EP_GET_DIR(urb->ep) == USB_EP_DIR_OUT && len) {
		/* copy xfer */
		rc = usbip_rx(priv->fd, net_buf_add(urb->xfer, len), len);
		if (rc) {
			goto exit;
		}
	}


	struct usbip_host_evt evt = {
		.evt = URB_SUBMIT,
		.urb = urb,
	};

	rc = k_msgq_put(config->msgq, &evt, K_NO_WAIT);
	if (rc) {
		LOG_ERR("Could not put message");
		goto exit;
	}

	return 0;

exit:
	usbip_host_urb_free(dev, urb);
	return rc;
}

static int usbip_host_rx_unlink(const struct device *dev, const struct usbip_cmd *cmd)
{
	struct usbip_host_urb *urb;
	int rc;
	const struct usbip_host_config *config = dev->config;

	rc = usbip_host_urb_alloc(dev, cmd, &urb);
	if (rc) {
		return rc;
	}

	/* unlink specifics */
	urb->unlink_seqnum = ntohl(cmd->unlink.seqnum);

	struct usbip_host_evt evt = {
		.evt = URB_UNLINK,
		.urb = urb,
	};

	rc = k_msgq_put(config->msgq, &evt, K_NO_WAIT);
	if (rc) {
		LOG_ERR("Could not put message");
		goto exit;
	}

	return 0;
exit:
	usbip_host_urb_free(dev, urb);
	return rc;
}

static int usbip_host_pre_init(const struct device *dev)
{
	struct usbip_host_data *priv = dev->data;
	const struct usbip_host_config *config = dev->config;

	priv->fd = -1;
	priv->udev.owner = (void *)dev;

	STRUCT_SECTION_FOREACH(usbh_contex, uhs_ctx) {
		if (uhs_ctx->dev == config->usbh) {
			priv->udev.ctx = uhs_ctx;
			break;
		}
	}

	if (priv->udev.ctx == NULL) {
		return -ENOTSUP;
	}

	return 0;
}

static void usbip_host_rx_handler(const struct device *dev)
{
	struct usbip_host_data *priv = dev->data;
	struct usbip_cmd cmd;
	int ret;

	while (true) {
		ret = usbip_rx(priv->fd, &cmd, sizeof(cmd));
		if (ret) {
			break;
		}

		uint32_t command = ntohl(cmd.hdr.command);
		switch (command) {
		case USBIP_CMD_SUBMIT:
			ret = usbip_host_rx_submit(dev, &cmd);
			break;
		case USBIP_CMD_UNLINK:
			ret = usbip_host_rx_unlink(dev, &cmd);
			break;
		default:
			LOG_ERR("Unknown command 0x%x", command);
			ret = -EINVAL;
			break;
		}

		if (ret) {
			return;
		}

		// TODO: remove this
		k_sleep(K_MSEC(10));
	}
}

static int usbip_host_imported(const struct usb_device *udev, int fd)
{
	const struct device *dev = udev->owner;
	struct usbip_host_data *priv = dev->data;
	const struct usbip_host_config *config = dev->config;

	priv->fd = fd;

	config->start_urb_handler(dev);
	config->start_rx_handler(dev);

	return 0;
}

static int usbip_host_disconneted(const struct usb_device *udev)
{
	const struct device *dev = udev->owner;
	struct usbip_host_data *priv = dev->data;

	priv->fd = -1;

	return 0;
}

int usbip_host_init(const struct device *dev)
{
	int ret;
	const struct usbip_host_config *config = dev->config;
	struct usbip_host_data *priv = dev->data;

	if (!udc_is_enabled(config->usbd)) {
		LOG_ERR("USB Device %s is not ready", config->usbd->name);
		return -EPERM;
	}

	if (!uhc_is_enabled(config->usbh)) {
		LOG_ERR("USB Host %s is not ready", config->usbh->name);
		return -EPERM;
	}

	/* Enable SOF timer */
	ret = uhc_bus_resume(config->usbh);
	if (ret) {
		LOG_ERR("Could not resume bust");
		return ret;
	}

	ret = usbh_req_set_address(&priv->udev, config->addr);
	if (ret) {
		LOG_ERR("Could not set device address %d", ret);
		return ret;
	}

	return 0;
}

int usbip_host_enable(const struct device *dev)
{
	int ret;
	struct usbip_host_data *priv = dev->data;

	ret = usbip_add_device(&priv->udev);
	if (ret) {
		LOG_ERR("Failed to add USBIP device: %d", ret);
		return ret;
	}

	return 0;
}

static struct usbip_api usbip_host_api = {
	.imported = usbip_host_imported,
	.disconnected = usbip_host_disconneted,
};

#define DT_DRV_COMPAT zephyr_usbip_host

#define USBIP_HOST_DEFINE(n)                                                                       \
	static void usbip_host_urb_thread_##n(void *dev, void *arg1, void *arg2)                   \
	{                                                                                          \
		usbip_host_urb_handler(dev);                                                       \
	}                                                                                          \
                                                                                                   \
	K_THREAD_STACK_DEFINE(usbip_host_urb_stack_##n, CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE); \
	static void usbip_host_start_urb_handler_##n(const struct device *dev)                     \
	{                                                                                          \
		struct usbip_host_data *priv = dev->data;                                          \
                                                                                                   \
		k_thread_create(&priv->urb_thread, usbip_host_urb_stack_##n,                       \
				CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE,                          \
				usbip_host_urb_thread_##n, (void *)dev, NULL, NULL,                \
				K_PRIO_COOP(4), K_ESSENTIAL, K_NO_WAIT);                           \
		k_thread_name_set(&priv->urb_thread, dev->name);                                   \
	}                                                                                          \
                                                                                                   \
	static void usbip_host_rx_thread_##n(void *dev, void *arg1, void *arg2)                    \
	{                                                                                          \
		usbip_host_rx_handler(dev);                                                        \
	}                                                                                          \
                                                                                                   \
	K_THREAD_STACK_DEFINE(usbip_host_rx_stack_##n, CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE);  \
	static void usbip_host_start_rx_handler_##n(const struct device *dev)                      \
	{                                                                                          \
		struct usbip_host_data *priv = dev->data;                                          \
                                                                                                   \
		k_thread_create(&priv->rx_thread, usbip_host_rx_stack_##n,                         \
				CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE,                          \
				usbip_host_rx_thread_##n, (void *)dev, NULL, NULL, K_PRIO_COOP(4), \
				K_ESSENTIAL, K_NO_WAIT);                                           \
		k_thread_name_set(&priv->rx_thread, dev->name);                                    \
	}                                                                                          \
	K_MEM_SLAB_DEFINE(usbip_host_urb_slab_##n, sizeof(struct usbip_host_urb),                  \
			  CONFIG_USBH_USBIP_URB_COUNT, sizeof(void *));                            \
	NET_BUF_POOL_DEFINE(usbip_host_xfr_pool_##n, CONFIG_USBH_USBIP_URB_COUNT,                  \
			    CONFIG_USBH_USBIP_MAX_URB_SIZE, 0, NULL);                              \
	K_MSGQ_DEFINE(usbip_host_msgq_##n, sizeof(struct usbip_host_evt),                          \
		      CONFIG_USBH_USBIP_URB_COUNT, 1);                                             \
                                                                                                   \
	static sys_dlist_t urb_list_##n = SYS_DLIST_STATIC_INIT(&urb_list_##n);                    \
                                                                                                   \
	struct usbip_host_data usbip_host_data_##n = {};                                           \
	struct usbip_host_config usbip_host_config_##n = {                                         \
		.usbd = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(n), device)),                         \
		.addr = DT_NODE_CHILD_IDX(DT_PHANDLE(DT_DRV_INST(n), device)) + 1,                 \
		.usbh = DEVICE_DT_GET(DT_PARENT(DT_PHANDLE(DT_DRV_INST(n), device))),              \
		.urb_slab = &usbip_host_urb_slab_##n,                                              \
		.xfr_pool = &usbip_host_xfr_pool_##n,                                              \
		.urb_list = &urb_list_##n,                                                         \
		.msgq = &usbip_host_msgq_##n,                                                      \
		.start_urb_handler = usbip_host_start_urb_handler_##n,                             \
		.start_rx_handler = usbip_host_start_rx_handler_##n,                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, usbip_host_pre_init, NULL, &usbip_host_data_##n,                  \
			      &usbip_host_config_##n, POST_KERNEL,                                 \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &usbip_host_api);

DT_INST_FOREACH_STATUS_OKAY(USBIP_HOST_DEFINE);