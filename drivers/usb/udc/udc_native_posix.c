/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * USB device controller (UDC) driver native_posix
 *
 * This is a native_posix for a device controller driver using the UDC API.
 * Please use it as a starting point for a driver implementation for your
 * USB device controller. Maintaining a common style, terminology and
 * abbreviations will allow us to speed up reviews and reduce maintenance.
 * Copy UDC driver native_posix, remove all unrelated comments and replace the
 * copyright notice with your own.
 *
 * Typically, a driver implementation contains only a single source file,
 * but the large list of e.g. register definitions should be in a separate
 * .h file.
 *
 * If you want to define a helper macro, check if there is something similar
 * in include/zephyr/sys/util.h or include/zephyr/usb/usb_ch9.h that you can use.
 * Please keep all identifiers and logging messages concise and clear.
 */

#include "udc_common.h"
#include "zephyr/device.h"
#include "zephyr/net/buf.h"
#include "zephyr/sys/dlist.h"
#include "zephyr/sys/iterable_sections.h"
#include "zephyr/sys/util.h"
#include "zephyr/usb/usbd.h"

#include <asm-generic/errno-base.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>

#include "usbip/usbip.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_native_posix, CONFIG_UDC_DRIVER_LOG_LEVEL);

/*
 * Structure for holding controller configuration items that can remain in
 * non-volatile memory. This is usually accessed as
 *   const struct udc_native_posix_config *config = dev->config;
 */
struct udc_native_posix_config {
	size_t num_of_eps;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	int speed_idx;
};

/*
 * Structure to hold driver private data.
 * Note that this is not accessible via dev->data, but as
 *   struct udc_native_posix_data *priv = udc_get_private(dev);
 */
struct udc_native_posix_data {
	struct k_thread thread_data;
	struct usbip_dev usbip_dev;
};

static int native_posix_ctrl_ep(const struct device *dev, uint8_t ep,
				struct usb_setup_packet *setup)
{
	struct udc_native_posix_data *priv = udc_get_private(dev);
	int err = 0;

	struct net_buf *buf = udc_ctrl_alloc(dev, ep, sizeof(*setup));
	if (buf == NULL) {
		return -ENOMEM;
	}

	net_buf_add_mem(buf, setup, sizeof(*setup));
	udc_ep_buf_set_setup(buf);
	udc_ctrl_update_stage(dev, buf);

	if (udc_ctrl_stage_is_data_out(dev)) {
		uint16_t length;
		struct net_buf *dout;

		LOG_DBG("s: %p | feed for -out-", buf);

		length = udc_data_stage_length(buf);
		dout = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, length);
		if (!dout) {
			return -ENOMEM;
		}

		if (usbip_recv(priv->usbip_dev.connfd, net_buf_add(dout, length), length) < 0) {
			return -EIO;
		}

		udc_ctrl_update_stage(dev, dout);
		err = udc_ctrl_submit_s_out_status(dev, dout);
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		LOG_DBG("s: %p | submit for -in-", setup);

		err = udc_ctrl_submit_s_in_status(dev);
	} else {
		LOG_DBG("s:%p | submit for -status", setup);

		err = udc_ctrl_submit_s_status(dev);
	}

	return err;
}

static int native_posix_data_ep(const struct device *dev, uint8_t ep, size_t length)
{
	struct udc_native_posix_data *priv = udc_get_private(dev);
	struct net_buf *buf = udc_buf_get(dev, ep);

	if (USB_EP_DIR_IS_OUT(ep)) {
		if (buf == NULL) {
			return -ENOMEM;
		}

		if (usbip_recv(priv->usbip_dev.connfd, net_buf_add(buf, length), length) < length) {
			return -EIO;
		}
		usbip_send_common(priv->usbip_dev.connfd, ep, 0);
		return udc_submit_ep_event(dev, buf, 0);
	}

	if (buf != NULL) {
		usbip_send_common(priv->usbip_dev.connfd, ep, buf->len);
		usbip_send(priv->usbip_dev.connfd, buf->data, buf->len);
		net_buf_unref(buf);
		udc_submit_ep_event(dev, buf, 0);
	} else {
		usbip_send_common(priv->usbip_dev.connfd, ep, 0);
	}

	return 0;
}

/*
 * This is called in the context of udc_ep_enqueue() and must
 * not block. The driver can immediately claim the buffer if the queue is empty,
 * but usually it is offloaded to a thread or workqueue to handle transfers
 * in a single location. Please refer to existing driver implementations
 * for examples.
 */
static int udc_native_posix_ep_enqueue(const struct device *dev, struct udc_ep_config *const cfg,
				       struct net_buf *buf)
{
	LOG_DBG("%p enqueue %p", dev, buf);

	if (cfg->stat.halted) {
		/*
		 * It is fine to enqueue a transfer for a halted endpoint,
		 * you need to make sure that transfers are retriggered when
		 * the halt is cleared.
		 *
		 * Always use the abbreviation 'ep' for the endpoint address
		 * and 'ep_idx' or 'ep_num' for the endpoint number identifiers.
		 * Although struct udc_ep_config uses address to be unambiguous
		 * in its context.
		 */
		LOG_DBG("ep 0x%02x halted", cfg->addr);
		// return 0;
	}

	if (USB_EP_GET_IDX(cfg->addr) != 0) {
		udc_buf_put(cfg, buf);
		return 0;
	}

	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		struct udc_native_posix_data *priv = udc_get_private(dev);
		int connfd = priv->usbip_dev.connfd;
		if (udc_ctrl_stage_is_data_in(dev)) {
			usbip_send_common(connfd, cfg->addr, buf->len);
			usbip_send(connfd, buf->data, buf->len);
		} else {
			usbip_send_common(connfd, cfg->addr, 0);
		}

		udc_ctrl_update_stage(dev, buf);
	}

	net_buf_unref(buf);

	return 0;
}

/*
 * This is called in the context of udc_ep_dequeue()
 * and must remove all requests from an endpoint queue
 * Successful removal should be reported to the higher level with
 * ECONNABORTED as the request result.
 * It is up to the request owner to clean up or reuse the buffer.
 */
static int udc_native_posix_ep_dequeue(const struct device *dev, struct udc_ep_config *const cfg)
{
	unsigned int lock_key;
	struct net_buf *buf;

	lock_key = irq_lock();

	buf = udc_buf_get_all(dev, cfg->addr);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	irq_unlock(lock_key);

	return 0;
}

/*
 * Configure and make an endpoint ready for use.
 * This is called in the context of udc_ep_enable() or udc_ep_enable_internal(),
 * the latter of which may be used by the driver to enable control endpoints.
 */
static int udc_native_posix_ep_enable(const struct device *dev, struct udc_ep_config *const cfg)
{
	LOG_DBG("Enable ep 0x%02x", cfg->addr);

	return 0;
}

/*
 * Opposite function to udc_native_posix_ep_enable(). udc_ep_disable_internal()
 * may be used by the driver to disable control endpoints.
 */
static int udc_native_posix_ep_disable(const struct device *dev, struct udc_ep_config *const cfg)
{
	LOG_DBG("Disable ep 0x%02x", cfg->addr);

	return 0;
}

/* Halt endpoint. Halted endpoint should respond with a STALL handshake. */
static int udc_native_posix_ep_set_halt(const struct device *dev, struct udc_ep_config *const cfg)
{
	LOG_DBG("Set halt ep 0x%02x", cfg->addr);

	cfg->stat.halted = true;

	struct udc_native_posix_data *priv = udc_get_private(dev);
	usbip_send_common(priv->usbip_dev.connfd, cfg->addr, 0);

	return 0;
}

/*
 * Opposite to halt endpoint. If there are requests in the endpoint queue,
 * the next transfer should be prepared.
 */
static int udc_native_posix_ep_clear_halt(const struct device *dev, struct udc_ep_config *const cfg)
{
	LOG_DBG("Clear halt ep 0x%02x", cfg->addr);
	cfg->stat.halted = false;

	return 0;
}

static int udc_native_posix_set_address(const struct device *dev, const uint8_t addr)
{
	LOG_DBG("Set new address %u for %p", addr, dev);

	return 0;
}

static int udc_native_posix_host_wakeup(const struct device *dev)
{
	LOG_DBG("Remote wakeup from %p", dev);

	return -ENOTSUP;
}

/* Return actual USB device speed */
static enum udc_bus_speed udc_native_posix_device_speed(const struct device *dev)
{
	struct udc_data *data = dev->data;

	return data->caps.hs ? UDC_BUS_SPEED_HS : UDC_BUS_SPEED_FS;
}

static int udc_native_posix_enable(const struct device *dev)
{
	struct udc_native_posix_data *priv = udc_get_private(dev);

	LOG_DBG("Enable device %p", dev);

	return usbip_add(&priv->usbip_dev);
}

static int udc_native_posix_disable(const struct device *dev)
{
	struct udc_native_posix_data *priv = udc_get_private(dev);

	LOG_DBG("Disable device %p", dev);

	return usbip_remove(&priv->usbip_dev);
}

/*
 * Prepare and configure most of the parts, if the controller has a way
 * of detecting VBUS activity it should be enabled here.
 * Only udc_native_posix_enable() makes device visible to the host.
 */
static int udc_native_posix_init(const struct device *dev)
{
	struct udc_native_posix_data *priv = udc_get_private(dev);
	const struct udc_native_posix_config *cfg = dev->config;
	bool found = false;

	STRUCT_SECTION_FOREACH(usbd_contex, uds_ctx) {
		priv->usbip_dev.dev = dev;
		priv->usbip_dev.ctrl_ep_cb = native_posix_ctrl_ep;
		priv->usbip_dev.data_ep_cb = native_posix_data_ep;
		priv->usbip_dev.speed_idx = cfg->speed_idx;

		if (uds_ctx->dev == dev) {
			sys_snode_t *snode;
			struct usbd_config_node *cfg_nd;
			struct usbd_class_node *c_nd;

			priv->usbip_dev.dev_desc = uds_ctx->desc;

			// TODO: Handle multiple configurations
			snode = sys_slist_peek_head(&uds_ctx->configs);
			cfg_nd = CONTAINER_OF(snode, struct usbd_config_node, node);
			priv->usbip_dev.cfg_desc = cfg_nd->desc;

			// TODO: Handle multiple interfaces
			snode = sys_slist_peek_head(&cfg_nd->class_list);
			c_nd = CONTAINER_OF(snode, struct usbd_class_node, node);
			priv->usbip_dev.if_desc = c_nd->data->desc;

			found = true;
			break;
		}
	}
	if (!found) {
		return -EIO;
	}

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL, 64, 0)) {
		LOG_ERR("Failed to enable control endpoint");
		return -EIO;
	}

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL, 64, 0)) {
		LOG_ERR("Failed to enable control endpoint");
		return -EIO;
	}

	return 0;
}

/* Shut down the controller completely */
static int udc_native_posix_shutdown(const struct device *dev)
{
	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	return 0;
}

/*
 * This is called once to initialize the controller and endpoints
 * capabilities, and register endpoint structures.
 */
static int udc_native_posix_driver_preinit(const struct device *dev)
{
	const struct udc_native_posix_config *config = dev->config;
	struct udc_data *data = dev->data;
	uint16_t mps = 1023;
	int err;

	/*
	 * You do not need to initialize it if your driver does not use
	 * udc_lock_internal() / udc_unlock_internal(), but implements its
	 * own mechanism.
	 */
	k_mutex_init(&data->mutex);

	data->caps.rwup = true;
	data->caps.mps0 = UDC_MPS0_64;
	// data->caps.out_ack = true;
	if (config->speed_idx == 2) {
		data->caps.hs = true;
		mps = 1024;
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			config->ep_cfg_out[i].caps.control = 1;
			config->ep_cfg_out[i].caps.mps = 64;
		} else {
			config->ep_cfg_out[i].caps.bulk = 1;
			config->ep_cfg_out[i].caps.interrupt = 1;
			config->ep_cfg_out[i].caps.iso = 1;
			config->ep_cfg_out[i].caps.mps = mps;
		}

		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		err = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			config->ep_cfg_in[i].caps.control = 1;
			config->ep_cfg_in[i].caps.mps = 64;
		} else {
			config->ep_cfg_in[i].caps.bulk = 1;
			config->ep_cfg_in[i].caps.interrupt = 1;
			config->ep_cfg_in[i].caps.iso = 1;
			config->ep_cfg_in[i].caps.mps = mps;
		}

		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	LOG_INF("Device %p (max. speed %d)", dev, config->speed_idx);

	return 0;
}

static int udc_native_posix_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}

static int udc_native_posix_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

static const struct udc_api udc_native_posix_api = {
	.lock = udc_native_posix_lock,
	.unlock = udc_native_posix_unlock,
	.device_speed = udc_native_posix_device_speed,
	.init = udc_native_posix_init,
	.enable = udc_native_posix_enable,
	.disable = udc_native_posix_disable,
	.shutdown = udc_native_posix_shutdown,
	.set_address = udc_native_posix_set_address,
	.host_wakeup = udc_native_posix_host_wakeup,
	.ep_enable = udc_native_posix_ep_enable,
	.ep_disable = udc_native_posix_ep_disable,
	.ep_set_halt = udc_native_posix_ep_set_halt,
	.ep_clear_halt = udc_native_posix_ep_clear_halt,
	.ep_enqueue = udc_native_posix_ep_enqueue,
	.ep_dequeue = udc_native_posix_ep_dequeue,
};

#define DT_DRV_COMPAT zephyr_native_posix_udc

#define UDC_NATIVE_POSIX_DEVICE_DEFINE(n)                                                          \
	static struct udc_ep_config ep_cfg_out[DT_INST_PROP(n, num_bidir_endpoints)];              \
	static struct udc_ep_config ep_cfg_in[DT_INST_PROP(n, num_bidir_endpoints)];               \
                                                                                                   \
	static const struct udc_native_posix_config udc_native_posix_config_##n = {                \
		.num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),                                \
		.ep_cfg_in = ep_cfg_out,                                                           \
		.ep_cfg_out = ep_cfg_in,                                                           \
		.speed_idx = DT_ENUM_IDX_OR(DT_DRV_INST(n), maximum_speed, 2),                     \
	};                                                                                         \
                                                                                                   \
	static struct udc_native_posix_data udc_priv_##n = {};                                     \
                                                                                                   \
	static struct udc_data udc_data_##n = {                                                    \
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),                                  \
		.priv = &udc_priv_##n,                                                             \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, udc_native_posix_driver_preinit, NULL, &udc_data_##n,             \
			      &udc_native_posix_config_##n, POST_KERNEL,                           \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &udc_native_posix_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_NATIVE_POSIX_DEVICE_DEFINE)
