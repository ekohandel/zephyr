#define _GNU_SOURCE 1
#include <stdio.h>
#include <asm-generic/errno-base.h>
#include "zephyr/net/buf.h"
#include "zephyr/sys/util_macro.h"
#include <errno.h>
#include <stdint.h>

#include <asm-generic/errno.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/sys/dlist.h>
#include <posix_board_if.h>

#include "usbip.h"
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <arpa/inet.h>

#include "usbh_ch9.h"

LOG_MODULE_REGISTER(usbip);

#define USBIP_BUSNUM 1

struct usbip_dev {
	struct usb_device *udev;
	uint8_t bConfigurationValue;
	uint8_t bNumInterfaces;
	struct usb_device_descriptor dev_desc;
	bool imported;

	sys_dnode_t _node;
};

struct usbip_conn {
	k_thread_stack_t *stack;
	struct k_thread thread;
	struct net_buf_pool *buf_pool;
	bool started;
};

K_MEM_SLAB_DEFINE(usbip_dev_slab, sizeof(struct usbip_dev), CONFIG_USBH_USBIP_MAX_DEVICE_COUNT,
		  sizeof(void *));
static sys_dlist_t usbip_dev_list = SYS_DLIST_STATIC_INIT(&usbip_dev_list);
static int usbip_dev_count = 0;

#define USBIP_CONN_BUF_POOL(i, _) NET_BUF_POOL_DEFINE(usbip_conn_buf_pool_##i, 1, 512, 0, NULL)

LISTIFY(CONFIG_USBH_USBIP_SERVER_MAX_CONNECTIONS, USBIP_CONN_BUF_POOL, (;));

#define USBIP_CONN(i, _)                                                                           \
	{                                                                                          \
		.stack = usbip_conn_stacks[i], .buf_pool = &usbip_conn_buf_pool_##i,               \
		.started = false,                                                                  \
	}

static K_THREAD_STACK_ARRAY_DEFINE(usbip_conn_stacks, CONFIG_USBH_USBIP_SERVER_MAX_CONNECTIONS,
				   4096);
static struct usbip_conn usbip_conn[] = {
	LISTIFY(CONFIG_USBH_USBIP_SERVER_MAX_CONNECTIONS, USBIP_CONN, (, ))};

static int usbip_srv_make_busid(uint8_t *buf, size_t len, const struct usbip_dev *ipdev)
{
	return snprintf(buf, len, "%u-%u", USBIP_BUSNUM, ipdev->udev->addr);
}

static int usbip_srv_tx_hdr(int fd, uint32_t command)
{
	const struct usbip_op_header hdr = {
		.version = htons(USBIP_VERSION),
		.command = htons(command),
		.status = 0,
	};

	return usbip_tx(fd, &hdr, sizeof(hdr));

}
static int usbip_srv_tx_device(int fd, const struct usbip_dev *ipdev)
{
	int rc;
	struct usb_device *udev = ipdev->udev;

	struct usbip_op_device device = {
		.busnum = htonl(USBIP_BUSNUM),
		.devnum = htonl(udev->addr),
		.speed = htonl(2),
		.idVendor = htons(ipdev->dev_desc.idVendor),
		.idProduct = htons(ipdev->dev_desc.idProduct),
		.bcdDevice = htons(ipdev->dev_desc.bcdDevice),
		.bcdDeviceClass = ipdev->dev_desc.bDeviceClass,
		.bcdDeviceSubClass = ipdev->dev_desc.bDeviceSubClass,
		.bcdDeviceProtocol = ipdev->dev_desc.bDeviceProtocol,
		.bConfigurationValue = ipdev->bConfigurationValue,
		.bNumConfigurations = ipdev->dev_desc.bNumConfigurations,
		.bNumInterfaces = ipdev->bNumInterfaces,
	};

	snprintf(device.path, sizeof(device.path), "%s", udev->ctx->name);
	usbip_srv_make_busid(device.busid, sizeof(device.busid), ipdev);

	rc = usbip_tx(fd, &device, sizeof(device));
	if (rc) {
		return rc;
	}

	return 0;
}

static int usbip_srv_tx_interfaces(int fd, const struct usbip_dev *ipdev, struct net_buf_pool *pool)
{
	int rc = 0;
	struct usb_if_descriptor *if_desc;
	struct usbip_op_interface interface;
	const uint8_t desc_buf[512];
	const uint8_t *buf;
	int count = 0;

	/* Interface descriptors are returned by a request to configuration descriptors */
	rc = usbh_req_desc_cfg(ipdev->udev, ipdev->bConfigurationValue, sizeof(desc_buf),
			       (void *)desc_buf);
	if (rc) {
		return rc;
	}

	buf = desc_buf;
	if_desc = (void *)buf;

	while (if_desc->bLength) {
		if (if_desc->bDescriptorType == USB_DESC_INTERFACE) {
			interface.bInterfaceClass = if_desc->bInterfaceClass;
			interface.bInterfaceSubClass = if_desc->bInterfaceSubClass;
			interface.bInterfaceProtocol = if_desc->bInterfaceProtocol;
			interface.padding = 0;

			if (++count > ipdev->bNumInterfaces) {
				LOG_ERR("Too many intefaces found, expected only %u",
					ipdev->bNumInterfaces);
				return -EINVAL;
			}

			rc = usbip_tx(fd, &interface, sizeof(interface));
			if (rc) {
				return rc;
			}
		}

		buf += if_desc->bLength;
		if_desc = (void *)buf;
	}

	return 0;
}

static int usbip_srv_rx_devlist(int fd, struct net_buf_pool *pool)
{
	int ret;
	uint32_t num = 0;
	struct usbip_dev *dev;

	ret = usbip_srv_tx_hdr(fd, USBIP_OP_REP_DEVLIST);
	if (ret) {
		return ret;
	}

	num = htonl(usbip_dev_count);
	ret = usbip_tx(fd, &num, sizeof(num));
	if (ret) {
		return ret;
	}

	SYS_DLIST_FOR_EACH_CONTAINER(&usbip_dev_list, dev, _node) {
		if (dev->imported) {
			continue;
		}

		ret = usbip_srv_tx_device(fd, dev);
		if (ret) {
			return ret;
		}

		ret = usbip_srv_tx_interfaces(fd, dev, pool);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

static int usbip_srv_rx_import(int fd, struct usbip_dev **ipdev)
{
	int ret = 0;
	struct usbip_dev *dev;
	uint8_t req_busid[USBIP_BUSID_LEN];
	uint8_t dev_busid[USBIP_BUSID_LEN];

	ret = usbip_rx(fd, &req_busid, sizeof(req_busid));
	if (ret) {
		return ret;
	}

	SYS_DLIST_FOR_EACH_CONTAINER(&usbip_dev_list, dev, _node) {
		if (dev->imported) {
			continue;
		}

		usbip_srv_make_busid(dev_busid, sizeof(dev_busid), dev);
		if (strncmp(req_busid, dev_busid, sizeof(req_busid)) != 0) {
			continue;
		}

		dev->imported = true;
		ret = usbip_srv_tx_hdr(fd, USBIP_OP_REP_IMPORT);
		if (ret) {
			return ret;
		}

		ret = usbip_srv_tx_device(fd, dev);
		*ipdev = dev;
		break;
	}

	return ret;
}

static void usbip_srv_conn_handler(void *p1, void *p2, void *p3)
{
	int ret;
	int fd = (int)(uintptr_t)p1;
	struct net_buf_pool *pool = (void *)(uintptr_t)p2;
	struct usbip_dev *ipdev = NULL;

	while (true) {
		int command;

		struct usbip_op_header hdr;

		ret = usbip_rx(fd, &hdr, sizeof(hdr));
		if (ret) {
			break;
		}

		LOG_HEXDUMP_DBG((uint8_t *)&hdr, sizeof(hdr), "Got request");
		LOG_DBG("Code: 0x%x", ntohs(hdr.command));

		command = ntohs(hdr.command);
		switch (command) {
		case USBIP_OP_REQ_DEVLIST:
			usbip_srv_rx_devlist(fd, pool);
			goto exit;
		case USBIP_OP_REQ_IMPORT:
			ret = usbip_srv_rx_import(fd, &ipdev);
			if (ret) {
				goto exit;
			}
			usbip_imported(ipdev->udev, fd);
			return;
		default:
			LOG_ERR("Unhandled code: 0x%x", command);
			break;
		}
	}
exit:
	close(fd);
}

static int usbip_srv_conn(int fd)
{
	int rc = 0;
	struct usbip_conn *conn;

	for (int i = 0; i < CONFIG_USBH_USBIP_SERVER_MAX_CONNECTIONS; i++) {
		conn = &usbip_conn[i];
		if (conn->started) {
			rc = k_thread_join(&conn->thread, K_NO_WAIT);
			if (rc) {
				continue;
			}
		}

		conn->started = true;
		k_thread_create(&conn->thread, conn->stack,
				4096, usbip_srv_conn_handler,
				(void *)(uintptr_t)fd, conn->buf_pool, NULL,
				K_PRIO_COOP(CONFIG_USBH_USBIP_SERVER_THREAD_PRIO), 0, K_NO_WAIT);

		return 0;
	}

	return rc;
}

static void usbip_srv_thread(void *p1, void *p2, void *p3)
{
	struct sockaddr_in srv;
	int listenfd, connfd;
	int reuse = 1;
	int rc;

	LOG_DBG("Starting");

	listenfd = socket(PF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
	if (listenfd < 0) {
		LOG_ERR("socket() failed: %s", strerror(errno));
		posix_exit(EXIT_FAILURE);
	}

	if (setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse)) <
	    0) {
		LOG_WRN("setsockopt() failed: %s", strerror(errno));
	}

	memset(&srv, 0, sizeof(srv));
	srv.sin_family = AF_INET;
	srv.sin_addr.s_addr = htonl(INADDR_ANY);
	srv.sin_port = htons(CONFIG_USBH_USBIP_SERVER_PORT);

	if (bind(listenfd, (struct sockaddr *)&srv, sizeof(srv)) < 0) {
		LOG_ERR("bind() failed: %s", strerror(errno));
		posix_exit(EXIT_FAILURE);
	}

	if (listen(listenfd, SOMAXCONN) < 0) {
		LOG_ERR("listen() failed: %s", strerror(errno));
		posix_exit(EXIT_FAILURE);
	}

	while (true) {
		struct sockaddr_in client_addr;
		socklen_t client_addr_len = sizeof(client_addr);

		connfd = accept4(listenfd, (struct sockaddr *)&client_addr, &client_addr_len,
				 SOCK_NONBLOCK);
		if (connfd < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				/* Non-blocking accept */
				k_sleep(K_MSEC(100));

				continue;
			}

			LOG_ERR("accept() failed: %s", strerror(errno));
			posix_exit(EXIT_FAILURE);
		}

		rc = usbip_srv_conn(connfd);
		if (rc) {
			LOG_ERR("No more connection threads available: %d", rc);
			close(connfd);
			continue;
		}

		LOG_DBG("Connection: %s", inet_ntoa(client_addr.sin_addr));
	}
}

int usbip_add_device(struct usb_device *udev)
{
	struct usbip_dev *dev;
	struct usb_cfg_descriptor cfg_desc;
	uint8_t bConfigurationValue;

	int rc;

	rc = k_mem_slab_alloc(&usbip_dev_slab, (void **)&dev, K_NO_WAIT);
	if (rc) {
		LOG_ERR("Could not allocate USBIP device: %d", rc);
		return rc;
	}

	dev->udev = udev;

	rc = usbh_req_desc_dev(udev, &dev->dev_desc);
	if (rc) {
		LOG_ERR("Failed to get Device Descriptor");
		return rc;
	}

	rc = usbh_req_get_cfg(udev, &bConfigurationValue);
	if (rc) {
		LOG_ERR("Failed to get bConfigurationValue: %d", rc);
		return rc;
	}

	rc = usbh_req_desc_cfg(udev, bConfigurationValue, sizeof(cfg_desc), &cfg_desc);
	if (rc) {
		LOG_ERR("Failed to get Configuration Descriptor for configuration %u",
			dev->bConfigurationValue);
		return rc;
	}

	dev->bConfigurationValue = bConfigurationValue;
	dev->bNumInterfaces = cfg_desc.bNumInterfaces;

	sys_dnode_init(&dev->_node);
	sys_dlist_append(&usbip_dev_list, &dev->_node);
	usbip_dev_count++;

	return 0;
}

int usbip_remove_device(struct usb_device *udev)
{
	return -ENOTSUP;
}

int usbip_imported(const struct usb_device *udev, int fd)
{
	struct device *dev = udev->owner;
	const struct usbip_api *api = dev->api;

	return api->imported(udev, fd);
}

int usbip_rx_cmd(const struct usb_device *udev, const struct usbip_cmd *cmd)
{
	return 0;
}

int usbip_disconnected(const struct usb_device *udev)
{
	return 0;
}

int usbip_rx(int fd, void *buf, size_t len)
{
	while (true) {
		int ret = recv(fd, buf, len, 0);
		if (ret < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				k_sleep(K_MSEC(100));
				continue;
			}
			LOG_ERR("recv() failed: %s", strerror(errno));
			return ret;
		}

		if (ret == 0) {
			LOG_INF("recv() failed: peer terminated connection");
			return -ECONNRESET;
		}

		if (ret != len) {
			LOG_ERR("recv() failed to receive all %zu", len);
			return -ENODATA;
		}

		return 0;
	}
}

int usbip_tx(int fd, const void *buf, size_t len)
{
	int ret = send(fd, buf, len, 0);
	if (ret < 0) {
		LOG_ERR("send() failed: %s", strerror(errno));
		return ret;
	}

	if (ret != len) {
		LOG_ERR("send() failed to send all %zu", len);
		return ret;
	}

	return 0;
}

K_THREAD_DEFINE(usbip_srv, 4096, usbip_srv_thread, NULL, NULL,
		NULL, K_PRIO_COOP(CONFIG_USBH_USBIP_SERVER_THREAD_PRIO), 0, 0);
