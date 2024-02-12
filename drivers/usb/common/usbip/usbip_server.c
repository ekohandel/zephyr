/*
 * Copyright (c) 2018 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* For accept4() */
#define _GNU_SOURCE 1

#define __packed __attribute__((__packed__))

#include "zephyr/sys/atomic.h"
#include "zephyr/sys/atomic_types.h"
#include "zephyr/device.h"
#include "zephyr/logging/log.h"
#include <stdint.h>
#include "zephyr/sys/dlist.h"
#include "zephyr/kernel/thread_stack.h"
#include "zephyr/net/buf.h"
#include "zephyr/sys/arch_interface.h"
#include "zephyr/sys/util.h"
#include <asm-generic/errno-base.h>

#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* Zephyr headers */
#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/dlist.h>
#include <zephyr/usb/usbd.h>

#include <posix_board_if.h>
#include "usbip.h"

#define LOG_LEVEL CONFIG_UDC_DRIVER_LOG_LEVEL
LOG_MODULE_REGISTER(usbip_server);

#define USBIP_VERSION 0x111
#define USBIP_BUS_NUM 1

#define VERBOSE_DEBUG

int seqnum_global;

static atomic_t dev_num = ATOMIC_INIT(1);

struct thread_node {
	sys_dnode_t node;
	k_thread_stack_t *stack;
	struct k_thread thread;
};
struct usbip_connection_thread {
	struct thread_node thread[CONFIG_USBIP_SERVER_MAX_CONNECTIONS];
	sys_dlist_t free_list;
	sys_dlist_t running_list;
};
static K_KERNEL_STACK_ARRAY_DEFINE(connection_thread_stack, CONFIG_USBIP_SERVER_MAX_CONNECTIONS,
				   CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE);
static struct usbip_connection_thread connection_thread;

static sys_dlist_t usbip_dev_list = SYS_DLIST_STATIC_INIT(&usbip_dev_list);

NET_BUF_POOL_DEFINE(usbip_netbufs, 8, 64, 64, NULL);

/* Helpers */

static void usbip_header_dump(struct usbip_header *hdr)
{
	LOG_DBG("cmd %x seq %u dir %u ep %x", ntohl(hdr->common.command), ntohl(hdr->common.seqnum),
		ntohl(hdr->common.direction), ntohl(hdr->common.ep));

	switch (ntohl(hdr->common.command)) {
	case USBIP_CMD_SUBMIT:
		LOG_DBG("flags %x np %u int %u buflen %u type %x",
			ntohl(hdr->u.submit.transfer_flags), ntohl(hdr->u.submit.number_of_packets),
			ntohl(hdr->u.submit.interval), ntohl(hdr->u.submit.transfer_buffer_length),
			ntohl(hdr->u.submit.bmRequestType));
		break;
	case USBIP_CMD_UNLINK:
		LOG_DBG("seq %d", ntohl(hdr->u.unlink.seqnum));
		break;
	default:
		break;
	}
}

static int send_interfaces(int fd, const struct usbip_dev *usbip_dev)
{
	struct devlist_interface iface;
	const uint8_t *descriptors = (void *)usbip_dev->if_desc;

	while (descriptors[0]) {
		if (descriptors[1] == USB_DESC_INTERFACE) {
			struct usb_if_descriptor *desc = (void *)descriptors;

			iface.bInterfaceClass = desc->bInterfaceClass;
			iface.bInterfaceSubClass = desc->bInterfaceSubClass;
			iface.bInterfaceProtocol = desc->bInterfaceProtocol;
			iface.padding = 0U;

			if (send(fd, &iface, sizeof(iface), 0) != sizeof(iface)) {
				LOG_ERR("send() failed: %s", strerror(errno));
				return errno;
			}
		}

		/* skip to next descriptor */
		descriptors += descriptors[0];
	}

	return 0;
}

static void fill_device(const struct usbip_dev *usbip_dev, struct devlist_device *dev)
{
	strncpy(dev->path, usbip_dev->dev->name, USBIP_DEV_PATH_LEN);

	memset(dev->busid, 0, USBIP_DEV_BUS_ID_LEN);
	snprintf(dev->busid, USBIP_DEV_BUS_ID_LEN, "%u-%u", USBIP_BUS_NUM, usbip_dev->dev_num);

	dev->busnum = htonl(USBIP_BUS_NUM);
	dev->devnum = htonl(usbip_dev->dev_num);
	dev->speed = htonl(usbip_dev->speed_idx);

	dev->idVendor = htons(usbip_dev->dev_desc->idVendor);
	dev->idProduct = htons(usbip_dev->dev_desc->idProduct);
	dev->bcdDevice = htons(usbip_dev->dev_desc->bcdDevice);
	dev->bDeviceClass = usbip_dev->dev_desc->bDeviceClass;
	dev->bDeviceSubClass = usbip_dev->dev_desc->bDeviceSubClass;
	dev->bDeviceProtocol = usbip_dev->dev_desc->bDeviceProtocol;

	dev->bConfigurationValue = usbip_dev->cfg_desc->bConfigurationValue;
	dev->bNumConfigurations = usbip_dev->dev_desc->bNumConfigurations;
	dev->bNumInterfaces = usbip_dev->cfg_desc->bNumInterfaces;
}

static int send_device(int fd, const struct usbip_dev *usbip_dev)
{
	struct devlist_device dev;

	fill_device(usbip_dev, &dev);

	if (send(fd, &dev, sizeof(dev), 0) != sizeof(dev)) {
		LOG_ERR("send() device failed: %s", strerror(errno));
		return errno;
	}

	return 0;
}

static int handle_device_list(int connfd)
{
	struct op_common header = {
		.version = htons(USBIP_VERSION),
		.code = htons(OP_REP_DEVLIST),
		.status = 0,
	};

	if (send(connfd, &header, sizeof(header), 0) != sizeof(header)) {
		LOG_ERR("send() header failed: %s", strerror(errno));
		return errno;
	}

	/* Send number of devices */
	uint32_t ndev = htonl(sys_dlist_len(&usbip_dev_list));

	if (send(connfd, &ndev, sizeof(ndev), 0) != sizeof(ndev)) {
		LOG_ERR("send() ndev failed: %s", strerror(errno));
		return errno;
	}

	struct usbip_dev *usbip_dev;
	SYS_DLIST_FOR_EACH_CONTAINER(&usbip_dev_list, usbip_dev, _node) {
		send_device(connfd, usbip_dev);
		send_interfaces(connfd, usbip_dev);
	}

	return 0;
}

static int handle_control(struct usbip_dev *usbip_dev, struct usbip_header *hdr)
{
	const struct device *dev = usbip_dev->dev;
	uint8_t ep = ntohl(hdr->common.ep);
	void *setup = (void *)&hdr->u.submit.bmRequestType;

	return usbip_dev->ctrl_ep_cb(dev, ep, setup);
}

int handle_data(struct usbip_dev *usbip_dev, struct usbip_header *hdr)
{
	const struct device *dev = usbip_dev->dev;
	uint8_t ep = ntohl(hdr->common.ep);
	ep |= (ntohl(hdr->common.direction) == USBIP_DIR_OUT) ? USB_EP_DIR_OUT : USB_EP_DIR_IN;
	size_t data_len = ntohl(hdr->u.submit.transfer_buffer_length);

	return usbip_dev->data_ep_cb(dev, ep, data_len);
}

static void handle_usbip_submit(struct usbip_dev *usbip_dev, struct usbip_header *hdr)
{
	struct usbip_submit *req = &hdr->u.submit;
	int read;
	int fd = usbip_dev->connfd;

	LOG_DBG("");

	read = recv(fd, req, sizeof(*req), 0);
	if (read != sizeof(*req)) {
		LOG_ERR("recv() failed: %s", strerror(errno));
		return;
	}

	usbip_header_dump((void *)hdr);

	if (ntohl(hdr->common.ep) == 0) {
		handle_control(usbip_dev, hdr);
	} else {
		handle_data(usbip_dev, hdr);
	}
}

static void handle_usbip_unlink(int connfd, struct usbip_header *hdr)
{
	int read;

	LOG_DBG("");

	/* Need to read the whole structure */
	read = recv(connfd, &hdr->u, sizeof(hdr->u), 0);
	if (read != sizeof(hdr->u)) {
		LOG_ERR("recv() failed: %s", strerror(errno));
		return;
	}

	usbip_header_dump((void *)hdr);

	/* TODO: unlink */
}

static int handle_import(int connfd, struct usbip_dev **usbip_dev)
{
	struct op_common header = {
		.version = htons(USBIP_VERSION),
		.code = htons(OP_REP_IMPORT),
		.status = 0,
	};
	char busid[32];

	LOG_DBG("attach device");

	if (recv(connfd, busid, sizeof(busid), 0) != sizeof(busid)) {
		LOG_ERR("recv() failed: %s", strerror(errno));
		return errno;
	}

	if (send(connfd, &header, sizeof(header), 0) != sizeof(header)) {
		LOG_ERR("send() header failed: %s", strerror(errno));
		return errno;
	}

	sys_dlist_t *dnode = sys_dlist_peek_head(&usbip_dev_list);
	*usbip_dev = CONTAINER_OF(dnode, struct usbip_dev, _node);

	send_device(connfd, *usbip_dev);

	return 0;
}

void usbip_connection_thread(void *p1, void *p2, void *p3)
{
	unsigned char attached;
	int connfd = (int)(uintptr_t)p1;

	/* Set attached 0 */
	attached = 0U;
	struct usbip_dev *usbip_dev = NULL;

	while (true) {
		struct usbip_header cmd;
		struct usbip_header_common *hdr = &cmd.common;
		int read;

		if (!attached) {
			struct op_common req;

			read = recv(connfd, &req, sizeof(req), 0);
			if (read < 0) {
				if (errno == EAGAIN || errno == EWOULDBLOCK) {
					/* Non-blocking accept */
					k_sleep(K_MSEC(100));

					continue;
				}
			}

			if (read != sizeof(req)) {
				LOG_WRN("wrong length, %d", read);

				/* Closing connection */
				break;
			}

			LOG_HEXDUMP_DBG((uint8_t *)&req, sizeof(req), "Got request");

			LOG_DBG("Code: 0x%x", ntohs(req.code));

			switch (ntohs(req.code)) {
			case OP_REQ_DEVLIST:
				handle_device_list(connfd);
				break;
			case OP_REQ_IMPORT:
				if (!handle_import(connfd, &usbip_dev)) {
					usbip_dev->connfd = connfd;
					attached = 1U;
				}
				break;
			default:
				LOG_ERR("Unhandled code: 0x%x", ntohs(req.code));
				break;
			}

			continue;
		}

		/* Handle attached case */

		read = recv(connfd, hdr, sizeof(*hdr), 0);
		if (read < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				/* Non-blocking accept */
				k_sleep(K_MSEC(100));

				continue;
			}
		}

		LOG_HEXDUMP_DBG((uint8_t *)hdr, read, "Got cmd");

		if (read != sizeof(*hdr)) {
			LOG_ERR("recv wrong length: %d", read);

			/* Closing connection */
			break;
		}

		seqnum_global = ntohl(hdr->seqnum);

		switch (ntohl(hdr->command)) {
		case USBIP_CMD_SUBMIT:
			handle_usbip_submit(usbip_dev, &cmd);
			break;
		case USBIP_CMD_UNLINK:
			handle_usbip_unlink(connfd, &cmd);
			break;
		default:
			LOG_ERR("Unknown command: 0x%x", ntohl(hdr->command));
			close(connfd);
			return;
		}

		k_sleep(K_MSEC(10));
	}

	LOG_DBG("Closing connection");
	close(connfd);
}

void usbip_server_thread(void *p1, void *p2, void *p3)
{
	struct sockaddr_in srv;
	int listenfd, connfd;
	int reuse = 1;

	LOG_DBG("Starting");

	sys_dlist_init(&connection_thread.free_list);
	sys_dlist_init(&connection_thread.running_list);
	for (int i = 0; i < CONFIG_USBIP_SERVER_MAX_CONNECTIONS; i++) {
		connection_thread.thread[i].stack = connection_thread_stack[i];
		sys_dlist_append(&connection_thread.free_list, &connection_thread.thread[i].node);
	}

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
	srv.sin_port = htons(CONFIG_USBIP_SERVER_PORT);

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
		sys_dnode_t *node, *safety;
		struct thread_node *thread_node;

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

		/* reclaim any threads that are done as free */
		SYS_DLIST_FOR_EACH_NODE_SAFE(&connection_thread.running_list, node, safety) {
			thread_node = CONTAINER_OF(node, struct thread_node, node);
			if (!k_thread_join(&thread_node->thread, K_NO_WAIT)) {
				sys_dlist_remove(&thread_node->node);
				sys_dlist_append(&connection_thread.free_list, &thread_node->node);
			}
		}

		/* find a free thread to handle the connection */
		node = sys_dlist_get(&connection_thread.free_list);
		if (!node) {
			LOG_ERR("No more connection threads available.");
			close(connfd);
			continue;
		}
		thread_node = CONTAINER_OF(node, struct thread_node, node);
		sys_dlist_append(&connection_thread.running_list, &thread_node->node);

		LOG_DBG("Connection: %s", inet_ntoa(client_addr.sin_addr));

		k_thread_create(&thread_node->thread, thread_node->stack,
				CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE, usbip_connection_thread,
				(void *)(uintptr_t)connfd, NULL, NULL, K_PRIO_COOP(4), 0,
				K_NO_WAIT);
	}
}

int usbip_add(struct usbip_dev *node)
{
	if (sys_dnode_is_linked(&node->_node)) {
		return -EINVAL;
	}

	sys_dlist_append(&usbip_dev_list, &node->_node);
	node->dev_num = atomic_inc(&dev_num);

	return 0;
}

int usbip_remove(struct usbip_dev *node)
{
	if (!sys_dnode_is_linked(&node->_node)) {
		return -EINVAL;
	}

	if (node->imported) {
		return -EBUSY;
	}

	sys_dlist_remove(&node->_node);

	return 0;
}

int usbip_recv(int fd, uint8_t *buf, size_t len)
{
	return recv(fd, buf, len, 0);
}

int usbip_send(int fd, const uint8_t *data, size_t len)
{
	return send(fd, data, len, 0);
}

bool usbip_send_common(int fd, uint8_t ep, uint32_t data_len)
{
	struct usbip_submit_rsp rsp;
	uint32_t ep_dir = USB_EP_DIR_IS_IN(ep) ? USBIP_DIR_IN : USBIP_DIR_OUT;
	uint32_t ep_idx = USB_EP_GET_IDX(ep);

	rsp.common.command = htonl(USBIP_RET_SUBMIT);
	rsp.common.seqnum = htonl(seqnum_global);
	rsp.common.devid = htonl(0);
	rsp.common.direction = htonl(ep_dir);
	rsp.common.ep = htonl(ep_idx);

	rsp.status = htonl(0);
	rsp.actual_length = htonl(data_len);
	rsp.start_frame = htonl(0);
	rsp.number_of_packets = htonl(0);
	rsp.error_count = htonl(0);

	rsp.setup = htonl(0);

	if (usbip_send(fd, (uint8_t *)&rsp, sizeof(rsp)) == sizeof(rsp)) {
		return true;
	}

	return false;
}

K_THREAD_DEFINE(usbip_server, CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE, usbip_server_thread, NULL,
		NULL, NULL, K_PRIO_COOP(2), 0, 0);
