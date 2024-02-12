/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/device.h"
#include "zephyr/net/buf.h"
#include <stdint.h>

#define USBIP_DEV_PATH_LEN   256
#define USBIP_DEV_BUS_ID_LEN 32
struct op_common {
	uint16_t version;
	uint16_t code;
	uint32_t status;
} __packed;

struct devlist_device {
	char path[USBIP_DEV_PATH_LEN];
	char busid[USBIP_DEV_BUS_ID_LEN];

	uint32_t busnum;
	uint32_t devnum;
	uint32_t speed;

	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;

	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bConfigurationValue;
	uint8_t bNumConfigurations;
	uint8_t bNumInterfaces;
} __packed;

struct devlist_interface {
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t padding; /* alignment */
} __packed;

#define OP_REQUEST (0x80 << 8)
#define OP_REPLY   (0x00 << 8)

/* Devlist */
#define OP_DEVLIST     0x05
#define OP_REQ_DEVLIST (OP_REQUEST | OP_DEVLIST)
#define OP_REP_DEVLIST (OP_REPLY | OP_DEVLIST)

/* Import USB device */
#define OP_IMPORT     0x03
#define OP_REQ_IMPORT (OP_REQUEST | OP_IMPORT)
#define OP_REP_IMPORT (OP_REPLY | OP_IMPORT)

/* USBIP requests */
#define USBIP_CMD_SUBMIT 0x0001
#define USBIP_CMD_UNLINK 0x0002
#define USBIP_RET_SUBMIT 0x0003
#define USBIP_RET_UNLINK 0x0004

/* USBIP direction */
#define USBIP_DIR_OUT 0x00
#define USBIP_DIR_IN  0x01

struct usbip_header_common {
	uint32_t command;
	uint32_t seqnum;
	uint32_t devid;
	uint32_t direction;
	uint32_t ep;
} __packed;

struct usbip_submit {
	uint32_t transfer_flags;
	int32_t transfer_buffer_length;
	int32_t start_frame;
	int32_t number_of_packets;
	int32_t interval;
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} __packed;

struct usbip_unlink {
	uint32_t seqnum;
} __packed;

struct usbip_submit_rsp {
	struct usbip_header_common common;

	int32_t status;
	int32_t actual_length;
	int32_t start_frame;
	int32_t number_of_packets;
	int32_t error_count;

	uint64_t setup;
} __packed;

struct usbip_header {
	struct usbip_header_common common;

	union {
		struct usbip_submit submit;
		struct usbip_unlink unlink;
	} u;
} __packed;

#include <zephyr/usb/usbd.h>
struct usbip_dev {
	const struct device *dev;

	struct usb_device_descriptor *dev_desc;
	struct usb_cfg_descriptor *cfg_desc;
	struct usb_if_descriptor *if_desc;

	int connfd;
	int dev_num;
	int speed_idx;
	int (*ctrl_ep_cb)(const struct device *dev, uint8_t ep, struct usb_setup_packet *setup);
	int (*data_ep_cb)(const struct device *dev, uint8_t ep, size_t length);

	bool imported;

	sys_dnode_t _node;
};

/* Function definitions */

int usbip_add(struct usbip_dev *node);
int usbip_remove(struct usbip_dev *node);

int usbip_recv(int fd, uint8_t *buf, size_t len);
bool usbip_send_common(int fd, uint8_t ep, uint32_t data_len);
int usbip_send(int fd, const uint8_t *data, size_t len);
