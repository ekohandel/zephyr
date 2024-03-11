#ifndef ZEPHYR_DRIVERS_USB_USBIP_HOST_H
#define ZEPHYR_DRIVERS_USB_USBIP_HOST_H

#include <stdint.h>

#include "usbh_device.h"

#define USBIP_VERSION 0x0111

#define USBIP_OP_REQ_DEVLIST 0x8005
#define USBIP_OP_REP_DEVLIST 0x0005
#define USBIP_OP_REQ_IMPORT 0x8003
#define USBIP_OP_REP_IMPORT 0x0003

#define USBIP_DIR_OUT 0
#define USBIP_DIR_IN  1

#define USBIP_CMD_SUBMIT 0x00000001
#define USBIP_CMD_UNLINK 0x00000002
#define USBIP_RET_SUBMIT 0x00000003
#define USBIP_RET_UNLINK 0x00000004

#define USBIP_PATH_LEN 256
#define USBIP_BUSID_LEN 32

struct usbip_op_header {
	uint16_t version;
	uint16_t command;
	uint32_t status;
} __packed;

struct usbip_op_device {
	uint8_t path[USBIP_PATH_LEN];
	uint8_t busid[USBIP_BUSID_LEN];
	uint32_t busnum;
	uint32_t devnum;
	uint32_t speed;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t bcdDeviceClass;
	uint8_t bcdDeviceSubClass;
	uint8_t bcdDeviceProtocol;
	uint8_t bConfigurationValue;
	uint8_t bNumConfigurations;
	uint8_t bNumInterfaces;
} __packed;

struct usbip_op_interface {
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t padding;
} __packed;

struct usbip_op_req_import {
	uint8_t busid[USBIP_BUSID_LEN];
} __packed;

struct usbip_header_basic {
	uint32_t command;
	uint32_t seqnum;
	uint32_t devid;
	uint32_t direction;
	uint32_t ep;
} __packed;

struct usbip_cmd_submit {
	uint32_t transfer_flags;
	int32_t transfer_buffer_length;
	int32_t start_frame;
	int32_t number_of_packets;
	int32_t interval;
	union {
		uint8_t setup[8];
		struct {
			uint8_t bmRequestType;
			uint8_t bRequest;
			uint16_t wValue;
			uint16_t wIndex;
			uint16_t wLength;
		} __packed;
	} __packed;
} __packed;

struct usbip_cmd_unlink {
	uint32_t seqnum;

	uint8_t padding[24];
} __packed;

struct usbip_cmd {
	struct usbip_header_basic hdr;
	union {
		struct usbip_cmd_submit submit;
		struct usbip_cmd_unlink unlink;
	} __packed;
} __packed;

struct usbip_ret_unlink {
	int32_t status;
	uint8_t padding[24];
} __packed;

struct usbip_ret_submit {
	int32_t status;
	uint32_t actual_length;
	uint32_t start_frame;
	uint32_t number_of_packets;
	uint32_t error_count;
	uint8_t padding[8];
} __packed;

struct usbip_api {
	int (*imported)(const struct usb_device *udev, int fd);
	int (*rx_cmd)(const struct usb_device *udev, const struct usbip_cmd *cmd);
	int (*disconnected)(const struct usb_device *udev);
};

int usbip_add_device(struct usb_device *udev);
int usbip_remove_device(struct usb_device *udev);
int usbip_imported(const struct usb_device *udev, int fd);
int usbip_rx_cmd(const struct usb_device *udev, const struct usbip_cmd *cmd);
int usbip_disconnected(const struct usb_device *udev);
int usbip_rx(int fd, void *buf, size_t len);
int usbip_tx(int fd, const void *buf, size_t len);

#endif /* ZEPHYR_DRIVERS_USB_USBIP_HOST_H */