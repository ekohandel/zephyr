# Copyright (c) 2022 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
zephyr_library_include_directories(${ZEPHYR_BASE}/subsys/usb/host)

zephyr_library_sources(
	usbip.c
    host.c
)
