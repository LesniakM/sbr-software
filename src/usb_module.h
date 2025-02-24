#ifndef USB_MODULE_H
#define USB_MODULE_H

#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

/*
prj.conf:
REQUIRED:
CONFIG_USB_DEVICE_STACK=y
OPTIONAL:
CONFIG_USB_DEVICE_PRODUCT="SBR with NiceNano v.2"
*/

LOG_MODULE_REGISTER(usb_detect, LOG_LEVEL_INF);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
			 "Console device is not ACM CDC UART device");

void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param);

#endif // USB_MODULE_H