#include "usb_module.h"

void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
    switch (status)
    {
    case USB_DC_CONNECTED:
        LOG_INF("USB Connected");
        // Perform actions when USB is plugged in
        break;

    case USB_DC_DISCONNECTED:
        LOG_INF("USB Disconnected");
        // Perform actions when USB is unplugged
        break;

    default:
        break;
    }
}