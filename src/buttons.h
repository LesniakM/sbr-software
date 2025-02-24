#ifndef BUTTONS_H
#define BUTTONS_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/sys/printk.h>

extern bool drive;

void button1_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

// void button2_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

void config_buttons_callbacks();

#endif // BUTTONS_H