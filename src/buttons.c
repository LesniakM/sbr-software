
#include "buttons.h"

#define SW1_NODE DT_ALIAS(bt1)
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
// #define SW1_NODE DT_ALIAS(bt2)
// static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);

static struct gpio_callback button1_cb_data;
// static struct gpio_callback button2_cb_data;

void button1_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (gpio_pin_get_dt(&button1))
    {
        button1_pressed();
    }
    else
    {
        button1_released();
    }
}

void button1_pressed()
{
    printk("btn down cb\n");
    // drive = true;
}

void button1_released()
{
    printk("btn up cb\n");
    // drive = true;
}

/*
void button2_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    gpio_pin_set_dt(&led, 0);
    drive = false;
}
*/

void config_buttons_callbacks()
{
    gpio_pin_configure_dt(&button1, GPIO_INPUT);
    // gpio_pin_configure_dt(&button2, GPIO_INPUT);

    // Setup interrupts for pins:
    gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_BOTH);
    // gpio_pin_interrupt_configure_dt(&button2, GPIO_INT_EDGE_TO_ACTIVE);

    // Initialize the static struct gpio_callback variable
    gpio_init_callback(&button1_cb_data, button1_cb, BIT(button1.pin));
    // gpio_init_callback(&button2_cb_data, button2_pressed_cb, BIT(button2.pin));

    // Add the callback function by calling gpio_add_callback()
    gpio_add_callback(button1.port, &button1_cb_data);
    // gpio_add_callback(button2.port, &button2_cb_data);
}
