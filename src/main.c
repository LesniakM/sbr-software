#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>

#include "motors.h"
#include "mpu6050.h"
#include "buttons.h"

// USB
#include "usb_module.h"

// #include "ble_comm.h"
// #include <zephyr/sys/printk.h>

#define LED0_NODE DT_ALIAS(led0) // LED0_NODE = led0 defined in the .dts file
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define MPU6500_ADDRESS_NODE DT_ALIAS(mpu6500_address)
const struct gpio_dt_spec mpu6500_address_pin = GPIO_DT_SPEC_GET(MPU6500_ADDRESS_NODE, gpios);

int time = 0;

bool drive = false;

#define I2C0_MPU_NODE DT_NODELABEL(mpu6500)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_MPU_NODE);
uint8_t readbuffer[12] = {};
int16_t correction_values[6] = {0, 0, 0, 0, 0, 0};
int16_t raw_values[6] = {0, 0, 0, 0, 0, 0};
int64_t cumulated_values[6] = {0, 0, 0, 0, 0, 0};
float imu_values[5] = {0, 0, 0, 0, 0};
float a_angle = 0;
float g_angle = 0;

float balance_value = 0;
float balance_value_P = 0;
float balance_value_D = 0;

float kP = 6.001;
float kI = 0.0;
float kD = 2.205;
float kD_dumping = 0.85F;
float gyro_favor_factor = 0.98F;

float complementary_angle = 0;

uint8_t loop_time = 2;

int main(void)
{
    k_msleep(100);
    // init_ble_c();
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&mpu6500_address_pin, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&m_driver_12_sleep, GPIO_OUTPUT_ACTIVE);

    config_buttons_callbacks();

    mpu6050_init(&dev_i2c);

    k_msleep(500);

    if (usb_enable(usb_status_cb))
    {
        return 0;
    }

    // Poll if the DTR flag was set
    const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    uint32_t dtr = 1; // 0;
    while (!dtr)
    {
        uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
        // Give CPU resources to low priority threads.
        k_msleep(100);
    }

    printk("Acquring correction values...\n");
    for (int n = 0; n < 250; n++)
    {
        mpu6050_get_readings(&dev_i2c, readbuffer);
        k_msleep(3);
        for (int i = 0; i < 6; i++)
        {
            raw_values[i] = (int16_t)readbuffer[i * 2] * 256 + (int16_t)readbuffer[i * 2 + 1];
            cumulated_values[i] += raw_values[i];
        }
    }
    correction_values[4] = cumulated_values[4] / 250;
    correction_values[5] = cumulated_values[5] / 250;
    printk("Correction values: GX: %d, GY: %d\n", correction_values[4], correction_values[5]);
    gpio_pin_toggle_dt(&led);

    gpio_pin_set_dt(&m_driver_12_sleep, 1);

    gpio_pin_toggle_dt(&led);
    k_msleep(200);
    gpio_pin_toggle_dt(&led);
    k_msleep(200);
    gpio_pin_toggle_dt(&led);
    k_msleep(200);

    mpu6050_get_readings(&dev_i2c, readbuffer);
    for (int i = 0; i < 6; i++)
    {
        raw_values[i] = (int16_t)readbuffer[i * 2] * 256 + (int16_t)readbuffer[i * 2 + 1] - correction_values[i];
    }
    imu_values[0] = convert_accel(raw_values[0]);
    imu_values[1] = convert_accel(raw_values[1]);
    imu_values[2] = convert_accel(raw_values[2]);
    imu_values[3] = convert_gyro(raw_values[4]);
    imu_values[4] = convert_gyro(raw_values[5]);
    gpio_pin_toggle_dt(&led);
    printk("Hello. AX %d, AY %d, AZ %d, T%d, GX%d, GY%d\n", raw_values[0], raw_values[1], raw_values[2], raw_values[3], raw_values[4], raw_values[5]);
    printk("F: AX:%f, AY:%f, AZ:%f, GX:%f, GY:%f\n", (double)imu_values[0], (double)imu_values[1], (double)imu_values[2], (double)imu_values[3], (double)imu_values[4]);

    while (1)
    {
        mpu6050_get_readings(&dev_i2c, readbuffer);
        for (int i = 0; i < 6; i++)
        {
            raw_values[i] = (int16_t)readbuffer[i * 2] * 256 + (int16_t)readbuffer[i * 2 + 1] - correction_values[i];
        }
        imu_values[0] = convert_accel(raw_values[0]);
        imu_values[1] = convert_accel(raw_values[1]);
        imu_values[2] = convert_accel(raw_values[2]);
        imu_values[3] = convert_gyro(raw_values[4]);
        imu_values[4] = convert_gyro(raw_values[5]);

        a_angle = accel_angle(imu_values[2], imu_values[0]);
        g_angle = gyro_angle(imu_values[4], loop_time);

        complementary_angle = gyro_favor_factor * (complementary_angle + (g_angle * (1.00F / 100.0F))) + (1.00F - gyro_favor_factor) * a_angle;

        balance_value_P = complementary_angle;
        balance_value_D = (balance_value_D + complementary_angle) * kD_dumping;

        balance_value = balance_value_P * kP + balance_value_D * kD;

        if (balance_value > 100)
        {
            balance_value = 100;
        }
        if (balance_value < -100)
        {
            balance_value = -100;
        }

        printk("%f %f\n", a_angle, g_angle);

        if (balance_value >= 0.15F && drive)
        {
            if (complementary_angle < 0.0F)
            {
                balance_value_D = balance_value_D / 4;
                balance_value = balance_value / 2;
            }
            set_l_motor_speed((uint8_t)balance_value, true);
            set_r_motor_speed((uint8_t)balance_value, true);
        }
        else if (balance_value <= -0.15F && drive)
        {
            if (complementary_angle > 0.0F)
            {
                balance_value_D = balance_value_D / 4;
                balance_value = balance_value / 2;
            }
            set_l_motor_speed((uint8_t)-balance_value, false);
            set_r_motor_speed((uint8_t)-balance_value, false);
        }
        else
        {
            stop_motors();
        }

        k_msleep(loop_time);
        // printk("Angle: %f   %f   %f\n", a_angle, g_angle, balance_value);
    }
}
