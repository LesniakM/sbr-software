#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>

#include "motors.h"
#include "mpu6050.h"
#include "buttons.h"
// #include "ble_comm.h"

// #include <zephyr/sys/printk.h>

#define I2C0_NODE DT_NODELABEL(mpu6050)

extern const struct gpio_dt_spec led;

int8_t mpu_readings[8] = {0};

int16_t acc_y_val = 0;
int16_t acc_z_val = 0;
int16_t gyr_x_val = 0;

int16_t corr_acc_y = 500;
int16_t corr_acc_z = 1000;
int16_t corr_gyr_x = 590;

float balance_value = 0;
float balance_value_P = 0;
float balance_value_D = 0;

float kP = 6.001;
float kI = 0.0;
float kD = 8.205;
float kD_dumping = 0.85F;
float gyro_favor_factor = 0.98F;

float acc_angle = 0;
float gyr_angle = 0;
float complementary_angle = 0;

bool drive = false;

int main(void)
{
        static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);
        if (!device_is_ready(dev_i2c.bus))
        {
                // printk("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
                return -1;
        }

        if (!pwm_is_ready_dt(&pwm_low_speed_0))
        {
                // printk("Error: PWM device %s is not ready\n", pwm_low_speed_0.dev->name);
                return 0;
        }
        if (!pwm_is_ready_dt(&pwm_low_speed_1))
        {
                // printk("Error: PWM device %s is not ready\n", pwm_low_speed_0.dev->name);
                return 0;
        }

        config_buttons_callbacks();

        gpio_pin_toggle_dt(&led);
        k_msleep(200);
        gpio_pin_toggle_dt(&led);
        k_msleep(200);
        gpio_pin_toggle_dt(&led);
        k_msleep(200);

        mpu6050_init(&dev_i2c);

        // init_ble_c();

        k_msleep(1000);
        while (1)
        {
                // gpio_pin_set_dt(&led, drive);
                gpio_pin_toggle_dt(&led);
                k_msleep(200);
                /*
                100k, 1    sample,  2  bytes: <1  ms
                100k, 1    sample,  6  bytes: 1   ms
                100k, 1    sample,  10 bytes: 1-2 ms
                100k, 1    sample, 2x2 bytes: 1-2 ms
                100k, 100 samples,  10 bytes: 133 ms
                100k, 100 samples, 2x2 bytes: 122 ms
                100k, 100 samples,  8  bytes: 115 ms

                400k, 100 samples,  8  bytes: 42 ms

                100k, 50   samples, 6 bytes: 19  ms
                100k, 1000 samples, 6 bytes: 383 ms
                100k, 1000 samples, 6 bytes: 383 ms
                */

                mpu6050_get_readings(&dev_i2c, mpu_readings);
                acc_y_val = mpu_readings[0] * 256 + mpu_readings[1] + corr_acc_y;
                acc_z_val = mpu_readings[2] * 256 + mpu_readings[3] + corr_acc_z;
                gyr_x_val = mpu_readings[6] * 256 + mpu_readings[7] + corr_gyr_x;

                gyr_angle = gyro_angle(gyr_x_val, 10);
                acc_angle = accel_angle(acc_y_val, acc_z_val);

                complementary_angle = gyro_favor_factor * (complementary_angle + (gyr_angle * (1.00F / 100.0F))) + (1.00F - gyro_favor_factor) * acc_angle;

                k_msleep(9);

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

                if (balance_value >= 0.15F)
                {
                        if (complementary_angle < 0.0F && drive)
                        {
                                balance_value_D = balance_value_D / 4;
                                balance_value = balance_value / 2;
                        }
                        set_l_motor_speed(balance_value, true);
                        set_r_motor_speed(balance_value, true);
                }
                else if (balance_value <= -0.15F && drive)
                {
                        if (complementary_angle > 0.0F)
                        {
                                balance_value_D = balance_value_D / 4;
                                balance_value = balance_value / 2;
                        }
                        set_l_motor_speed(-balance_value, false);
                        set_r_motor_speed(-balance_value, false);
                }
                else
                {
                        motors_stop();
                }

                // printk("Notify return: %d. Value: %f, %d\n", send_angle_notify((int32_t)complementary_angle), complementary_angle, (int32_t)complementary_angle);

                // printk("%f, %f, %f, %f\n", complementary_angle, balance_value, gyr_angle, acc_angle);

                // gpio_pin_set_dt(&led, acc_y_val > 50);

                // printk("A: %d   Y: %d.%d m/s^2 (%d)   Z: %d.%d m/s^2 (%d)      \r", angle(acc_y_val) / 10, acc_y_val / 10, (acc_y_val)-acc_y_val / 10 * 10, acc_y_val, acc_z_val / 10, (acc_z_val)-acc_z_val / 10 * 10, acc_z_val);
        }
}
