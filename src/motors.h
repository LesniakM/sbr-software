#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>

#define M_DRIVER_12_SLEEP_NODE DT_ALIAS(drv_slp_12)
static const struct gpio_dt_spec m_driver_12_sleep = GPIO_DT_SPEC_GET(M_DRIVER_12_SLEEP_NODE, gpios);

#define MAX_PULSE_NS 25000
#define MAX_ALLOWED_PULSE_NS_L 20000
#define MIN_WORKING_PULSE_NS_L 14000
#define MAX_ALLOWED_PULSE_NS_R 20000
#define MIN_WORKING_PULSE_NS_R 14000

/**
 * @brief Sets the PWM duty cycle for a specified channel.
 *
 * This function configures the PWM duty cycle for the given channel.
 * The duty cycle is clamped between 0 and 100%. The function asserts
 * that the channel index is within the valid range (0 to 3).
 *
 * @param channel   The PWM channel to configure (valid range: 0-3).
 * @param dutycycle The desired duty cycle percentage (0-100%).
 *
 * @note If the duty cycle exceeds 100%, it will be capped at 100%.
 * @note The function assumes that `pwm_channels` is properly initialized.
 */
void set_pwm_channel(uint8_t channel, uint8_t dutycycle);

/**
 * @brief Sets the speed and direction of the left motor.
 *
 * @param speed   The speed of the motor (0-100% duty cycle).
 * @param forward If true, the motor moves forward; otherwise, it moves in reverse.
 *
 * @note If the speed exceeds 100%, it will be capped at 100%.
 */
void set_l_motor_speed(uint8_t speed, bool forward);

/**
 * @brief Sets the speed and direction of the right motor.
 *
 * @param speed   The speed of the motor (0-100% duty cycle).
 * @param forward If true, the motor moves forward; otherwise, it moves in reverse.
 *
 * @note If the speed exceeds 100%, it will be capped at 100%.
 */
void set_r_motor_speed(uint8_t speed, bool forward);

/**
 * @brief Stops all motors by setting their PWM duty cycles to 0%.
 *
 * This function disables all motor movement by setting the duty cycle of
 * all PWM channels to 0, effectively stopping the motors.
 *
 *
 */
void stop_motors();

#endif // MOTORS_H