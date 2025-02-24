#include "motors.h"
#include <assert.h>

#define MOTOR_PWM_NODE DT_NODELABEL(motors) // Reference the motors node
#define MOTOR_PWM_CTLR(idx) DT_PWMS_CTLR_BY_IDX(MOTOR_PWM_NODE, idx)
#define MOTOR_PWM_CHANNEL(idx) DT_PWMS_CHANNEL_BY_IDX(MOTOR_PWM_NODE, idx)
#define MOTOR_PWM_FLAGS(idx) DT_PWMS_FLAGS_BY_IDX(MOTOR_PWM_NODE, idx)

#define MOTOR_START_DC 50

const struct pwm_dt_spec pwm_channels[4] = {
    {.dev = DEVICE_DT_GET(MOTOR_PWM_CTLR(0)),
     .channel = MOTOR_PWM_CHANNEL(0),
     .period = PWM_KHZ(20),
     .flags = MOTOR_PWM_FLAGS(0)},
    {.dev = DEVICE_DT_GET(MOTOR_PWM_CTLR(1)),
     .channel = MOTOR_PWM_CHANNEL(1),
     .period = PWM_KHZ(20),
     .flags = MOTOR_PWM_FLAGS(1)},
    {.dev = DEVICE_DT_GET(MOTOR_PWM_CTLR(2)),
     .channel = MOTOR_PWM_CHANNEL(2),
     .period = PWM_KHZ(20),
     .flags = MOTOR_PWM_FLAGS(2)},
    {.dev = DEVICE_DT_GET(MOTOR_PWM_CTLR(3)),
     .channel = MOTOR_PWM_CHANNEL(3),
     .period = PWM_KHZ(20),
     .flags = MOTOR_PWM_FLAGS(3)}};

void set_pwm_channel(uint8_t channel, uint8_t dutycycle)
{
    assert(channel <= 3);
    if (dutycycle > 100)
    {
        dutycycle = 100;
    }
    pwm_set_dt(&pwm_channels[channel], PWM_KHZ(20), PWM_NSEC(500 * dutycycle));
}

void set_l_motor_speed(uint8_t speed, bool forward)
{
    if (speed > 100)
    {
        speed = 100;
    }
    if (forward)
    {
        set_pwm_channel(0, 0);
        set_pwm_channel(1, MOTOR_START_DC + speed / 2);
    }
    else
    {
        set_pwm_channel(1, 0);
        set_pwm_channel(0, MOTOR_START_DC + speed / 2);
    }
}

void set_r_motor_speed(uint8_t speed, bool forward)
{
    if (speed > 100)
    {
        speed = 100;
    }
    if (forward)
    {
        set_pwm_channel(3, 0);
        set_pwm_channel(2, MOTOR_START_DC + speed / 2);
    }
    else
    {
        set_pwm_channel(2, 0);
        set_pwm_channel(3, MOTOR_START_DC + speed / 2);
    }
}

void stop_motors()
{
    set_pwm_channel(0, 0);
    set_pwm_channel(1, 0);
    set_pwm_channel(2, 0);
    set_pwm_channel(3, 0);
}
