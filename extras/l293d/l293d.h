/*
 * Driver for stepper motor, e.g. L293D and derivatives
 *
 * Part of esp-open-rtos
 * Copyright (C) 2018 Jean-Nicolas Graux <nicogrx@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */
#ifndef EXTRAS_L293D_H_
#define EXTRAS_L293D_H_

enum dc_motor {
	L293D_M1,
	L293D_M2
};

enum dc_motor_direction {
	L293D_IDLE_HI,
	L293D_IDLE,
	L293D_CLOCKWISE,
	L293D_ANTI_CLOCKWISE
};

enum gpio_mode {
	GPIO_ESP,
	GPIO_EXPANDER,
};

/*
 * l293d device struct
 */
struct l293d_device
{
	bool init_done;
    uint8_t enable_1_pin;
    uint8_t input_a1_pin;
    uint8_t input_a2_pin;
    uint8_t enable_2_pin;
    uint8_t input_b1_pin;
    uint8_t input_b2_pin;
	uint8_t mode;
	i2c_dev_t *i2c;
	void (* gpio_write)(i2c_dev_t *dev,
						uint8_t num,
						bool value);
};

int l293d_init(struct l293d_device *dev);
int l293d_dc_motors_start(const struct l293d_device *dev, uint8_t speed);
int l293d_dc_motors_stop(const struct l293d_device *dev);
int l293d_dc_motor_rotate(const struct l293d_device *dev, int motor, int dir);

#endif
