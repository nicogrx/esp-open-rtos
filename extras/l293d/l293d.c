/*
 * Driver for DC & stepper motor control, e.g. L293D and derivatives
 *
 * Part of esp-open-rtos
 * Copyright (C) 2018 Jean-Nicolas Graux <nicogrx@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */
#include <esp/gpio.h>
#include <espressif/esp_common.h>
#include <stdio.h>
#include "i2c/i2c.h"
#include "pwm/pwm.h"
#include "l293d.h"

int l293d_init(struct l293d_device *dev)
{
    uint8_t enable_pins[2];
	if (!dev)
		return -1;
	if (dev->init_done)
		return -1;

	switch (dev->mode) {
	case GPIO_ESP:
		gpio_enable(dev->input_a1_pin, GPIO_OUTPUT);
		gpio_enable(dev->input_a2_pin, GPIO_OUTPUT);
		gpio_enable(dev->input_b1_pin, GPIO_OUTPUT);
		gpio_enable(dev->input_b2_pin, GPIO_OUTPUT);
		gpio_write(dev->input_a1_pin, false);
		gpio_write(dev->input_a2_pin, false);
		gpio_write(dev->input_b1_pin, false);
		gpio_write(dev->input_b2_pin, false);
		enable_pins[0] = dev->enable_1_pin;
		enable_pins[1] = dev->enable_2_pin;
		pwm_init(2, enable_pins, false);
		pwm_set_freq(1000);
		pwm_stop();
		break;
	case GPIO_EXPANDER:
		if (!dev->i2c)
			return -1;
		if (!dev->gpio_write)
			return -1;
		dev->gpio_write(dev->i2c, dev->input_a1_pin, false);
		dev->gpio_write(dev->i2c, dev->input_a2_pin, false);
		dev->gpio_write(dev->i2c, dev->input_b1_pin, false);
		dev->gpio_write(dev->i2c, dev->input_b2_pin, false);
		dev->gpio_write(dev->i2c, dev->enable_1_pin, false);
		dev->gpio_write(dev->i2c, dev->enable_2_pin, false);
		break;
	default:
		return -1;
	}
	dev->init_done = true;
	return 0;
}


int l293d_dc_motors_stop(const struct l293d_device *dev)
{
	if (!dev || !dev->init_done)
		return -1;
	
	switch (dev->mode) {
	case GPIO_ESP:
		pwm_stop();
		break;
	case GPIO_EXPANDER:
		dev->gpio_write(dev->i2c, dev->enable_1_pin, false);
		dev->gpio_write(dev->i2c, dev->enable_2_pin, false);
		break;
	}
	return 0;
	
}

int l293d_dc_motors_start(const struct l293d_device *dev, uint8_t speed)
{
	static bool first_start = true;
	if (!dev || !dev->init_done)
		return -1;

	switch (dev->mode) {
	case GPIO_ESP:
    	pwm_set_duty(UINT16_MAX * speed / 255);
		if (first_start) {
			first_start = false;
			pwm_start();
		}
		break;
	case GPIO_EXPANDER:
		dev->gpio_write(dev->i2c, dev->enable_1_pin, true);
		dev->gpio_write(dev->i2c, dev->enable_2_pin, true);
		break;
	}
	return 0;
}

int l293d_dc_motor_rotate(const struct l293d_device *dev, int motor, int dir)
{
	uint8_t input1, input2;

	if (!dev || !dev->init_done)
		return -1;
	
	switch (motor) {
	case L293D_M1:
		input1 = dev->input_a1_pin;
		input2 = dev->input_a2_pin;
		break;
	case L293D_M2:
		input1 = dev->input_b1_pin;
		input2 = dev->input_b2_pin;
		break;
	default:
		return -1;
	}

	switch (dev->mode) {
	case GPIO_ESP:
		switch (dir) {
		case L293D_CLOCKWISE:
			gpio_write(input1, true);
			gpio_write(input2, false);
			break;
		case L293D_ANTI_CLOCKWISE:
			gpio_write(input1, false);
			gpio_write(input2, true);
			break;
		case L293D_IDLE_HI:
			gpio_write(input1, false);
			gpio_write(input2, false);
			break;
		case L293D_IDLE:
			gpio_write(input1, true);
			gpio_write(input2, true);
			break;
		default:
			return -1;
		}
		break;
	case GPIO_EXPANDER:
		switch (dir) {
		case L293D_CLOCKWISE:
			dev->gpio_write(dev->i2c, input1, true);
			dev->gpio_write(dev->i2c, input2, false);
			break;
		case L293D_ANTI_CLOCKWISE:
			dev->gpio_write(dev->i2c, input1, false);
			dev->gpio_write(dev->i2c, input2, true);
			break;
		case L293D_IDLE_HI:
			dev->gpio_write(dev->i2c, input1, false);
			dev->gpio_write(dev->i2c, input2, false);
			break;
		case L293D_IDLE:
			dev->gpio_write(dev->i2c, input1, true);
			dev->gpio_write(dev->i2c, input2, true);
			break;
		default:
			return -1;
		}
		break;
	}
	return 0;
}

