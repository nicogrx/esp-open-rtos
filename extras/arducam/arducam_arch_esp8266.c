/* 
 * The MIT License (MIT)
 * 
 * Copyright (c) 2015 Johan Kanflo (github.com/kanflo)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <FreeRTOS.h>
#include "task.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <esp/spi.h>
#include <i2c/i2c.h>
#include "arducam.h"
#include "arducam_arch.h"

static uint8_t _sensor_addr;

//#define CONFIG_VERIFY

static uint8_t arducam_spi_bus;
static uint8_t arducam_spi_cs;
static uint8_t arducam_i2c_bus;

static void spi_chip_select(uint8_t spi_cs);
static void spi_chip_unselect(uint8_t spi_cs);

void arducam_arch(sensor_model_t model,
					   uint8_t spi_bus, uint8_t spi_cs,
					   uint8_t i2c_bus)
{
	arducam_spi_bus = spi_bus;
	arducam_spi_cs = spi_cs;
	gpio_enable(arducam_spi_cs, GPIO_OUTPUT);
    gpio_write(arducam_spi_cs, 1);
	arducam_i2c_bus = i2c_bus;
	_sensor_addr = arducam(model);
}

void arducam_delay_ms(uint32_t delay)
{
	vTaskDelay(delay / portTICK_PERIOD_MS);
}

void arducam_spi_write(uint8_t address, uint8_t value)
{
#ifdef CONFIG_VERIFY
	static int counter = 0;
#endif // CONFIG_VERIFY

	uint8_t data[2] = {address, value};
	spi_chip_select(arducam_spi_cs);
	spi_transfer_16(arducam_spi_bus, data[0] << 8 | data[1]);
	spi_chip_unselect(arducam_spi_cs);
#ifdef CONFIG_VERIFY
	data[0] = arducam_spi_read(address & 0x7f);
//	printf("arducam_spi_write: [0x%02x] = 0x%02x\n", address & 0x7f, value);
	if (data[0] != value) {
		printf("arducam_spi_write: verify failed after %d for reg 0x%02x (0x%02x should be 0x%02x)\n", counter, address & 0x7f, data[0], value);
	}
	counter++;
#endif // CONFIG_VERIFY
}

uint8_t arducam_spi_read(uint8_t address)
{
	uint8_t data[2] = {address, 0x00};
	spi_chip_select(arducam_spi_cs);
	spi_transfer_8(arducam_spi_bus, data[0]);
//	spi_chip_unselect(arducam_spi_cs); // The HW SPI does this but things does not work if we do the same
//	spi_chip_select(arducam_spi_cs);
	data[1] = (uint8_t) spi_transfer_8(arducam_spi_bus, 0);
	spi_chip_unselect(arducam_spi_cs);
  	return data[1];
}

void arducam_spi_transfer_start(void)
{
	spi_chip_select(arducam_spi_cs);
	spi_transfer_8(arducam_spi_bus, BURST_FIFO_READ);
	spi_transfer_8(arducam_spi_bus, 0x0);
}

void arducam_spi_transfer_stop(void)
{
	spi_chip_unselect(arducam_spi_cs);
}

void arducam_spi_transfer(uint8_t *out_buf, uint8_t *in_buf, uint32_t size)
{
    spi_transfer(arducam_spi_bus, out_buf, in_buf, size, SPI_8BIT);
}

uint8_t arducam_i2c_write(uint8_t regID, uint8_t regDat)
{
	const uint8_t data[] = {regID, regDat};
	return i2c_slave_write(arducam_i2c_bus, _sensor_addr, 0, data, sizeof(data)) == 0;
}

uint8_t arducam_i2c_read(uint8_t regID, uint8_t* regDat)
{
	const uint8_t data[] = {regID};
	return i2c_slave_read(arducam_i2c_bus, _sensor_addr, data, regDat, 1) == 0;
}

uint8_t arducam_i2c_write16(uint8_t regID, uint16_t regDat)
{
	return 0;
}

uint8_t arducam_i2c_read16(uint8_t regID, uint16_t* regDat)
{
	return 0;
}

uint8_t arducam_i2c_word_write(uint16_t regID, uint8_t regDat)
{
	return 0;
}

uint8_t arducam_i2c_word_read(uint16_t regID, uint8_t* regDat)
{
	return 0;
}

int arducam_i2c_write_regs(const struct sensor_reg reglist[])
{
	uint16_t reg_addr = 0, reg_idx = 0;
	uint16_t reg_val = 0;
	const struct sensor_reg *next = reglist;

	while ((reg_addr != 0xff) | (reg_val != 0xff))
	{
		reg_addr = pgm_read_word(&next->reg);
		reg_val = pgm_read_word(&next->val);
		if (!arducam_i2c_write(reg_addr, reg_val)) {
			printf("arducam_i2c_write_regs failed at register %d\n", reg_idx);
			return 0;
		}
	   	next++;
	   	reg_idx++;
	}

	return 1;
}


int arducam_i2c_write_regs16(const struct sensor_reg reglist[])
{
	unsigned int reg_addr = 0, reg_val = 0;
	const struct sensor_reg *next = reglist;
	while ((reg_addr != 0xff) | (reg_val != 0xffff))
	{
		reg_addr = pgm_read_word(&next->reg);
		reg_val = pgm_read_word(&next->val);
		if (!arducam_i2c_write16(reg_addr, reg_val)) {
			return 0;
		}
	   	next++;
	}

	return 1;
}

int arducam_i2c_write_word_regs(const struct sensor_reg reglist[])
{
	unsigned int reg_addr = 0, reg_val = 0, reg_idx = 0;
	const struct sensor_reg *next = reglist;

	while ((reg_addr != 0xffff) | (reg_val != 0xff))
	{
		reg_addr = pgm_read_word(&next->reg);
		reg_val = pgm_read_word(&next->val);
		if (!arducam_i2c_write16(reg_addr, reg_val)) {
			printf("arducam_i2c_write_word_regs failed at register %d\n", reg_idx);
			return 0;
		}
	   	next++;
	   	reg_idx++;
	}

	return 1;
}

static void spi_chip_select(uint8_t spi_cs)
{
    gpio_write(spi_cs, 0);
}

static void spi_chip_unselect(uint8_t spi_cs)
{
    gpio_write(spi_cs, 1);
}
