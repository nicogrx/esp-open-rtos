#include <stdio.h>

#include "espressif/esp_common.h"
#include "esp/uart.h"

#include "i2c/i2c.h"
#include "bmp280/bmp280.h"

//#define DEBUG
#include "trace.h"

static bmp280_t bmp280_dev;

int sensor_bmp280_init(void *private)
{
	bmp280_params_t  params;
	struct i2c_dev *i2c = (struct i2c_dev *)private;

	bmp280_init_default_params(&params);
	bmp280_dev.i2c_dev.bus = i2c->bus;
	bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;
	if (!bmp280_init(&bmp280_dev, &params)) {
		INFO("BMP280 initialization failed\n");
		return -1;
	}
	INFO("BMP280: found %s\n", bmp280_dev.id == BME280_CHIP_ID ?
			"BME280" : "BMP280");
	return 0;
}

int sensor_bmp280_refresh(char *out, int max_chars)
{
	bool bme280p = bmp280_dev.id == BME280_CHIP_ID;
	float pressure, temperature, humidity;

	if (!bmp280_read_float(&bmp280_dev, &temperature, &pressure, &humidity)) {
		INFO("Temperature/pressure reading failed\n");
		return -1;
	}
	INFO("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
	if (bme280p)
		INFO(", Humidity: %.2f\n", humidity);

	if (bme280p)
		snprintf(out, max_chars, "%.2f;%.2f;0;%.2f;0", temperature, humidity, pressure / 100);
	else
		snprintf(out, max_chars, "%.2f;%.2f;0;%.2f;0", temperature, humidity, pressure / 100);
	return 0;
}
