#include <stdio.h>

#include "espressif/esp_common.h"
#include "esp/uart.h"

//#define DEBUG
#include "trace.h"

int sensor_voltage_refresh(char *out, int max_chars)
{
	float voltage;
	uint16_t val;

	val = sdk_system_adc_read();
	voltage = (val * 12.0) / 231.0;

	INFO("Voltage: %.2f V\n", voltage);
	snprintf(out, max_chars, "%.2f", voltage);
	return 0;
}
