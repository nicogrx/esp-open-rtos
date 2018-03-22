/**
 * @file   sensor_ctrl.c
 * @author Jean-Nicolas Graux
 *
 * @brief  my little sensor controller.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "i2c/i2c.h"
#include "esp/uart.h"
#include "espressif/esp_common.h"
#include <ssid_config.h>

#include "server.h"
#include "client.h"
#include "nodemcu.h"
#include "utils.h"
#include "json_parser.h"

#define DEBUG
#include "trace.h"

#define I2C_BUS 0

/* esp pins */
#define I2C_SCL_PIN		D3
#define I2C_SDA_PIN		D4

#define SERVER "192.168.1.6"
#define PORT "8080"

static const char *json_req = "/json.htm?type=command&param=getuservariable&idx=7";
static bool sensor_ctrl_main_task_end = false;
char http_resp[MAX_OUT_CHARS];
#define STRING_SIZE 32
char string_val[STRING_SIZE];

static void sensor_ctrl_main_task(void *pvParameters)
{
	int s;

	sdk_system_update_cpu_freq(160);
	uart_set_baud(0, 115200);
	i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ_100K);
	
	delay_ms(10000);
	server_init();
	s = client_open(SERVER, PORT);
	if (s < 0)
		goto end;
	if(client_http_get(s, json_req, http_resp)) {
		client_close(s);
		goto end;
	}
	client_close(s);
	if (!json_get_value_from_key(http_resp, "Value", string_val, STRING_SIZE))
		INFO("Value = %s\n", string_val);

	while(!sensor_ctrl_main_task_end) {
		vTaskDelay(10);
	}
end:
	vTaskDelete(NULL);
}

void user_init(void)
{
	struct sdk_station_config config = {
		.ssid = WIFI_SSID,
		.password = WIFI_PASS,
	};

	/* required to call wifi_set_opmode before station_set_config */
	sdk_wifi_set_opmode(STATION_MODE);
	sdk_wifi_station_set_config(&config);
	sdk_wifi_station_connect();
	xTaskCreate(&sensor_ctrl_main_task, "sensor_ctrl mngt", 512, NULL, 2, NULL);
}


