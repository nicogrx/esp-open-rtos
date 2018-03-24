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
#include "sensor_bmp280.h"

#define DEBUG
#include "trace.h"

#define I2C_BUS 0

/* esp pins */
#define I2C_SCL_PIN		D3
#define I2C_SDA_PIN		D4

#define SERVER "192.168.1.6"
#define PORT "8080"

static const char *json_updt_dev_hdr = "/json.htm?type=command&param=udevice&idx=";
static bool sensor_ctrl_main_task_end = false;

char http_resp[MAX_OUT_CHARS];

#define STRING_SIZE 32
char string_val[STRING_SIZE];
#define SENSOR_DATA_SIZE 80
char sensor_data[SENSOR_DATA_SIZE];
#define JSON_REQ_SIZE 128
char json_req[JSON_REQ_SIZE];

struct sensor {
	int idx; /* index in domoticz */
	int period; /* in seconds */
	int timeout_ticks;
	int (*init)(void *private);
	int (*refresh)(char *data, int max_chars);
	void (*destroy)(void);
	void *private;
};

static struct i2c_dev bmp280_i2c = {
	.bus = I2C_BUS,
};

static struct sensor sensors[] = {
	{
		.idx = 210,
		.period = 10,
		.init = sensor_bmp280_init,
		.refresh = sensor_bmp280_refresh,
		.destroy = NULL,
		.private = (void *)&bmp280_i2c,
	},
	{
		.period = 0, /* dummy sensor marking end of list */
	},
};

static inline void sensor_reload(struct sensor *s, int ticks)
{
	s->timeout_ticks = ticks + (s->period * 1000) / portTICK_PERIOD_MS;
}

void sensors_init(struct sensor *sensors)
{
	struct sensor *s = sensors;
	int ticks = xTaskGetTickCount();

	while(s->period != 0) {
		s->init(s->private);
		sensor_reload(s, ticks);
		s++;
	}
}

void sensors_destroy(struct sensor *sensors)
{
	struct sensor *s = sensors;

	while(s->period != 0) {
		s->destroy();
		s++;
	}
}

void sensors_refresh(struct sensor *sensors)
{
	struct sensor *s = sensors;
	int ticks = xTaskGetTickCount();
	int sock = -1;

	while(s->period != 0) {
		if (ticks >= s->timeout_ticks) {
			if (sock == -1) {
				sock = client_open(SERVER, PORT);
				if (sock < 0) {
					return;
				}
			}
			s->refresh(sensor_data, SENSOR_DATA_SIZE);
			snprintf(json_req, JSON_REQ_SIZE, "%s%d&nvalue=0&svalue=%s", json_updt_dev_hdr,
					s->idx, sensor_data);
			client_http_get(sock, json_req, http_resp);
			sensor_reload(s, ticks);
		}
		s++;
	}
	if (sock != - 1)
		client_close(sock);
}

static void sensor_ctrl_main_task(void *pvParameters)
{
	sdk_system_update_cpu_freq(160);
	uart_set_baud(0, 115200);
	i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ_100K);
	
	delay_ms(10000);
	server_init();

	sensors_init(sensors);
	while(!sensor_ctrl_main_task_end) {
		sensors_refresh(sensors);
		vTaskDelay(10);
	}
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


