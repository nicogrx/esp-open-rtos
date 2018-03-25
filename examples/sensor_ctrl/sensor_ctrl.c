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
#include "esp/gpio.h"
#include <ssid_config.h>

#include "server.h"
#include "client.h"
#include "nodemcu.h"
#include "utils.h"
#include "json_parser.h"
#include "sensor_bmp280.h"
#include "server.h"
#include "ser2net.h"

#define DEBUG
#include "trace.h"

#define I2C_BUS 0

/* esp pins */
#define I2C_SCL_PIN		D3
#define I2C_SDA_PIN		D4

#define SERVER "192.168.1.6"
#define PORT "8080"

static const char *json_updt_dev_hdr = "/json.htm?type=command&param=udevice&idx=";
static const char *host_cmd_hdr = "/control?cmd=";
static bool sensor_ctrl_main_task_end = false;

char http_resp[MAX_OUT_CHARS];
char host_req[HOST_REQ_SIZE];

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

static int gpios[] = {
	D6,
	D7,
};

static int sensor_ctrl_gpio_set(int num, int val)
{
	int *gpio = gpios;

	while (*gpio != -1) {
		if (*gpio == num) {
			gpio_enable(num, GPIO_OUTPUT);
			gpio_write(num, (bool)val);
			return 0;
		}
		gpio++;
	}
	return -1;
}

static void sensor_ctrl_process_host_cmd(char * host_req)
{
	char *tmp;
	char *tmp2;
	int gpio_num;
	int gpio_val;
	char gpio_num_str[4];
	char gpio_val_str[2];

	int len = strlen(host_cmd_hdr);

	/* /control?cmd=GPIO,12,0 */

	if (strncmp(host_req, host_cmd_hdr, len)) {
		INFO("%s: invalid header\n", __func__);
		return;
	}

	tmp = host_req + len;

	if (!strncmp(tmp, "GPIO,", 5)) {
		tmp += 5;
		tmp2 = strchr(tmp, ',');
		if (tmp2) {
			len = tmp2 + 1 - tmp;
			snprintf(gpio_num_str, len > 3 ? 3: len, "%s", tmp);
			gpio_num = atoi(gpio_num_str);
			gpio_val_str[0] = *(tmp2 + 1);
			gpio_val_str[1] = '\0';
			gpio_val = atoi(gpio_val_str);
			INFO("%s: GPIO %d = %d\n", __func__, gpio_num, gpio_val);
			sensor_ctrl_gpio_set(gpio_num, gpio_val);
		}
	}

}

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
	ser2net_init(0, 1200, UART_BYTELENGTH_7, UART_STOPBITS_1, true,
				 UART_PARITY_EVEN);

	sensors_init(sensors);
	while(!sensor_ctrl_main_task_end) {
		sensors_refresh(sensors);
		if (server_wait_for_event(host_req)) {
			sensor_ctrl_process_host_cmd(host_req);
		}
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


