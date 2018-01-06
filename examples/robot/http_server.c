/*
 * HTTP server example.
 *
 * This sample code is in the public domain.
 */
#include <espressif/esp_common.h>
#include <esp8266.h>
#include <esp/uart.h>
#include <string.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <ssid_config.h>
#include <httpd/httpd.h>
#include "queue.h"
#include "http_server.h"
#include "robot.h"

#define LED_PIN 2

enum {
    SSI_UPTIME,
    SSI_FREE_HEAP,
    SSI_LED_STATE
};

static QueueHandle_t wbs_queue;

static int32_t ssi_handler(int32_t iIndex, char *pcInsert, int32_t iInsertLen)
{
    switch (iIndex) {
        case SSI_UPTIME:
            snprintf(pcInsert, iInsertLen, "%d",
                    xTaskGetTickCount() * portTICK_PERIOD_MS / 1000);
            break;
        case SSI_FREE_HEAP:
            snprintf(pcInsert, iInsertLen, "%d", (int) xPortGetFreeHeapSize());
            break;
        case SSI_LED_STATE:
            snprintf(pcInsert, iInsertLen, (GPIO.OUT & BIT(LED_PIN)) ? "Off" : "On");
            break;
        default:
            snprintf(pcInsert, iInsertLen, "N/A");
            break;
    }

    /* Tell the server how many characters to insert */
    return (strlen(pcInsert));
}

static char *gpio_cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    for (int i = 0; i < iNumParams; i++) {
        if (strcmp(pcParam[i], "on") == 0) {
            uint8_t gpio_num = atoi(pcValue[i]);
            gpio_enable(gpio_num, GPIO_OUTPUT);
            gpio_write(gpio_num, true);
        } else if (strcmp(pcParam[i], "off") == 0) {
            uint8_t gpio_num = atoi(pcValue[i]);
            gpio_enable(gpio_num, GPIO_OUTPUT);
            gpio_write(gpio_num, false);
        } else if (strcmp(pcParam[i], "toggle") == 0) {
            uint8_t gpio_num = atoi(pcValue[i]);
            gpio_enable(gpio_num, GPIO_OUTPUT);
            gpio_toggle(gpio_num);
        }
    }
    return "/index.ssi";
}

static char *about_cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    return "/about.html";
}

static char *websocket_cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    return "/websockets.html";
}

static void websocket_task(void *pvParameter)
{
    struct tcp_pcb *pcb = (struct tcp_pcb *) pvParameter;

    for (;;) {
        if (pcb == NULL || pcb->state != ESTABLISHED) {
            //printf("Connection closed, deleting task\n");
            break;
        }

        int uptime = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
        int heap = (int) xPortGetFreeHeapSize();
        int led = (int)robot_get_leds_status();
        int us_distance = (int)robot_get_us_distance();

        /* Generate response in JSON format */
        char response[96];
        int len = snprintf(response, sizeof (response),
                "{\"uptime\" : \"%d\","
                " \"heap\" : \"%d\","
                " \"led\" : \"%d\","
				" \"us_distance\" : \"%d\"}", uptime, heap, led, us_distance);
        if (len < sizeof (response))
            websocket_write(pcb, (unsigned char *) response, len, WS_TEXT_MODE);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

/**
 * This function is called when websocket frame is received.
 *
 * Note: this function is executed on TCP thread and should return as soon
 * as possible.
 */
static void websocket_cb(struct tcp_pcb *pcb, uint8_t *data, u16_t data_len, uint8_t mode)
{
    uint8_t response[2];
    uint16_t val;
	int ev[2];

    switch (data[0]) {
        case 'D': // Disable LED
			ev[0] = WBS_LEDS_OFF;
			xQueueSend(wbs_queue, ev, 0);
            val = 0xDEAD;
            break;
        case 'E': // Enable LED
			ev[0] = WBS_LEDS_ON;
			ev[1] = atoi((char *)&data[1]);
			xQueueSend(wbs_queue, ev, 0);
            val = 0xBEEF;
            break;
        case 'S': // Scroll LED
			ev[0] = WBS_LEDS_SCROLL;
			ev[1] = atoi((char *)&data[1]);
			xQueueSend(wbs_queue, ev, 0);
            val = 0xBEEF;
            break;
		case 'M': // Dimml LED
			ev[0] = WBS_LEDS_DIMM;
			ev[1] = atoi((char *)&data[1]);
			xQueueSend(wbs_queue, ev, 0);
            val = 0xBEEF;
            break;

        default:
            printf("%s: unknown command: %c\n", __func__, data[0]);
            val = 0;
            break;
    }

    response[1] = (uint8_t) val;
    response[0] = val >> 8;

    websocket_write(pcb, response, 2, WS_BIN_MODE);
}

/**
 * This function is called when new websocket is open and
 * creates a new websocket_task if requested URI equals '/stream'.
 */
static void websocket_open_cb(struct tcp_pcb *pcb, const char *uri)
{
    printf("WS URI: %s\n", uri);
    if (!strcmp(uri, "/stream")) {
        printf("request for streaming\n");
        xTaskCreate(&websocket_task, "websocket_task", 256, (void *) pcb, 2, NULL);
    }
}

bool websocket_wait_for_event(int *ev)
{
	if (xQueueReceive(wbs_queue, ev, 0) == pdFALSE)
		return false;
	return true;
}

static void httpd_task(void *pvParameters)
{
    tCGI pCGIs[] = {
        {"/gpio", (tCGIHandler) gpio_cgi_handler},
        {"/about", (tCGIHandler) about_cgi_handler},
        {"/websockets", (tCGIHandler) websocket_cgi_handler},
    };

    const char *pcConfigSSITags[] = {
        "uptime", // SSI_UPTIME
        "heap",   // SSI_FREE_HEAP
        "led"     // SSI_LED_STATE
    };

    /* register handlers and start the server */
    http_set_cgi_handlers(pCGIs, sizeof (pCGIs) / sizeof (pCGIs[0]));
    http_set_ssi_handler((tSSIHandler) ssi_handler, pcConfigSSITags,
            sizeof (pcConfigSSITags) / sizeof (pcConfigSSITags[0]));
    websocket_register_callbacks((tWsOpenHandler) websocket_open_cb,
            (tWsHandler) websocket_cb);
    httpd_init();
    for (;;) {
		vTaskDelay(10000);
	}
}

int http_server_init(void)
{
    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

	wbs_queue = xQueueCreate(10, sizeof(int) * 2);
	if (!wbs_queue)
		return -1;

    /* required to call wifi_set_opmode before station_set_config */
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);
    sdk_wifi_station_connect();

    /* initialize tasks */
    xTaskCreate(&httpd_task, "HTTP Daemon", 128, NULL, 2, NULL);
	return 0;
}
