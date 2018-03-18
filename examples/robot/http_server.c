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
#include <httpd/httpd.h>
#include "queue.h"
#include "http_server.h"
#include "robot.h"
#include "trace.h"

enum {
    SSI_UPTIME,
    SSI_FREE_HEAP,
    SSI_LED_STATE
};

static QueueHandle_t wbs_queue;

static char *websocket_cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    return "/websockets.html";
}

static void websocket_task(void *pvParameter)
{
    struct tcp_pcb *pcb = (struct tcp_pcb *) pvParameter;

    for (;;) {
        if (pcb == NULL || pcb->state != ESTABLISHED) {
            INFO("Connection closed, deleting task\n");
            break;
        }

        int uptime = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
        int heap = (int) xPortGetFreeHeapSize();
        int led = (int)robot_get_leds_status();
        int us_left, us_right;
		robot_get_us_distance(&us_left, &us_right);

        /* Generate response in JSON format */
        char response[128];
        int len = snprintf(response, sizeof (response),
                "{\"uptime\" : \"%d\","
                " \"heap\" : \"%d\","
                " \"led\" : \"%d\","
				" \"us_left\" : \"%d\","
				" \"us_right\" : \"%d\"}",
				uptime, heap, led, us_left, us_right);
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
    uint16_t val = 0;
	int ev[2];

    switch (data[0]) {
		case 'B': // motor Backward
			ev[0] = WBS_MC_BACKWARD;
			xQueueSend(wbs_queue, ev, 0);
			break;
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
		case 'F': // motor Forward
			ev[0] = WBS_MC_FORWARD;
			xQueueSend(wbs_queue, ev, 0);
			break;
		case 'L': // motor Left
			ev[0] = WBS_MC_STEP_LEFT;
			xQueueSend(wbs_queue, ev, 0);
			break;
		case 'M': // Dimm LED
			ev[0] = WBS_LEDS_DIMM;
			ev[1] = atoi((char *)&data[1]);
			xQueueSend(wbs_queue, ev, 0);
            val = 0xBEEF;
            break;
		case 'P': // motor stoP
			ev[0] = WBS_MC_STOP;
			xQueueSend(wbs_queue, ev, 0);
			break;
		case 'R': // motor Right
			ev[0] = WBS_MC_STEP_RIGHT;
			xQueueSend(wbs_queue, ev, 0);
			break;
        case 'S': // Scroll LED
			ev[0] = WBS_LEDS_SCROLL;
			ev[1] = atoi((char *)&data[1]);
			xQueueSend(wbs_queue, ev, 0);
            val = 0xBEEF;
            break;
		case 'T': // Turn Left
			ev[0] = WBS_MC_TURN_LEFT;
			xQueueSend(wbs_queue, ev, 0);
			break;
		case 'U': // Turn Right
			ev[0] = WBS_MC_TURN_RIGHT;
			xQueueSend(wbs_queue, ev, 0);
			break;
		case 'W': // Power down;
			ev[0] = WBS_POWER_DOWN;
			xQueueSend(wbs_queue, ev, 0);
			break;
		default:
            INFO("%s: unknown command: %c\n", __func__, data[0]);
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
    INFO("WS URI: %s\n", uri);
    if (!strcmp(uri, "/stream")) {
        INFO("request for streaming\n");
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
        {"/websockets", (tCGIHandler) websocket_cgi_handler},
    };

    /* register handlers and start the server */
    http_set_cgi_handlers(pCGIs, sizeof (pCGIs) / sizeof (pCGIs[0]));
    websocket_register_callbacks((tWsOpenHandler) websocket_open_cb,
            (tWsHandler) websocket_cb);
    httpd_init();
    for (;;) {
		vTaskDelay(10000);
	}
}

int http_server_init(void)
{
	wbs_queue = xQueueCreate(10, sizeof(int) * 2);
	if (!wbs_queue)
		return -1;

    /* initialize tasks */
    xTaskCreate(&httpd_task, "HTTP Daemon", 256, NULL, 2, NULL);
	return 0;
}
