/**
 * @file   robot.c
 * @author Jean-Nicolas Graux
 *
 * @brief  my little robot.
 */
#include <stdio.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "esp/uart.h"
#include "espressif/esp_common.h"
#include "ultrasonic/ultrasonic.h"

#include "leds.h"
#include "pir.h"
#include "utils.h"
#include "http_server.h"

#define LEDS_PIN		2
#define US_ECHO_PIN		4
#define US_TRIGGER_PIN	5
#define PIR_PIN			12

#define US_MAX_DISTANCE_CM 500 // 5m max

static bool robot_task_end = false;
static SemaphoreHandle_t us_sem;
static int32_t us_distance;
static void pir_timer_cb(TimerHandle_t xTimer)
{
	leds_turn_off();
}

static void us_timer_cb(TimerHandle_t xTimer)
{
	xSemaphoreGive(us_sem);
}

static int32_t get_distance_from_obstacle(ultrasonic_sensor_t *sensor)
{
	taskENTER_CRITICAL();
	us_distance = ultrasoinc_measure_cm(sensor, US_MAX_DISTANCE_CM);
	taskEXIT_CRITICAL();

    if (us_distance < 0) {
		printf("Error: ");
		switch (us_distance)
		{
		case ULTRASONIC_ERROR_PING:
			printf("Cannot ping (device is in invalid state)\n");
			break;
		case ULTRASONIC_ERROR_PING_TIMEOUT:
			printf("Ping timeout (no device found)\n");
			break;
		case ULTRASONIC_ERROR_ECHO_TIMEOUT:
			printf("Echo timeout (i.e. distance too big)\n");
			break;
		}
	}
#if 0
	else
		printf("%s : %d cm\n", __func__, us_distance);
#endif
	return us_distance;
}

static void robot_task(void *pvParameters) {
	int pir_ev;
	int wbs_ev[2];
	TimerHandle_t on_pir_timer, us_timer;
	ultrasonic_sensor_t us = {
        .trigger_pin = US_TRIGGER_PIN,
        .echo_pin = US_ECHO_PIN
    };

    ultrasoinc_init(&us);

	if (http_server_init()) {
		printf ("%s: failed to init httpd\n", __func__);
		goto end;
	}

	on_pir_timer = xTimerCreate("on pir timer", 10000/portTICK_PERIOD_MS,
								pdFALSE, NULL, pir_timer_cb);
    if (on_pir_timer == NULL)
		goto end;

	us_timer = xTimerCreate("ultrasonic timer", 20/portTICK_PERIOD_MS,
								pdTRUE, NULL, us_timer_cb);
    if (us_timer == NULL)
		goto end;

	us_sem = xSemaphoreCreateBinary();
	if (!us_sem)
		goto end;

	if (xTimerStart(us_timer, 0) != pdPASS) {
		printf("%s: failed to start timer\n", __func__);
		goto end;
	}

	while(!robot_task_end) {

        if (xSemaphoreTake(us_sem, 0) == pdTRUE) {
			get_distance_from_obstacle(&us);
			/*if (us_distance) < 20)
				printf ("%s: TODO: stop if moving forward\n", __func__);*/
		}

		if (websocket_wait_for_event(wbs_ev)) {
			switch(wbs_ev[0]) {
			case WBS_LEDS_ON:
				leds_turn_on((uint32_t)wbs_ev[1]);
				break;
			case WBS_LEDS_OFF:
				leds_turn_off();
				break;
			case WBS_LEDS_SCROLL:
				leds_scroll((uint32_t)wbs_ev[1]);
				break;
			case WBS_LEDS_DIMM:
				leds_dimm();
				break;
			default:
				printf("%s: unknown wbs event: %i\n", __func__, wbs_ev[0]);
			}
		}

		if(pir_wait_for_event(&pir_ev)) {
			printf("%s: pir event at %i\n", __func__, pir_ev);
			if (xTimerStart(on_pir_timer, 0) != pdPASS) {
				printf("%s: failed to start timer\n", __func__);
			} else if (!leds_is_on()) {
				leds_dimm();
			}
		}
		taskYIELD();
	}
end:
	vTaskDelete(NULL);
}

bool robot_get_leds_status(void)
{
	return leds_is_on();
}

int32_t robot_get_us_distance(void)
{
	return us_distance;
}

void user_init(void)
{
	uart_set_baud(0, 115200);
	leds_init(24, LEDS_PIN);
	pir_init(PIR_PIN);
	xTaskCreate(&robot_task, "robot mngt", 256, NULL, 2, NULL);
}


