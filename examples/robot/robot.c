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
#include "l293d/l293d.h"

#include "leds.h"
#include "pir.h"
#include "nodemcu.h"
#include "utils.h"
#include "http_server.h"

#define US_ECHO_PIN		D0
#define US_TRIGGER_PIN	D1
#define US2_ECHO_PIN	D2
#define US2_TRIGGER_PIN	D3
#define LEDS_PIN		D4
#ifdef PIR
#define PIR_PIN			D6
#endif
#define M1_ENABLE_PIN	D9

#define US_MAX_DISTANCE_CM 500 // 5m max

static bool robot_main_task_end = false;
static bool robot_motorctrl_task_end = false;
static int32_t us_right_distance;
static int32_t us_left_distance;
#ifdef PIR
static bool pir_pending = false;
#endif

static int32_t get_distance_from_obstacle(ultrasonic_sensor_t *sensor)
{
	int32_t distance;
	taskENTER_CRITICAL();
	distance = ultrasoinc_measure_cm(sensor, US_MAX_DISTANCE_CM);
	taskEXIT_CRITICAL();

    if (distance < 0) {
		printf("Error: ");
		switch (distance)
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
		printf("%s : %d cm\n", __func__, distance);
#endif
	return distance;
}

static void robot_motorctrl_task(void *pvParameters) {
	ultrasonic_sensor_t us = {
		.trigger_pin = US_TRIGGER_PIN,
		.echo_pin = US_ECHO_PIN
	};
	ultrasonic_sensor_t us2 = {
		.trigger_pin = US2_TRIGGER_PIN,
		.echo_pin = US2_ECHO_PIN
	};
	struct l293d_device mc_dev = {
		.enable_1_pin = M1_ENABLE_PIN,
	};

	l293d_init(&mc_dev);
    ultrasoinc_init(&us);
    ultrasoinc_init(&us2);

	while(!robot_motorctrl_task_end) {
		us_right_distance = get_distance_from_obstacle(&us);
		us_left_distance = get_distance_from_obstacle(&us2);
		if (us_right_distance < 30 || us_left_distance < 30) {
				printf ("%s: us (left, right) = (%i, %i) cms\n", __func__,
						us_left_distance, us_right_distance);
		}
		vTaskDelay(10);
	}
}

#ifdef PIR
static void pir_timer_cb(TimerHandle_t xTimer)
{
	pir_pending = false;
}
#endif

static void robot_main_task(void *pvParameters) {
#ifdef PIR
	int pir_ev;
	TimerHandle_t on_pir_timer;
#endif
	int wbs_ev[2];

	uart_set_baud(0, 115200);
	leds_init(24, LEDS_PIN);
#ifdef PIR
	pir_init(PIR_PIN);
#endif
	xTaskCreate(&robot_motorctrl_task, "robot motor mngt", 256, NULL, 3, NULL);

	if (http_server_init()) {
		printf ("%s: failed to init httpd\n", __func__);
		goto end;
	}

#ifdef PIR
	on_pir_timer = xTimerCreate("on pir timer", 10000/portTICK_PERIOD_MS,
		pdFALSE, NULL, pir_timer_cb);
	if (on_pir_timer == NULL)
		goto end;
#endif
	while(!robot_main_task_end) {
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
#ifdef PIR
		if(pir_wait_for_event(&pir_ev)) {
			if (!pir_pending) {
				if (xTimerStart(on_pir_timer, 0) == pdPASS) {
					pir_pending = true;
					printf("%s: pir event at %i\n", __func__, pir_ev);
					/*if (!leds_is_on())
						leds_dimm();*/
				} else {
					printf("%s: failed to start timer\n", __func__);
				}
			}
		}
#endif
		vTaskDelay(10);
	}
end:
	vTaskDelete(NULL);
}

bool robot_get_leds_status(void)
{
	return leds_is_on();
}

void robot_get_us_distance(int32_t *left, uint32_t *right)
{
	*left = us_left_distance;
	*right = us_right_distance;
}

void user_init(void)
{
	xTaskCreate(&robot_main_task, "robot mngt", 256, NULL, 2, NULL);
}


