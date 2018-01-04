/**
 * @file   robot.c
 * @author Jean-Nicolas Graux
 *
 * @brief  my little robot.
 */

#include "espressif/esp_common.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "esp/uart.h"
#include <stdio.h>
#include <stdint.h>

#include "leds.h"
#include "pir.h"
#include "utils.h"
#include "http_server.h"

static bool robot_task_end = false;
static TimerHandle_t on_pir_timer;
static bool leds_on = false;

static void pir_timer_cb(TimerHandle_t xTimer)
{
	leds_turn_off();
}

static void robot_task(void *pvParameters) {
	int pir_ev;
	int wbs_ev[2];

	if (http_server_init()) {
		printf ("%s: failed to init httpd\n", __func__);
		goto end;
	}

	on_pir_timer = xTimerCreate("on pir timer", 10000/portTICK_PERIOD_MS,
								pdFALSE, NULL, pir_timer_cb);
    if (on_pir_timer == NULL)
		goto end;

	while(!robot_task_end) {

		if (websocket_wait_for_event(wbs_ev)) {
			switch(wbs_ev[0]) {
			case WBS_LEDS_ON:
				leds_turn_on((uint32_t)wbs_ev[1]);
				leds_on = true;
				break;
			case WBS_LEDS_OFF:
				leds_turn_off();
				leds_on = false;
				break;
			case WBS_LEDS_SCROLL:
				leds_scroll((uint32_t)wbs_ev[1]);
				break;
			case WBS_LEDS_DIMM:
				leds_dimm();
				break;
			default:
				printf("%s: unknown bws event: %i\n", __func__, wbs_ev[0]);
			}
		}

		if(pir_wait_for_event(&pir_ev)) {
			printf("%s: ev at %i\n", __func__, pir_ev);
			if (xTimerStart(on_pir_timer, 0) != pdPASS) {
				printf("%s: failed to start timer\n", __func__);
			} else if (leds_on) {
				leds_dimm();
			}
		}
		taskYIELD();
	}
end:
	vTaskDelete(NULL);
}

void user_init(void)
{
	uart_set_baud(0, 115200);
	leds_init(24, 2);
	pir_init(12);
	xTaskCreate(&robot_task, "robot mngt", 256, NULL, 2, NULL);
}


