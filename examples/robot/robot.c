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

static void pir_timer_cb(TimerHandle_t xTimer)
{
	leds_turn_off();
}

static void robot_task(void *pvParameters) {
	int pir_ev;

	on_pir_timer = xTimerCreate("on pir timer", 10000/portTICK_PERIOD_MS,
								pdFALSE, NULL, pir_timer_cb);
    if (on_pir_timer == NULL)
		goto end;

	while(!robot_task_end) {
		if(pir_wait_for_event(&pir_ev)) {
			printf("%s: ev at %i\n", __func__, pir_ev);
			if (xTimerStart(on_pir_timer, 0) != pdPASS) {
				printf("%s: failed to start timer\n", __func__);
			} else {
				//leds_turn_on(BLUE);
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
	http_server_init();
	xTaskCreate(&robot_task, "robot mngt", 256, NULL, 2, NULL);
}


