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
#include "esp/uart.h"
#include <stdio.h>
#include <stdint.h>

#include "leds.h"
#include "pir.h"
#include "utils.h"
#include "http_server.h"

static bool robot_task_end = false;

static void robot_task(void *pvParameters) {
	while(!robot_task_end) {
		pir_wait_for_event();
		leds_turn_on(BLUE);
		delay_ms(1000);
		leds_turn_off();
	}
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


