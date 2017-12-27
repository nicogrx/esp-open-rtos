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
#include "utils.h"
#include "http_server.h"

void user_init(void)
{
	uart_set_baud(0, 115200);

	http_server_init();
	leds_init(12, 2);
}


