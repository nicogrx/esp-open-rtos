#include <stdio.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"
#include "trace.h"

static bool pir_end = false;
static SemaphoreHandle_t pir_sem;
static QueueHandle_t pir_queue;

static void pir_intr_handler(uint8_t gpio_num)
{
	static BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xSemaphoreGiveFromISR(pir_sem, &xHigherPriorityTaskWoken);
}

static void pir_task(void *pvParameters) {
	QueueHandle_t queue = (QueueHandle_t)pvParameters;
	int ticks;

	while (!pir_end) {
        xSemaphoreTake(pir_sem, portMAX_DELAY);
		INFO("%s: presence detected!\n", __func__);
		ticks = xTaskGetTickCount();
		xQueueSend(queue, &ticks, 0);
	};
	vTaskDelete(NULL);
}

bool pir_wait_for_event(int *ev)
{
	if (xQueueReceive(pir_queue, ev, 0) == pdFALSE)
		return false;
	return true;
}

int pir_init(uint8_t pin)
{
    pir_sem = xSemaphoreCreateBinary();
	if (!pir_sem)
		return -1;
	pir_queue = xQueueCreate(2, sizeof(int));
	if (!pir_queue)
		return -1;
    xTaskCreate(&pir_task, "pir mngt", 256, pir_queue, 2, NULL);
    gpio_enable(pin, GPIO_INPUT);
    gpio_set_interrupt(pin, GPIO_INTTYPE_EDGE_NEG, pir_intr_handler);
	return 0;
}
