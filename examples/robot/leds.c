#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "ws2812.h"
#include "leds.h"
#include "utils.h"

enum {
	LEDS_ON,
	LEDS_OFF,
	LEDS_SCROLL,
	LEDS_DIMM,
};

static int number_of_leds = 12;
static uint8_t led_pin = 2;
static bool init_done = false;
static bool leds_task_end = false;
static QueueHandle_t leds_queue;
static bool leds_on = false;

static uint32_t color_on = WHITE;

static void leds_set_all(uint32_t c)
{
	if (!init_done)
		return;

	ws2812_seq_start();	
	for (int i = 0; i < number_of_leds; i++) {
		ws2812_seq_rgb(led_pin, c);
	}
	ws2812_seq_end();
}

static void leds_set_all_with_pattern(uint32_t c1, uint32_t c2, uint32_t p)
{
	if (!init_done)
		return;
	ws2812_seq_start();	
	for (int i = 0; i < number_of_leds; i++) {
		if ((p >> i) & 0x1)
			ws2812_seq_rgb(led_pin, c1);
		else
			ws2812_seq_rgb(led_pin, c2);
	}
	ws2812_seq_end();
}

#define PATTERN 0x3
/**
 * @brief scroll clock & counter clockwise for 
 * a couple of time
 * @time: time to scroll in ms.
 */
static void leds_do_scroll(uint32_t c1, uint32_t c2, int time)
{
	int timeout_ticks = xTaskGetTickCount() + (time / portTICK_PERIOD_MS);
	static uint32_t p;
	if (!init_done)
		return;

	while (xTaskGetTickCount() < timeout_ticks) {
		p = PATTERN;
		for (int i = 0; i < number_of_leds - 1; i++) {
			leds_set_all_with_pattern(c1, c2, p);
			delay_ms(100);
			p = p << 1;
		}
		for (int i = 0; i < number_of_leds; i++) {
			leds_set_all_with_pattern(c1, c2, p);
			delay_ms(100);
			p = p >> 1;
		}
	}
}

static void leds_do_dimm(void)
{
	uint32_t c;
	uint32_t r_msk = 0, g_msk = 0, b_msk = 0, msk = 0;

	if (color_on & 0xFF0000)
		r_msk = 0xFF0000;
	if (color_on & 0xFF00)
		g_msk = 0xFF00;
	if (color_on & 0xFF)
		b_msk = 0xFF;
	msk = r_msk | g_msk | b_msk;

	for (int i = 0; i < 3; i++) {
		for (c = 0; c < 100; c += 2) {
			leds_set_all(((c << 16) | (c << 8) | c) & msk);
			delay_ms(10);
		}
		for (c = 100; c > 0; c -= 2) {
			leds_set_all(((c << 16) | (c << 8) | c) & msk);
			delay_ms(10);
		}
	}
}

void leds_turn_on(uint32_t color)
{
	int ev;	
	color_on = color;
	if (!init_done)
		return;
	ev = LEDS_ON;
	xQueueSend(leds_queue, &ev, 0);
}

void leds_turn_off(void)
{
	int ev;	
	if (!init_done)
		return;
	ev = LEDS_OFF;
	xQueueSend(leds_queue, &ev, 0);
}

void leds_scroll(uint32_t color)
{
	int ev;
	color_on = color;
	if (!init_done)
		return;
	ev = LEDS_SCROLL;
	xQueueSend(leds_queue, &ev, 0);
}

void leds_dimm(void)
{
	int ev;	
	ev = LEDS_DIMM;
	xQueueSend(leds_queue, &ev, 0);	
}

static void leds_task(void *pvParameters) {
    QueueHandle_t queue = (QueueHandle_t)pvParameters;
	int ev;

	while(!leds_task_end) {
        xQueueReceive(queue, &ev, portMAX_DELAY);
		//printf("%s: ev:%i\n", __func__, ev);
		switch(ev) {
		case LEDS_ON:
			leds_on = true;
			leds_set_all(color_on);
			break;
		case LEDS_OFF:
			leds_on = false;
			leds_set_all(BLACK);
			break;
		case LEDS_SCROLL:
			leds_on = true;
			leds_do_scroll(color_on, BLACK, 5000);
			leds_on = false;
			break;
		case LEDS_DIMM:
			leds_on = true;
			leds_do_dimm();
			break;
		default:
			printf("unkown leds ev: %i\n", ev);
		}
	}
	vTaskDelete(NULL);
}

bool leds_is_on(void)
{
	return leds_on;
}

void leds_init(int nb_leds, uint8_t pin)
{
	if (init_done)
		return;
	number_of_leds = nb_leds;
	led_pin = pin;
	gpio_enable(led_pin, GPIO_OUTPUT);
	gpio_write(led_pin, 0);
    leds_queue = xQueueCreate(10, sizeof(int));
    xTaskCreate(&leds_task, "leds mngt", 256, leds_queue, 2, NULL);
	init_done = true;
	leds_turn_off();
}
