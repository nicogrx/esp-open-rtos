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

#include "i2c/i2c.h"
#include <esp/spi.h>
#include "esp/uart.h"
#include "espressif/esp_common.h"
#include "l293d/l293d.h"
#include "pcf8574/pcf8574.h"
#include "ultrasonic/ultrasonic.h"

#include "access_point.h"
#include "cam.h"
#include "leds.h"
#include "pir.h"
#include "nodemcu.h"
#include "utils.h"
#include "http_server.h"

#define DEBUG
#include "trace.h"

#define US1
//#define US2

#define I2C_BUS 0
#define SPI_BUS 1
#define SPI_CS  D9

/* esp pins */
#define US_ECHO_PIN		D0
#define I2C_SCL_PIN		D1
#define I2C_SDA_PIN		D2
#define US_TRIGGER_PIN	D3
#define LEDS_PIN		D4
#define US_ECHO2_PIN	D9
#define US_TRIGGER2_PIN	D10

/* pcf8574 pins */
#define M1_ENABLE_PIN	0
#define M2_ENABLE_PIN	1
#define M1_A_PIN		2
#define M1_B_PIN		3
#define M2_A_PIN		4
#define M2_B_PIN		5
#define PIR_PIN			6

#define US_MAX_DISTANCE_CM 500 // 5m max

enum MC_EVENTS {
	MC_FORWARD,
	MC_BACKWARD,
	MC_LEFT,
	MC_RIGHT,
	MC_STOP,
	MC_STEP_LEFT,
	MC_STEP_RIGHT,
	MC_NO_EV,
};

static bool robot_main_task_end = false;
static bool robot_motorctrl_task_end = false;
static bool mc_step_pending = false;
static int32_t us_right_distance;
static int32_t us_left_distance;
#ifdef PIR
static bool pir_pending = false;
#endif

static QueueHandle_t mc_queue;

#define PCF8574_ADDR 0x25
static i2c_dev_t pcf8574_dev = {
	.bus = I2C_BUS,
	.addr = PCF8574_ADDR
};
#ifdef US1
static ultrasonic_sensor_t us = {
		.trigger_pin = US_TRIGGER_PIN,
		.echo_pin = US_ECHO_PIN
};
#endif
#ifdef US2
static ultrasonic_sensor_t us2 = {
	.trigger_pin = US_TRIGGER2_PIN,
	.echo_pin = US_ECHO2_PIN
};
#endif

static struct l293d_device mc_dev = {
	.enable_1_pin = M1_ENABLE_PIN,
	.enable_1_pin = M2_ENABLE_PIN,
	.input_a1_pin = M1_A_PIN,
	.input_a2_pin = M1_B_PIN,
	.input_b1_pin = M2_A_PIN,
	.input_b2_pin = M2_B_PIN,
	.mode = GPIO_EXPANDER,
	.i2c = &pcf8574_dev,
	.gpio_write = pcf8574_gpio_write,
};

static const spi_settings_t spi_config = {
    .endianness = SPI_BIG_ENDIAN,
    .msb = true,
    .minimal_pins = true,
    .mode = SPI_MODE0,
    .freq_divider = SPI_FREQ_DIV_2M
};

static int32_t get_distance_from_obstacle(ultrasonic_sensor_t *sensor)
{
	int32_t distance;
	taskENTER_CRITICAL();
	distance = ultrasoinc_measure_cm(sensor, US_MAX_DISTANCE_CM);
	taskEXIT_CRITICAL();

    if (distance < 0) {
		INFO("Error: ");
		switch (distance)
		{
		case ULTRASONIC_ERROR_PING:
			INFO("Cannot ping (device is in invalid state)\n");
			break;
		case ULTRASONIC_ERROR_PING_TIMEOUT:
			INFO("Ping timeout (no device found)\n");
			break;
		case ULTRASONIC_ERROR_ECHO_TIMEOUT:
			INFO("Echo timeout (i.e. distance too big)\n");
			break;
		}
	}
#if 0
	else
		INFO("%s : %d cm\n", __func__, distance);
#endif
	return distance;
}

static void mc_step_timer_cb(TimerHandle_t xTimer)
{
	l293d_dc_motors_stop(&mc_dev);
	mc_step_pending = false;
}

static void robot_motorctrl_task(void *pvParameters) {
	static bool obstacle = false;
	int ev, last_ev = MC_NO_EV;
	TimerHandle_t on_mc_step_timer;

	if (l293d_init(&mc_dev))
		goto end;

#ifdef US1
    ultrasoinc_init(&us);
#endif
#ifdef US2
    ultrasoinc_init(&us2);
#endif

    mc_queue = xQueueCreate(2, sizeof(int));

	on_mc_step_timer = xTimerCreate("on mc step timer", 100/portTICK_PERIOD_MS,
		pdFALSE, NULL, mc_step_timer_cb);
	if (on_mc_step_timer == NULL)
		goto end;

	while(!robot_motorctrl_task_end) {
#ifdef US1
		us_right_distance = get_distance_from_obstacle(&us);
#else
		us_right_distance = 54321;
#endif
#ifdef US2
		us_left_distance = get_distance_from_obstacle(&us2);
#else
		us_left_distance = 54321;
#endif
		if (us_right_distance < 30 || us_left_distance < 30) {
				INFO ("%s: us (left, right) = (%i, %i) cms\n", __func__,
						us_left_distance, us_right_distance);
					if(last_ev == MC_FORWARD)
						l293d_dc_motors_stop(&mc_dev);
					obstacle = true;
		} else {
			obstacle = false;
		}

		if (mc_step_pending)
			goto wait;
		if (xQueueReceive(mc_queue, &ev, 0) == pdFALSE)
			goto wait;

		switch (ev) {
		case MC_FORWARD:
			if (!obstacle) {
				l293d_dc_motor_rotate(&mc_dev, L293D_M1, L293D_CLOCKWISE);
				l293d_dc_motor_rotate(&mc_dev, L293D_M2, L293D_CLOCKWISE);
				l293d_dc_motors_start(&mc_dev, 1);
			}
			break;
		case MC_BACKWARD:
			/* dangerous since no way to detect obstacle */
			l293d_dc_motor_rotate(&mc_dev, L293D_M1, L293D_ANTI_CLOCKWISE);
			l293d_dc_motor_rotate(&mc_dev, L293D_M2, L293D_ANTI_CLOCKWISE);
			l293d_dc_motors_start(&mc_dev, 1);
			break;
		case MC_STEP_LEFT:
			if (xTimerStart(on_mc_step_timer, 0) == pdPASS) {
					mc_step_pending = true;
			} else {
				INFO("%s: failed to start timer\n", __func__);
			}
			/* fall through */
		case MC_LEFT:
			l293d_dc_motor_rotate(&mc_dev, L293D_M1, L293D_CLOCKWISE);
			l293d_dc_motor_rotate(&mc_dev, L293D_M2, L293D_ANTI_CLOCKWISE);
			l293d_dc_motors_start(&mc_dev, 1);
			break;
		case MC_STEP_RIGHT:
			if (xTimerStart(on_mc_step_timer, 0) == pdPASS) {
					mc_step_pending = true;
			} else {
				INFO("%s: failed to start timer\n", __func__);
			}
			/* fall through */
		case MC_RIGHT:
			l293d_dc_motor_rotate(&mc_dev, L293D_M1, L293D_ANTI_CLOCKWISE);
			l293d_dc_motor_rotate(&mc_dev, L293D_M2, L293D_CLOCKWISE);
			l293d_dc_motors_start(&mc_dev, 1);
			break;
		case MC_STOP:
			l293d_dc_motors_stop(&mc_dev);
			break;
		default:
			break;
		}
		last_ev = ev;
wait:
		vTaskDelay(10);
	}
end:
	vTaskDelete(NULL);
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
	int mc_ev;
	int wbs_ev[2];

	uart_set_baud(0, 115200);
    i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ_100K);
	if (!spi_set_settings(SPI_BUS, &spi_config)) {
		INFO("%s: failed to init SPI\n", __func__);
		goto end;
	}
	leds_init(24, LEDS_PIN);
#ifdef PIR
	pir_init(PIR_PIN);
#endif
	xTaskCreate(&robot_motorctrl_task, "robot motor mngt", 256, NULL, 3, NULL);

	if (http_server_init()) {
		INFO ("%s: failed to init httpd\n", __func__);
		goto end;
	}
	if (!cam_setup(SPI_BUS, SPI_CS, I2C_BUS))
		goto end;
	access_point_init();

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
			case WBS_MC_FORWARD:
				mc_ev = MC_FORWARD;
				xQueueSend(mc_queue, &mc_ev, 0);
				break;
			case WBS_MC_BACKWARD:
				mc_ev = MC_BACKWARD;
				xQueueSend(mc_queue, &mc_ev, 0);
				break;
			case WBS_MC_STOP:
				mc_ev = MC_STOP;
				xQueueSend(mc_queue, &mc_ev, 0);
				break;
			case WBS_MC_LEFT:
				mc_ev = MC_STEP_LEFT;
				xQueueSend(mc_queue, &mc_ev, 0);
				break;
			case WBS_MC_RIGHT:
				mc_ev = MC_STEP_RIGHT;
				xQueueSend(mc_queue, &mc_ev, 0);
				break;
			default:
				INFO("%s: unknown wbs event: %i\n", __func__, wbs_ev[0]);
			}
		}
#ifdef PIR
		if(pir_wait_for_event(&pir_ev)) {
			if (!pir_pending) {
				if (xTimerStart(on_pir_timer, 0) == pdPASS) {
					pir_pending = true;
					INFO("%s: pir event at %i\n", __func__, pir_ev);
					/*if (!leds_is_on())
						leds_dimm();*/
				} else {
					INFO("%s: failed to start timer\n", __func__);
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


