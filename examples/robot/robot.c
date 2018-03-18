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
#include <ssid_config.h>
#include "l293d/l293d.h"
#include "pcf8574/pcf8574.h"
#include "ultrasonic/ultrasonic.h"

#include "server.h"
#include "cam.h"
#include "leds.h"
#include "pir.h"
#include "nodemcu.h"
#include "utils.h"
#include "http_server.h"

#define DEBUG
#include "trace.h"

#define SLEEP_TIME (60 * 1000000)

#define US1
#define US2

#define I2C_BUS 0
#define SPI_BUS 1
#define SPI_CS  D9

/* esp pins */
#define US_ECHO_PIN		D1
#define US_TRIGGER_PIN	D2
#define I2C_SCL_PIN		D3
#define I2C_SDA_PIN		D4
#define NEOPIXELS_PIN	D4
#define US_ECHO2_PIN	D8
#define US_TRIGGER2_PIN	D10

/* pcf8574 pins */
#define M1_ENABLE_PIN	0
#define M2_ENABLE_PIN	1
#define M1_A_PIN		2
#define M1_B_PIN		3
#define M2_A_PIN		4
#define M2_B_PIN		5
#define PIR_PIN			6
#define LEDS_PIN		7

#define US_MAX_DISTANCE_CM 500 // 5m max

enum MC_EVENTS {
	MC_FORWARD,
	MC_BACKWARD,
	MC_LEFT,
	MC_RIGHT,
	MC_STOP,
	MC_STEP_LEFT,
	MC_STEP_RIGHT,
	MC_TURN_LEFT,
	MC_TURN_RIGHT,
	MC_NO_EV,
};

static bool light_on = false;
static bool robot_main_task_end = false;
static bool robot_motorctrl_task_end = false;
static volatile bool robot_motorctrl_task_ended = false;
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
	.freq_divider = SPI_FREQ_DIV_8M
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

static void robot_go_timed(int ev, TimerHandle_t on_mc_step_timer, bool timed, TickType_t time)
{
	if (timed) {
		if (xTimerStart(on_mc_step_timer, 0) == pdPASS) {
			mc_step_pending = true;
		} else {
			INFO("%s: failed to start timer\n", __func__);
		}
		if (time) {
			if (xTimerChangePeriod(on_mc_step_timer, time, 0) != pdPASS)
				INFO("%s: failed to update timer period\n", __func__);
		}
	}
	switch(ev) {
	case MC_FORWARD:
		l293d_dc_motor_rotate(&mc_dev, L293D_M1, L293D_ANTI_CLOCKWISE);
		l293d_dc_motor_rotate(&mc_dev, L293D_M2, L293D_ANTI_CLOCKWISE);
		break;
	case MC_BACKWARD:
		l293d_dc_motor_rotate(&mc_dev, L293D_M1, L293D_CLOCKWISE);
		l293d_dc_motor_rotate(&mc_dev, L293D_M2, L293D_CLOCKWISE);
		break;
	case MC_LEFT:
		l293d_dc_motor_rotate(&mc_dev, L293D_M1, L293D_CLOCKWISE);
		l293d_dc_motor_rotate(&mc_dev, L293D_M2, L293D_ANTI_CLOCKWISE);
		break;
	case MC_RIGHT:
		l293d_dc_motor_rotate(&mc_dev, L293D_M1, L293D_ANTI_CLOCKWISE);
		l293d_dc_motor_rotate(&mc_dev, L293D_M2, L293D_CLOCKWISE);
		break;
	}
	l293d_dc_motors_start(&mc_dev, 1);
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
			/*INFO ("%s: us (left, right) = (%i, %i) cms\n", __func__,
					us_left_distance, us_right_distance);*/
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
			if (!obstacle)
				robot_go_timed(MC_FORWARD, NULL, false, 0);
			break;
		case MC_BACKWARD:
			robot_go_timed(MC_BACKWARD, on_mc_step_timer, false, 0);
			break;
		case MC_STEP_LEFT:
			robot_go_timed(MC_LEFT, on_mc_step_timer, true, 10);
			break;
		case MC_TURN_LEFT:
			robot_go_timed(MC_LEFT, on_mc_step_timer, false, 0);
			break;
		case MC_LEFT:
			robot_go_timed(MC_LEFT, NULL, false, 0);
			break;
		case MC_STEP_RIGHT:
			robot_go_timed(MC_RIGHT, on_mc_step_timer, true, 10);
			break;
		case MC_TURN_RIGHT:
			robot_go_timed(MC_RIGHT, on_mc_step_timer, false, 0);
			break;
		case MC_RIGHT:
			robot_go_timed(MC_RIGHT, NULL, false, 0);
			break;
		case MC_STOP:
			l293d_dc_motors_stop(&mc_dev);
			break;
		default:
			break;
		}
		last_ev = ev;
wait:
		vTaskDelay(20);
	}
end:
	robot_motorctrl_task_ended = true;
	vTaskDelete(NULL);
}

#ifdef PIR
static void pir_timer_cb(TimerHandle_t xTimer)
{
	pir_pending = false;
}
#endif

static void robot_sleep(uint32_t time_in_us)
{
	server_destroy();
	robot_motorctrl_task_end = true;
	while (!robot_motorctrl_task_ended);
	l293d_dc_motors_stop(&mc_dev);
	cam_sensor_stanby(true);
	sdk_system_deep_sleep(time_in_us);
}

static void robot_main_task(void *pvParameters)
{
#ifdef PIR
	int pir_ev;
	TimerHandle_t on_pir_timer;
#endif
	int mc_ev;
	int wbs_ev[2];
	int retry = 0;

	sdk_system_update_cpu_freq(160);

	uart_set_baud(0, 115200);
	i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ_100K);
	if (!spi_set_settings(SPI_BUS, &spi_config)) {
		INFO("%s: failed to init SPI\n", __func__);
		goto end;
	}
#ifdef NEOPIXELS
	leds_init(24, NEOPIXELS_PIN);
#endif

#ifdef PIR
	pir_init(PIR_PIN);
#endif
	xTaskCreate(&robot_motorctrl_task, "robot motor mngt", 256, NULL, 3, NULL);

	if (http_server_init()) {
		INFO ("%s: failed to init httpd\n", __func__);
		goto end;
	}

	cam_sensor_stanby(true);
	delay_ms(200);
	cam_sensor_stanby(false);
	while (!cam_setup(SPI_BUS, SPI_CS, I2C_BUS)) {
		cam_sensor_stanby(true);
		delay_ms(200);
		cam_sensor_stanby(false);
		if (retry++ > 5)
			break;
	}
	server_init();

#ifdef PIR
	on_pir_timer = xTimerCreate("on pir timer", 10000/portTICK_PERIOD_MS,
			pdFALSE, NULL, pir_timer_cb);
	if (on_pir_timer == NULL)
		goto end;
#endif
#ifdef NEOPIXELS
	/* ugly way to make sure all rbg leds are off */
	leds_dimm();
#endif

	while(!robot_main_task_end) {
		if (websocket_wait_for_event(wbs_ev)) {
			switch(wbs_ev[0]) {
			case WBS_POWER_DOWN:
				robot_sleep(SLEEP_TIME);
				goto end;
				break;
			case WBS_LEDS_ON:
#ifdef NEOPIXELS
				leds_turn_on((uint32_t)wbs_ev[1]);
#else
				pcf8574_gpio_write(&pcf8574_dev, LEDS_PIN, false);
				light_on = true;
#endif
				break;
			case WBS_LEDS_OFF:
#ifdef NEOPIXELS
				leds_turn_off();
#else
				pcf8574_gpio_write(&pcf8574_dev, LEDS_PIN, true);
				light_on = false;
#endif
				break;
			case WBS_LEDS_SCROLL:
#ifdef NEOPIXELS
				leds_scroll((uint32_t)wbs_ev[1]);
#endif
				break;
			case WBS_LEDS_DIMM:
#ifdef NEOPIXELS
				leds_dimm();
#endif
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
			case WBS_MC_STEP_LEFT:
				mc_ev = MC_STEP_LEFT;
				xQueueSend(mc_queue, &mc_ev, 0);
				break;
			case WBS_MC_STEP_RIGHT:
				mc_ev = MC_STEP_RIGHT;
				xQueueSend(mc_queue, &mc_ev, 0);
				break;
			case WBS_MC_TURN_LEFT:
				mc_ev = MC_TURN_LEFT;
				xQueueSend(mc_queue, &mc_ev, 0);
				break;
			case WBS_MC_TURN_RIGHT:
				mc_ev = MC_TURN_RIGHT;
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
#ifdef NEOPIWELS
	return leds_is_on();
#else
	return light_on;
#endif
}

void robot_get_us_distance(int32_t *left, uint32_t *right)
{
	*left = us_left_distance;
	*right = us_right_distance;
}

void user_init(void)
{
	struct sdk_station_config config = {
		.ssid = WIFI_SSID,
		.password = WIFI_PASS,
	};

	/* required to call wifi_set_opmode before station_set_config */
	sdk_wifi_set_opmode(STATION_MODE);
	sdk_wifi_station_set_config(&config);
	sdk_wifi_station_connect();

	xTaskCreate(&robot_main_task, "robot mngt", 256, NULL, 2, NULL);
}


