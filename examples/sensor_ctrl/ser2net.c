#include <string.h>

#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <lwip/api.h>

#include "ser2net.h"

#define DEBUG
#include "trace.h"

#define BUFSIZE 32
uint8_t buf[BUFSIZE];

#define PORT 9999

static bool ser2net_end = false;
static volatile bool ser2net_ended = false;
static int serial_port;

static void ser2net_task(void *pvParameters)
{
	ip_addr_t client_addr;
	uint16_t port_ignore;
	struct netconn *client = NULL;
	struct netconn *nc;
	err_t err;
	int c, i;

	nc = netconn_new(NETCONN_TCP);
	if (!nc)
	{
		INFO("%s: failed to allocate socket\n", __func__);
		return;
	}
	netconn_bind(nc, IP_ANY_TYPE, PORT);
	netconn_listen(nc);

	while (!ser2net_end)
	{
		INFO("%s: wait for accept\n", __func__);
		err = netconn_accept(nc, &client);
		if (err != ERR_OK)
			goto endcon;
		err = netconn_peer(client, &client_addr, &port_ignore);
		if (err != ERR_OK)
			goto endcon;
		i = 0;
		while (!ser2net_end) {
			c = uart_getc_nowait(serial_port);
			if (c != -1) {
				buf[i++] = (uint8_t)c;
				if (i == BUFSIZE) {
					err = netconn_write(client, buf, BUFSIZE, NETCONN_COPY);
					if (err != ERR_OK)
						goto endcon;
					i = 0;
				}
			} else {
				vTaskDelay(10);
			}
		}
endcon:
		if (client)
			netconn_delete(client);
	}
	ser2net_ended = true;
	vTaskDelete(NULL);
}

void ser2net_init(int port, int baudrate, UART_ByteLength bl, UART_StopBits sb,
				  bool parity_en, UART_Parity p)
{
	/*setbuf(stdout, NULL);*/
	uart_set_baud(port, baudrate);
	uart_set_byte_length(port, bl);
	uart_set_stopbits(port, sb);
	uart_set_parity_enabled(port, parity_en);
	if (parity_en)
		uart_set_parity(port, p);
	serial_port = port;
	xTaskCreate(ser2net_task, "ser2net", 512, NULL, 2, NULL);
}

void ser2net_destroy(void)
{
	ser2net_end = true;
	while (!ser2net_ended);
}
