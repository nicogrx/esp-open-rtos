#include <string.h>

#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <lwip/api.h>

#define DEBUG
#include "trace.h"

#define BUFSIZE 1024
uint8_t buf[BUFSIZE];
uint8_t dummy_buf[BUFSIZE];

#define PORT 80

static bool http_server_end = false;
static volatile bool http_server_ended = false;

static void http_server_task(void *pvParameters)
{
	ip_addr_t client_addr;
	uint16_t port_ignore, len;
	struct netbuf *nb;
	struct netconn *client = NULL;
	char *data;

	struct netconn *nc = netconn_new(NETCONN_TCP);
	if (!nc)
	{
		printf("Status monitor: Failed to allocate socket.\r\n");
		return;
	}
	netconn_bind(nc, IP_ANY_TYPE, PORT);
	netconn_listen(nc);

	while (!http_server_end)
	{
		INFO("%s: wait for accept\n", __func__);
		err_t err = netconn_accept(nc, &client);

		if (err != ERR_OK)
		{
			if (client)
				netconn_delete(client);
			continue;
		}

		netconn_peer(client, &client_addr, &port_ignore);

		if ((err = netconn_recv(client, &nb)) != ERR_OK)
			goto endcon;
		err = netbuf_data(nb, (void*)&data, &len);
		if (err == ERR_OK) {
			INFO("%s: received %d bytes\n", __func__, len);
		} else {
			INFO("%s: netbuf_data returned error %d\n", __func__, err);
			goto endcon;
		}
		INFO("%s\n", data); 
		netbuf_delete(nb);

		sprintf((char *)buf, "HTTP/1.1 200 OK\nContent-Type: image/jpeg; charset=utf-8\nConnection: close\n\n");
		netconn_write(client, buf, strlen((char *)buf), NETCONN_COPY);
endcon:
		netconn_delete(client);
	}
	http_server_ended = true;
	vTaskDelete(NULL);
}

void server_init(void)
{
	xTaskCreate(http_server_task, "simple http server", 512, NULL, 2, NULL);
}

void server_destroy(void)
{
	http_server_end = true;
	while (!http_server_ended);
}
