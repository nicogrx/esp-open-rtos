#include <string.h>

#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <lwip/api.h>

//#define DEBUG

#include "cam.h"
#include "trace.h"

#define BUFSIZE 1024
uint8_t buf[BUFSIZE];
uint8_t dummy_buf[BUFSIZE];

//#define TELNET

#ifdef TELNET
#define TELNET_PORT 23
static void telnet_task(void *pvParameters)
{
	uint32_t bytes_to_read, bytes_to_send;

	struct netconn *nc = netconn_new(NETCONN_TCP);
	if (!nc)
	{
		printf("Status monitor: Failed to allocate socket.\r\n");
		return;
	}
	netconn_bind(nc, IP_ANY_TYPE, TELNET_PORT);
	netconn_listen(nc);

	while (1)
	{
		struct netconn *client = NULL;
		err_t err = netconn_accept(nc, &client);

		if (err != ERR_OK)
		{
			if (client)
				netconn_delete(client);
			continue;
		}

		ip_addr_t client_addr;
		uint16_t port_ignore;
		netconn_peer(client, &client_addr, &port_ignore);

		bytes_to_read = cam_capture();
		if (!bytes_to_read)
			goto endcon;

		INFO("%s: sending %u bytes...", __func__, bytes_to_read);
		netconn_write(client, &bytes_to_read, sizeof(uint32_t), NETCONN_COPY);

		cam_read_start();
		while (bytes_to_read) {
			bytes_to_send = bytes_to_read > BUFSIZE ? BUFSIZE : bytes_to_read;
			cam_read(dummy_buf, buf, bytes_to_send);
			/*INFO("%s: sending %u bytes\n", __func__, bytes_to_send);*/
			netconn_write(client, buf, bytes_to_send, NETCONN_COPY);
			bytes_to_read -= bytes_to_send;
		}
		cam_read_stop();
		INFO("done\n");
endcon:
		netconn_delete(client);
	}
}

void access_point_init(void)
{
	xTaskCreate(telnet_task, "telnet task", 512, NULL, 2, NULL);
}

#else
#define SIMPLE_HTTP_PORT 8081

static bool simple_http_task_end = false;
static volatile bool simple_http_task_ended = false;

static void simple_http_task(void *pvParameters)
{
	uint32_t bytes_to_read, bytes_to_send;
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
	netconn_bind(nc, IP_ANY_TYPE, SIMPLE_HTTP_PORT);
	netconn_listen(nc);

	while (!simple_http_task_end)
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
		err = netbuf_data(nb, (void*) &data, &len);
		if (err == ERR_OK) {
			INFO("%s: received %d bytes\n", __func__, len);
		} else {
			INFO("%s: netbuf_data returned error %d\n", __func__, err);
			goto endcon;
		}
		netbuf_delete(nb);

		bytes_to_read = cam_capture();
		if (!bytes_to_read)
			goto endcon;

		INFO("%s: sending %u bytes...", __func__, bytes_to_read);
		sprintf((char *)buf, "HTTP/1.1 200 OK\nContent-Type: image/jpeg; charset=utf-8\nConnection: close\n\n");
		netconn_write(client, buf, strlen((char *)buf), NETCONN_COPY);

		cam_read_start();
		while (bytes_to_read) {
			bytes_to_send = bytes_to_read > BUFSIZE ? BUFSIZE : bytes_to_read;
			cam_read(dummy_buf, buf, bytes_to_send);
			/*INFO("%s: sending %u bytes\n", __func__, bytes_to_send);*/
			netconn_write(client, buf, bytes_to_send, NETCONN_COPY);
			bytes_to_read -= bytes_to_send;
		}
		cam_read_stop();
		INFO("done\n");
endcon:
		netconn_delete(client);
	}
	simple_http_task_ended = true;
	vTaskDelete(NULL);
}

void access_point_init(void)
{
	xTaskCreate(simple_http_task, "simple http task", 512, NULL, 2, NULL);
}

void access_point_destroy(void)
{
	simple_http_task_end = true;
	while (!simple_http_task_ended);
}

#endif
