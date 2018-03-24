#include <string.h>

#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <lwip/api.h>

#include "server.h"

#define DEBUG
#include "trace.h"

static QueueHandle_t host_req_queue;

#define BUFSIZE 512
char buf[BUFSIZE];

#define PORT 80

static bool http_server_end = false;
static volatile bool http_server_ended = false;


static void server_parse_data(char *data)
{
	char *tmp;
	char *tmp2;

	tmp = strstr(data, "GET");
	if (!tmp)
		return;

	if (tmp[3] != ' ')
		return;
	tmp += 4;
	tmp2 = strchr(tmp, ' ');
	if (!tmp2)
		return;
	snprintf(buf, tmp2 + 1 - tmp, "%s", tmp);
	INFO("%s: %s\n", __func__, buf);
	xQueueSend(host_req_queue, buf, 0);
}

static void server_task(void *pvParameters)
{
	ip_addr_t client_addr;
	uint16_t port_ignore, len;
	struct netbuf *nb;
	struct netconn *client = NULL;
	char *data;
	struct netconn *nc;

	host_req_queue = xQueueCreate(3, HOST_REQ_SIZE);

	nc = netconn_new(NETCONN_TCP);
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
		/*INFO("%s\n", data);*/
		server_parse_data(data);
		netbuf_delete(nb);

		sprintf(buf, "HTTP/1.1 200 OK\nContent-Type: image/jpeg; charset=utf-8\nConnection: close\n\n");
		netconn_write(client, (uint8_t *)buf, strlen(buf), NETCONN_COPY);
endcon:
		netconn_delete(client);
	}
	http_server_ended = true;
	vTaskDelete(NULL);
}

bool server_wait_for_event(char * host_req)
{
	if (xQueueReceive(host_req_queue, host_req, 0) == pdFALSE)
		return false;
	return true;
}

void server_init(void)
{
	xTaskCreate(server_task, "simple http server", 512, NULL, 2, NULL);
}

void server_destroy(void)
{
	http_server_end = true;
	while (!http_server_ended);
}
