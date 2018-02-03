#include <string.h>

#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <lwip/api.h>

#define DEBUG

#include "cam.h"
#include "trace.h"

#define TELNET_PORT 23

#define BUFSIZE 512
uint8_t buf[BUFSIZE];
uint8_t dummy_buf[BUFSIZE];

static void telnetTask(void *pvParameters)
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
    xTaskCreate(telnetTask, "telnetTask", 512, NULL, 2, NULL);
}


