#include <string.h>

#include <unistd.h>

#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include <lwip/api.h>

#include "client.h"
#include "utils.h"

#define DEBUG
#include "trace.h"

static char req[MAX_IN_CHARS];

int client_http_get(int s, const char *in, char *out)
{
	int r;
	char *out_p;
	snprintf(req, MAX_IN_CHARS, "GET %s HTTP/1.1\r\n"
			"Host: localhost\r\n"
			"User-Agent: esp-open-rtos/0.1 esp8266\r\n"
			"Connection: close\r\n"
			"\r\n", in);
	INFO("%s", req);

	if (write(s, req, strlen(req)) < 0) {
		INFO("socket send failed\r\n");
		return -1;
	}

	bzero(out, MAX_OUT_CHARS);
	out_p = out;
	do {
		r = read(s, out_p, (out + MAX_OUT_CHARS) - out_p - 1);
		out_p += r;
		if (out_p >= (out + (MAX_OUT_CHARS - 2)))
			break;
	} while (r > 0);
	INFO("%s", out);
	return 0;
}

int client_open(const char *server, const char *port)
{
	int s;
	const struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM,
	};
	struct addrinfo *res;

	int err = getaddrinfo(server, port, &hints, &res);
	if (err != 0 || res == NULL) {
		INFO("DNS lookup failed err=%d res=%p\n", err, res);
		if(res)
			freeaddrinfo(res);
		return -1;
	}

	struct sockaddr *sa = res->ai_addr;
	if (sa->sa_family == AF_INET) {
		INFO("DNS lookup succeeded. IP=%s\n",
				inet_ntoa(((struct sockaddr_in *)sa)->sin_addr));
	}

	s = socket(res->ai_family, res->ai_socktype, 0);
	if(s < 0) {
		printf("failed to allocate socket\n");
		freeaddrinfo(res);
		return -1;
	}

	if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
		close(s);
		freeaddrinfo(res);
		INFO("socket connect failed.\r\n");
		return -1;
	}
	INFO("connected to %s\n", server);
	freeaddrinfo(res);

	return s;
}

void client_close(int s)
{
	close(s);
}
