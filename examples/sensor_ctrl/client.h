#ifndef CLIENT_H
#define CLIENT_H

#define MAX_IN_CHARS 256
#define MAX_OUT_CHARS 512

int client_open(const char *server, const char *port);
void client_close(int s);
int client_http_get(int s, const char *in, char *out);
#endif
