#ifndef HTTP_CLIENT_H
#define HTTP_CLIENT_H

#define MAX_IN_CHARS 256
#define MAX_OUT_CHARS 512
int http_get(const char *server, const char *port, const char *in, char *out);

#endif
