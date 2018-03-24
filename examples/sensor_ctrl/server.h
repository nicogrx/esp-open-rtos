#ifndef SERVER_H
#define SERVER_H

#define HOST_REQ_SIZE 256

bool server_wait_for_event(char * host_req);
void server_init(void);
void server_destroy(void);
#endif
