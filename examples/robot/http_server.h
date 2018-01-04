#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

enum {
	WBS_LEDS_ON,
	WBS_LEDS_OFF,
	WBS_LEDS_SCROLL,
	WBS_LEDS_DIMM,

};

bool websocket_wait_for_event(int *ev);
int http_server_init(void);

#endif
