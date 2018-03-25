#ifndef SER2NET_H
#define SER2NET_H

void ser2net_init(int port, int baudrate, UART_StopBits sb,
				  bool parity_en, UART_Parity p);
void ser2net_destroy(void);
#endif
