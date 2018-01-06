#ifndef LEDS_H
#define LEDS_H

#define RED		0xFF0000
#define GREEN	0x00FF00
#define BLUE	0x0000FF
#define BLACK	0x000000
#define WHITE	0xFFFFFF
#define PINK	0xFF007F

void leds_init(int nb_leds, uint8_t pin);
void leds_turn_on(uint32_t);
void leds_turn_off(void);
void leds_scroll(uint32_t);
void leds_dimm(void);
bool leds_is_on(void);
#endif
