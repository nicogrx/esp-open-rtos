#ifndef SENSOR_BMP280_H
#define SENSOR_BMP280_H
int sensor_bmp280_init(void *private);
int sensor_bmp280_refresh(char *out);
#endif
