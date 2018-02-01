#ifndef CAM_H
#define CAM_H

bool cam_setup(uint8_t spi_bus, uint8_t spi_cs, uint8_t i2c_bus);
uint32_t cam_capture(void);
void cam_read_start(void);
void cam_read_stop(void);
void cam_read(uint8_t *dummy_buf, uint8_t *buf, int size);

#endif
