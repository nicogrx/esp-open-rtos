#include "FreeRTOS.h"
#include "task.h"

#include "arducam/arducam_arch.h"
#include "arducam/arducam.h"

//#define DEBUG
#include "trace.h"
#include "utils.h"

void cam_sensor_power(bool on)
{
	uint8_t reg = arducam_read_reg(ARDUCHIP_GPIO);
	if (on) {
		arducam_write_reg(ARDUCHIP_GPIO, (reg | GPIO_LDOS_EN_MASK)
			& ~GPIO_PWDN_MASK);
		INFO("%s: on\n", __func__);
	} else {
		arducam_write_reg(ARDUCHIP_GPIO, (reg | GPIO_PWDN_MASK)
			& ~GPIO_LDOS_EN_MASK);
		INFO("%s: off\n", __func__);
	}
}

void cam_sensor_stanby(bool low)
{
	uint8_t reg = arducam_read_reg(ARDUCHIP_GPIO);
	if (low)
		arducam_write_reg(ARDUCHIP_GPIO, reg | GPIO_PWDN_MASK);
	else
		arducam_write_reg(ARDUCHIP_GPIO, reg & ~GPIO_PWDN_MASK);
}

bool cam_setup(uint8_t spi_bus, uint8_t spi_cs, uint8_t i2c_bus)
{
	uint8_t vid, pid, temp;

	arducam_arch(smOV2640, spi_bus, spi_cs, i2c_bus);

	/* Check if the ArduCAM SPI bus is OK */
	arducam_write_reg(ARDUCHIP_TEST1, 0x55);
	temp = arducam_read_reg(ARDUCHIP_TEST1);
	if(temp != 0x55) {
		INFO("%s: spi test error (got 0x%02x)!\n", __func__, temp);
	}
	/* Change MCU mode */
	arducam_write_reg(ARDUCHIP_MODE, 0x00);
	arducam_i2c_read(OV2640_CHIPID_HIGH, &vid);
	arducam_i2c_read(OV2640_CHIPID_LOW, &pid);
	if((vid != 0x26) || (pid != 0x42)) {
		INFO("%s: cannot find OV2640 chip (got 0x%02x, 0x%02x)\n", __func__, vid, pid);
		return false;
	} else {
		INFO("%s: OV2640 detected\n", __func__);
	}
	arducam_set_format(fmtJPEG);
	arducam_init();
	arducam_set_jpeg_size(sz640x480);
	INFO("%s exposure delay...", __func__);
	delay_ms(1000); /* time required for autoexposure */
	INFO(" done\n");

	return true;
}

uint32_t cam_capture(void)
{
	uint8_t val;
	uint32_t start_time = systime_ms();
	uint32_t length;

    arducam_flush_fifo();    
	arducam_clear_fifo_flag(); /* same than arducam_flush_fifo(); */
	arducam_start_capture();
	while (1) {
		val = arducam_read_reg(ARDUCHIP_TRIG);
		/*INFO("%s: trig:%x\n", __func__, val);*/
		if (val & CAP_DONE_MASK)
			break;
		if ((systime_ms() - start_time) > 1000) {
			INFO("%s: timeout\n", __func__);
			return 0;
		}
		delay_ms(10);
	}
	INFO("%s: capture done after %ums\n", __func__,
		 systime_ms() - start_time);
	length = arducam_read_fifo_length();
	INFO("%s: fifo length = %u\n", __func__, length);
	return length;
}

void cam_read_start(void)
{
	arducam_burst_read_start();
}

void cam_read_stop(void)
{
	arducam_burst_read_stop();
}

void cam_read(uint8_t *dummy_buf, uint8_t *buf, int size)
{
	arducam_burst_read_fifo(dummy_buf, buf, size);
}
