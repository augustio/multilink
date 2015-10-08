#include <stdint.h>
#include <stdlib.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nordic_common.h"
#include "sw_spi.h"

#define SPI_MASTER_HALF_PERIOD_DELAY __ASM ( \
" NOP\n\t" \
" NOP\n\t" \
" NOP\n\t" \
" NOP\n\t" \
" NOP\n\t" \
" NOP\n\t" \
" NOP\n\t" \
" NOP\n\t" \
) //About 0.5 us

void spi_sw_master_init()
{
	//A Slave select must be set as high before setting it as output,
	//because during connect it to the pin it causes glitches.
	nrf_gpio_pin_set(SPIM0_SS_PIN);
	nrf_gpio_cfg_output(SPIM0_SS_PIN);
	nrf_gpio_pin_set(SPIM0_SS_PIN);

	//Configure GPIO
	nrf_gpio_cfg_output(SPIM0_SCK_PIN);
	nrf_gpio_cfg_output(SPIM0_MOSI_PIN);
	nrf_gpio_cfg_input(SPIM0_MISO_PIN, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_pin_set(SPIM0_SCK_PIN);
	nrf_gpio_pin_clear(SPIM0_MOSI_PIN);	//not required
	//nrf_gpio_cfg_input(SPIM0_INT1_PIN, NRF_GPIO_PIN_NOPULL);
	//*(uint8_t*)(0x0A)(FIFO_MODE_[2:0] = 001);
}

void spi_sw_master_send_bytes(uint16_t* tx_buffer,
		uint8_t* rx_buffer, uint16_t buffer_len)
{
	for (int k = 0; k < buffer_len; k++) {
		nrf_gpio_pin_clear(SPIM0_SS_PIN);

		//Necessary?
		SPI_MASTER_HALF_PERIOD_DELAY;
		SPI_MASTER_HALF_PERIOD_DELAY;

		uint8_t rx_byte = 0;

		//CPOL=1,CPHA=1, msbfirst

		//READ
		if ((tx_buffer[k] >> 15) & 0x0001) {

			for (int i = 15; i >= 8; i--) {
				nrf_gpio_pin_clear(SPIM0_SCK_PIN); //falling edge

				//send bit
				if ((tx_buffer[k] >> i) & 0x0001)
					nrf_gpio_pin_set(SPIM0_MOSI_PIN);
				else
					nrf_gpio_pin_clear(SPIM0_MOSI_PIN);

				SPI_MASTER_HALF_PERIOD_DELAY;

				nrf_gpio_pin_set(SPIM0_SCK_PIN); //rising edge

				SPI_MASTER_HALF_PERIOD_DELAY;
			}
			rx_byte = 0;
			for (int i = 7; i >= 0; i--) {
				nrf_gpio_pin_clear(SPIM0_SCK_PIN); //falling edge
				SPI_MASTER_HALF_PERIOD_DELAY;
				nrf_gpio_pin_set(SPIM0_SCK_PIN); //rising edge

				//receive bit
				rx_byte = rx_byte | ((uint8_t) nrf_gpio_pin_read(
							SPIM0_MISO_PIN) << i);

				SPI_MASTER_HALF_PERIOD_DELAY;
			}
			rx_buffer[k] = rx_byte;
			nrf_gpio_pin_set(SPIM0_SS_PIN);

			//WRITE
		} else {

			//CPOL=1,CPHA=1, msbfirst
			for (int i = 15; i >= 0; i--) {
				nrf_gpio_pin_clear(SPIM0_SCK_PIN); //falling edge

				//send bit
				if ((tx_buffer[k] >> i) & 0x0001)
					nrf_gpio_pin_set(SPIM0_MOSI_PIN);
				else
					nrf_gpio_pin_clear(SPIM0_MOSI_PIN);

				nrf_gpio_pin_set(SPIM0_SCK_PIN); //rising edge

				SPI_MASTER_HALF_PERIOD_DELAY;
			}
			nrf_gpio_pin_set(SPIM0_SS_PIN);
		}
	}
}

void spi_register_conf(void)
{
	uint16_t buf_len = 12;
	uint8_t rx_buffer[buf_len];
	uint16_t tx_buffer[buf_len];
	uint16_t *tx = tx_buffer;
	uint8_t *rx = rx_buffer;

	int index;
	for (index = 0; index < buf_len; index++) {
		tx_buffer[index] = 0;
		rx_buffer[index] = 0;
	}

	tx_buffer[0] = 0x1000 | 0x48; // CTRL1_XL: Set 104 Hz, 4 g full scale.
	tx_buffer[1] = 0x1500 | 0x10; // CTRL6_C: Disable high-performance mode for acceleration.
	tx_buffer[2] = 0x1600 | 0x80; // CTRL7_G: Disable high-performance mode for gyro.
	tx_buffer[3] = 0x1900 | 0x10; // CTRL10_C: Disable x (roll) and z (pitch) axes for gyro. 00zyx000
	tx_buffer[4] = 0x5800 | 0x06; // TAP_CFG: Enable tap interrupt in yz directions(011). No latch(0).
	tx_buffer[5] = 0x5900 | 0x08; // TAP_THS_6D: Tap threshold full scale/32 * 8 = 1g.
	tx_buffer[6] = 0x5A00 | 0x3C; // INT_DUR2: Dur(0011)=960ms,Quiet(11)=120ms,Shock(00)=40ms.
	tx_buffer[7] = 0x5B00 | 0xC4; // WAKE_UP_THS: Enable double tap and activity/inactivity (THS=0.25g). Double tap only: 80.
	tx_buffer[8] = 0x5C00 | 0x41; // WAKE_UP_DUR: Wake with 2 points above THS. Sleep in 5 s.
	tx_buffer[9] = 0x5E00 | 0x08; // MD1_CFG: Route double tap to INT1.
	tx_buffer[10] = 0x5F00 | 0x00; // MD2_CFG: Disable INT 2.
	tx_buffer[11] = 0x0000;

	spi_sw_master_send_bytes(tx, rx, buf_len);
}
