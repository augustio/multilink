#include <stdint.h>
#include <stdlib.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nordic_common.h"
#include "sw_spi.h"

/**Eleohjain LSM3DS6 connection pins */

void spi_sw_master_init() {
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
		uint8_t* rx_buffer, uint16_t buffer_len) {

	uint32_t time_us = 1;

	for (int k = 0; k < buffer_len; k++) {
		nrf_gpio_pin_clear(SPIM0_SS_PIN);
		nrf_delay_us(2); //necessary?
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

				nrf_delay_us(time_us);

				nrf_gpio_pin_set(SPIM0_SCK_PIN); //rising edge

				nrf_delay_us(time_us);
			}
			rx_byte = 0;
			if (rx_byte != 0x69) {
				for (int i = 7; i >= 0; i--) {
					nrf_gpio_pin_clear(SPIM0_SCK_PIN); //falling edge
					nrf_delay_us(time_us);
					nrf_gpio_pin_set(SPIM0_SCK_PIN); //rising edge

					//receive bit
					rx_byte = rx_byte | ((uint8_t) nrf_gpio_pin_read(
											SPIM0_MISO_PIN) << i);

					nrf_delay_us(time_us);
				}
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

				nrf_delay_us(time_us);
			}
			nrf_gpio_pin_set(SPIM0_SS_PIN);
		}
	}
}

void spi_interrupt(uint8_t* const interrupt) {
	uint8_t inter;
	inter = (uint8_t) nrf_gpio_pin_read(SPIM0_INT1_PIN);
	interrupt[1] = inter;
}
