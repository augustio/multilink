#ifndef SW_SPI_H
#define SW_SPI_H

#include "nrf_gpio.h"
#include <stdint.h>

/**Eleohjain LSM3DS6 connection pins 
*/
#define SPIM0_SCK_PIN       3     /**< SPI clock GPIO pin number. */
#define SPIM0_MOSI_PIN      2     /**< SPI Master Out Slave In GPIO pin number. */
#define SPIM0_MISO_PIN      30     /**< SPI Master In Slave Out GPIO pin number. */
#define SPIM0_SS_PIN        4     /**< SPI Slave Select GPIO pin number. */
#define SPIM0_INT1_PIN		9		/**< SPI Interrupt1 GPIO pin number */

void spi_sw_master_init();
void spi_sw_master_send_bytes(uint16_t* const tx_buffer, uint8_t* const rx_buffer, uint16_t buffer_len);
//void spi_sw_master_send_bytes2(uint16_t* const tx_buffer, uint8_t* const rx_buffer, uint16_t buffer_len, uint32_t freq);
//void spi_sw_master_send_conf(uint16_t* const tx_buffer, uint32_t buffer_len, uint32_t freq);
void spi_interrupt(uint8_t* const interrupt);

//initialize SPI and send bytes
void init_spi();

#endif
