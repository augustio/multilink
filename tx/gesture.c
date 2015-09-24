#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "gesture.h"
#include "sw_spi.h"

static bool gyro_enabled = false;
static accel_t gyro;

static const double yaw_activate = 200.0; // degree per second

#define n (SAVED_ACCELERATION_BUFFER_SIZE - 1)

static double calcYaw() 
{
	double yawv;

	// This could be improved to take orientation into account
	// Now assumes perfect semipronation
	if (acc.y[n] > 0) {
		yawv = gyro.y[n];
	} else {
		yawv = -gyro.y[n];
	}
	return yawv;
}

void getGyroValues() 
{
	// Yaw only
	uint16_t tx_buffer[2];
	uint8_t rx_buffer[2];
	int16_t tmp;

	tx_buffer[0] = 0x2400 | 0x8000;
	tx_buffer[1] = 0x2500 | 0x8000;
	spi_sw_master_send_bytes(tx_buffer, rx_buffer, 2);

	gyro.y[0] = gyro.y[1];
	tmp = (rx_buffer[0] | (rx_buffer[1] << 8));

	gyro.y[1] = -tmp / 114.28 + 0.001; // Yaw in degrees per second
	if (!gyro_enabled)
		yaw = 0.0;
	else
		yaw = calcYaw();
}

void gyroEnable(void)
{
	uint16_t tx_buffer[2]; //Transmit buffer to send data from SPI master with sample data.
	uint16_t *tx = tx_buffer;

	tx_buffer[0] = 0x1100 | 0x40; // CTRL2_G: Set gyro 104 Hz, 245 dps full scale.
	tx_buffer[1] = 0x0000;
	spi_sw_master_send_bytes(tx, NULL, 2);
	gyro_enabled = 1;
}

void gyroDisable(void)
{	
	uint16_t tx_buffer[2]; //Transmit buffer to send data from SPI master with sample data.
	uint16_t *tx = tx_buffer;

	tx_buffer[0] = 0x1100 | 0x00; // CTRL2_G: Set gyro power down.
	tx_buffer[1] = 0x0000;
	spi_sw_master_send_bytes(tx, NULL, 2);
	gyro_enabled = 0;
}

int swipeRight(void)
{
	return (yaw > yaw_activate && fabs(fabs(acc.y[n]) - 1) < 0.2);
}

int swipeLeft(void)
{
	return (yaw < -yaw_activate && fabs(fabs(acc.y[n]) - 1) < 0.2);
}
