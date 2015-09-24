#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "gesture.h"
#include "sw_spi.h"

static bool gyro_enabled = false;
static accel_t gyro;
static accel_t acc;

static double yaw = 0.0;
static const double yaw_activate = 200.0; // degree per second

static double dx;
static double dy;
static double dz;
static double g;
static double dg;
static double theta; // Deviation from horizontal in degrees (-90,90). Up positive.
static double phi; // Deviation from semipronation in degrees (-90,90). Clockwise positive.

static const double theta_high = 45.0; // degree
static const double theta_low = -60.0; // degree
static const double phi_neutral_cw = 20; //clockwise activation angle, phi_activate/2 degree
static const double phi_neutral_ccw = -20; //counterclockwise activation angle, phi_activate/2 degree
static const double theta_neutral_high = 20.0; // theta_high/2 degree
static const double theta_neutral_low = -45.0; // degree
static const double yaw_limit = 50;
static const double theta_limit_high = 35.0; // degree, above this blocks phi recognition
static const double theta_limit_low = -45.0; // degree, below this blocks phi recognition
static const double phi_cw = 40.0; // degree
static const double phi_ccw = -40.0; // degree

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

int armIsUp(void)
{
	return (theta > theta_high);
}

int armIsDown(void)
{
	return (theta < theta_low);
}

int armIsNeutral(void)
{
	return (phi < phi_neutral_cw && phi > phi_neutral_ccw
			&& theta < theta_neutral_high && theta > theta_neutral_low);
}

int armRotation(void)
{
	if (fabs(yaw) > yaw_limit || theta > theta_limit_high
			|| theta < theta_limit_low || fabs(phi) > 80) {
		return 0;
	} else if (phi > phi_cw) {
		return 1;
	} else if (phi < phi_ccw) {
		return -1;
	} else {
		return 0;
	}
}

static double calcDx()
{
	return (acc.x[n] - acc.x[n - 1]);
}

static double calcDy()
{
	return (acc.y[n] - acc.y[n - 1]);
}

static double calcDz()
{
	return (acc.z[n] - acc.z[n - 1]);
}

static double calcG()
{
	return (sqrt(pow(acc.x[n], 2.0) + pow(acc.y[n], 2.0) + pow(acc.z[n], 2.0)));
}

static double calcDg()
{
	return (sqrt(pow(dx, 2.0) + pow(dy, 2.0) + pow(dz, 2.0)));
}

static double calcTheta()
{
	return (asin(acc.x[n] / g) * 180 / M_PI);
}

static double calcPhi()
{
	double phiv = atan(
			acc.z[n] / sqrt(0.001 * pow(acc.x[n], 2.0) + pow(acc.y[n], 2.0)))
			* 180 / M_PI;
	if (acc.y[n] > 0) {
		return -phiv;
	} else {
		return phiv;
	}
}

void getXYZValues(void)
{
	uint16_t tx_buffer[6];
	uint8_t rx_buffer[6];
	int16_t tmpx, tmpy, tmpz;

	tx_buffer[0] = 0x2800 | 0x8000;
	tx_buffer[1] = 0x2900 | 0x8000;
	tx_buffer[2] = 0x2A00 | 0x8000;
	tx_buffer[3] = 0x2B00 | 0x8000;
	tx_buffer[4] = 0x2C00 | 0x8000;
	tx_buffer[5] = 0x2D00 | 0x8000;

	spi_sw_master_send_bytes(tx_buffer, rx_buffer, 6);

	acc.x[0] = acc.x[1];
	acc.y[0] = acc.y[1];
	acc.z[0] = acc.z[1];

	tmpx = (rx_buffer[0] | (rx_buffer[1] << 8));
	tmpy = (rx_buffer[2] | (rx_buffer[3] << 8));
	tmpz = (rx_buffer[4] | (rx_buffer[5] << 8));

	// Normitus s.e. g = 1 ja poistetaan datasta nollat
	acc.x[1] = -tmpx / (double) 0x2000 + 0.00001; // +x = kyynärpäätä kohti
	acc.y[1] =  tmpy / (double) 0x2000 + 0.00001; //
	acc.z[1] = -tmpz / (double) 0x2000 + 0.00001; // +z = kämmentä kohti
	dx = calcDx();
	dy = calcDy();
	dz = calcDz();
	g = calcG();
	dg = calcDg();
	theta = calcTheta();
	phi = calcPhi();
}
