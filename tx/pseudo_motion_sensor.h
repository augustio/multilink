/*Missing lot of declarations. Should move them from pseudo_motion_sensor.c
*/

#ifndef MOTION_SENSOR_H
#define MOTION_SENSOR_H

#define SAVED_ACCELERATION_BUFFER_SIZE 2

struct accel {
	double x[SAVED_ACCELERATION_BUFFER_SIZE];
	double y[SAVED_ACCELERATION_BUFFER_SIZE];
	double z[SAVED_ACCELERATION_BUFFER_SIZE];
};

struct accel_int {
	int16_t x[1];
	int16_t y[1];
	int16_t z[1];
};

enum Command{
	NEXT = 0xdd,
	PREVIOUS = 0x11,
	UP = 0x12,
	DOWN = 0x13,
	TOGGLE = 0x14
};

#endif
