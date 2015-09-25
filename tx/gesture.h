#ifndef __ELECTRIA_GESTURE_H
#define __ELECTRIA_GESTURE_H

#include <stdbool.h>

void getXYZValues(void);
void getGyroValues(void);
void gyroEnable(void);
void gyroDisable(void);

bool isGyroEnabled(void);

int swipeRight(void);
int swipeLeft(void);

int armIsUp(void);
int armIsDown(void);
int armIsNeutral(void);
int armIsStationary(void);
int armRotation(void);

#define SAVED_ACCELERATION_BUFFER_SIZE (2)

typedef struct accel {
	double x[SAVED_ACCELERATION_BUFFER_SIZE];
	double y[SAVED_ACCELERATION_BUFFER_SIZE];
	double z[SAVED_ACCELERATION_BUFFER_SIZE];
} accel_t;

#endif
