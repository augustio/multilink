#ifndef __ELECTRIA_GESTURE_H
#define __ELECTRIA_GESTURE_H

void getGyroValues(void);
void gyroEnable(void);
void gyroDisable(void);

int swipeRight(void);
int swipeLeft(void);

#define SAVED_ACCELERATION_BUFFER_SIZE (2)

typedef struct accel {
	double x[SAVED_ACCELERATION_BUFFER_SIZE];
	double y[SAVED_ACCELERATION_BUFFER_SIZE];
	double z[SAVED_ACCELERATION_BUFFER_SIZE];
} accel_t;

extern accel_t acc;
extern double yaw;

#endif
