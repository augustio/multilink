/*Missing lot of declarations. Should move them from pseudo_motion_sensor.c
Probably should not use function macros
*/

#include <stdint.h>

#define SAVED_ACCELERATION_BUFFER_SIZE 2
#define DELAY_WITH_STATE_CHANGE(ticks,n_s) current_motion_sensor_state=DELAY_STATE; next_state=n_s; delay_ticks=ticks
#define VIBRATION_WITH_STATE_CHANGE(ticks,n_s,vib_mode,pause_ticks) current_motion_sensor_state=VIBRATION_STATE; next_state=n_s; delay_ticks=ticks; delay_ticks_after_vibration=pause_ticks

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

enum Motion_sensor_states
{
	GETTING_START_STATE,
	RECEIVER_SELECTION,
	RECEIVER_CONTROL,
	DELAY_STATE,
	VIBRATION_STATE,
	SHUTDOWN_STATE
};

enum Vibration_modes
{
	VIBRATION_MODE_NORMAL,
	VIBRATION_MODE_BUZZ_BUZZ
};

void getXYZValues();
void getGyroValues();
void gyroEnable();
void gyroDisable();
void delay();
void vibrate();
void endGestureControl();
void getStartingState();
void selectReceiver();
void controlReceiver();
void gestureControl();
void timer_10ms_tick_handler();

