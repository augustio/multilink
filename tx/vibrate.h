#ifndef __ELECTRIA_VIBRATE_H
#define __ELECTRIA_VIBRATE_H

#include <stdbool.h>
#include <stdint.h>

/* Time in milliseconds */
#define VIBRATE_DURATION_ZERO 0
#define VIBRATE_DURATION_EXTRA_SHORT 100
#define VIBRATE_DURATION_SHORT 150
#define VIBRATE_DURATION_LONG 200
#define VIBRATE_DURATION_EXTRA_LONG 240
#define VIBRATE_PAUSE_DURATION_EXTRA_LONG 700
#define VIBRATE_PAUSE_DURATION_NORMAL 300
#define VIBRATE_PAUSE_DURATION_SHORT 100

#define VIBRATOR_PIN 10

void vibrate_init();
void do_vibrate(uint32_t duration, uint32_t post_duration, bool *in_vibration);

#endif
