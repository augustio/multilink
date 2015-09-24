#include "vibrate.h"

#include "app_error.h"
#include "app_timer.h"

#include "nrf_gpio.h"

#define APP_TIMER_PRESCALER	0

static app_timer_id_t m_vibro_timer;
static app_timer_id_t m_post_vibro_timer;

static bool vibrating = false;

static void post_vibro_timer_handler(void *p_context)
{
}

static void vibro_timer_handler(void *p_context)
{
	bool *in_vibration = (bool *)p_context;

	nrf_gpio_pin_clear(VIBRATOR_PIN);
	vibrating = false;
	*in_vibration = false;
}

void do_vibrate(uint32_t duration, uint32_t post_duration, bool *in_vibration)
{
	uint32_t ticks;
	uint32_t err_code;

	if (vibrating)
		return;

	ticks = APP_TIMER_TICKS(duration, APP_TIMER_PRESCALER);
	err_code = app_timer_start(m_vibro_timer, ticks, (void *)in_vibration);
	if (err_code != NRF_SUCCESS) {
		return;
	}

	/* timer ticking, set the pin */
	nrf_gpio_pin_set(VIBRATOR_PIN);
	*in_vibration = true;
	vibrating = true;
}

void vibrate_init(void)
{
	uint32_t err_code;

	err_code = app_timer_create(&m_vibro_timer,
					APP_TIMER_MODE_SINGLE_SHOT,
					vibro_timer_handler);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_create(&m_post_vibro_timer,
					APP_TIMER_MODE_SINGLE_SHOT,
					post_vibro_timer_handler);
	APP_ERROR_CHECK(err_code);

	nrf_gpio_cfg_output(VIBRATOR_PIN);
}
