#include "vibrate.h"

#include "app_error.h"
#include "app_timer.h"

static app_timer_id_t m_vibro_timer;
static app_timer_id_t m_post_vibro_timer;

static void vibro_timer_handler(void *p_context)
{
}

static void post_vibro_timer_handler(void *p_context)
{
}

void do_vibrate(uint32_t duration, uint32_t post_duration, bool *in_vibration)
{
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
}
