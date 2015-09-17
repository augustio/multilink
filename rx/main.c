#include <stdint.h>
#include <string.h>

#include "softdevice_handler.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0

static void ble_evt_dispatch(ble_evt_t *p_ble_evt)
{
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
}

static void ble_stack_init(void)
{
	uint32_t err_code;
	ble_enable_params_t ble_enable_params;

	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

	memset(&ble_enable_params, 0, sizeof(ble_enable_params));
	ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
	err_code = sd_ble_enable(&ble_enable_params);
	APP_ERROR_CHECK(err_code);

	// Register with the SoftDevice handler module for BLE events.
	err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
	APP_ERROR_CHECK(err_code);
	
	// Register with the SoftDevice handler module for BLE events.
	err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
	APP_ERROR_CHECK(err_code);
}

static void power_manage(void)
{
	uint32_t err_code = sd_app_evt_wait();
	APP_ERROR_CHECK(err_code);
}

int main(void)
{
	ble_stack_init();

	while (1)
		power_manage();

	return 0;
}
