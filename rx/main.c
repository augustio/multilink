#include <stdint.h>
#include <string.h>

#include "softdevice_handler.h"
#include "ble_advertising.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0
#define APP_ADV_INTERVAL                   MSEC_TO_UNITS(50, UNIT_0_625_MS)
#define APP_ADV_TIMEOUT_IN_SECONDS         0

static void ble_evt_dispatch(ble_evt_t *p_ble_evt)
{
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
}

static void advertising_init(void)
{
	uint32_t err_code;
	ble_advdata_t advdata;
	ble_adv_modes_config_t options = {0};

	memset(&advdata, 0, sizeof(advdata));

	err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
	APP_ERROR_CHECK(err_code);
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

void advertising_start()
{
	uint32_t err_code;
	ble_gap_adv_params_t adv_params;

	memset(&adv_params, 0, sizeof(adv_params));

	adv_params.type = BLE_GAP_ADV_TYPE_ADV_IND;
	adv_params.p_peer_addr = NULL;
	adv_params.fp = BLE_GAP_ADV_FP_ANY;
	adv_params.interval = APP_ADV_INTERVAL;
	adv_params.timeout = APP_ADV_TIMEOUT_IN_SECONDS;

	err_code = sd_ble_gap_adv_start(&adv_params);
	APP_ERROR_CHECK(err_code);
}

int main(void)
{
	ble_stack_init();

	advertising_init();

	advertising_start();

	while (1)
		power_manage();

	return 0;
}
