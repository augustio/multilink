#include <stdint.h>
#include <string.h>

#include "softdevice_handler.h"
#include "ble_advertising.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0
#define APP_ADV_INTERVAL                   MSEC_TO_UNITS(50, UNIT_0_625_MS)
#define APP_ADV_TIMEOUT_IN_SECONDS         0

#define DEVICE_NAME "Electria_receiver"
#define MIN_CONN_INTERVAL                  MSEC_TO_UNITS(500, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL                  MSEC_TO_UNITS(1000, UNIT_1_25_MS)          /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                      0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                   MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define MULTILINK_PERIPHERAL_BASE_UUID     {{0xB3, 0x58, 0x55, 0x40, 0x50, 0x60, 0x11, \
	                                           0xe3, 0x8f, 0x96, 0x08, 0x00, 0x00, 0x00,   \
	                                           0x9a, 0x66}}

#define MULTILINK_PERIPHERAL_SERVICE_UUID  0x9001
#define MULTILINK_PERIPHERAL_CHAR_UUID     0x900A

static uint8_t m_base_uuid_type;
static ble_gatts_char_handles_t  m_char_handles;

static void ble_evt_dispatch(ble_evt_t *p_ble_evt)
{
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
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

static void advertising_init(void)
{
	uint32_t err_code;
	ble_advdata_t advdata;
	ble_adv_modes_config_t options = {0};

	memset(&advdata, 0, sizeof(advdata));

	advdata.name_type               = BLE_ADVDATA_FULL_NAME;
	advdata.include_appearance      = true;
	advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

	options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
	options.ble_adv_fast_interval = APP_ADV_INTERVAL;
	options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

	err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
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

static void gap_params_init(void)
{
	uint32_t                err_code;
	ble_gap_conn_params_t   gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode,
			(const uint8_t *)DEVICE_NAME,
			strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency     = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}

static void services_init(void)
{
	uint32_t            err_code;
	ble_uuid_t          uuid;
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t    attr;
	ble_gatts_attr_md_t attr_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_md_t char_ud_md;
	uint16_t            svc_test;

	static uint8_t multilink_peripheral_data;
	static uint8_t multilink_peripheral_ud[] = "Modifiable multilink_peripheral Data";

	ble_uuid128_t base_uuid = MULTILINK_PERIPHERAL_BASE_UUID;

	err_code = sd_ble_uuid_vs_add(&base_uuid, &m_base_uuid_type);
	APP_ERROR_CHECK(err_code);

	uuid.type = m_base_uuid_type;
	uuid.uuid = MULTILINK_PERIPHERAL_SERVICE_UUID;

	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &uuid, &svc_test);
	APP_ERROR_CHECK(err_code);

	uuid.uuid = MULTILINK_PERIPHERAL_CHAR_UUID;

	memset(&attr, 0, sizeof(ble_gatts_attr_t));
	attr.p_uuid    = &uuid;
	attr.p_attr_md = &attr_md;
	attr.max_len   = 1;
	attr.p_value   = &multilink_peripheral_data;
	attr.init_len  = sizeof(multilink_peripheral_data);

	memset(&attr_md, 0, sizeof(ble_gatts_attr_md_t));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&attr_md.write_perm);
	attr_md.vloc = BLE_GATTS_VLOC_STACK;
	attr_md.vlen = 0;

	memset(&cccd_md, 0, sizeof(ble_gatts_attr_md_t));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&cccd_md.write_perm);
	cccd_md.vloc = BLE_GATTS_VLOC_STACK;

	memset(&char_md, 0, sizeof(ble_gatts_char_md_t));
	char_md.p_cccd_md               = &cccd_md;
	char_md.char_props.notify       = 1;
	char_md.char_props.indicate     = 1;
	char_md.char_props.read         = 1;
	char_md.char_props.write        = 1;
	char_md.char_ext_props.wr_aux   = 1;
	char_md.p_user_desc_md          = &char_ud_md;
	char_md.p_char_user_desc        = multilink_peripheral_ud;
	char_md.char_user_desc_size     = (uint8_t)strlen((char *)multilink_peripheral_ud);
	char_md.char_user_desc_max_size = (uint8_t)strlen((char *)multilink_peripheral_ud);

	memset(&char_ud_md, 0, sizeof(ble_gatts_attr_md_t));
	char_ud_md.vloc = BLE_GATTS_VLOC_STACK;
	char_ud_md.vlen = 1;
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&char_ud_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&char_ud_md.write_perm);

	err_code = sd_ble_gatts_characteristic_add(BLE_GATT_HANDLE_INVALID,
			&char_md,
			&attr,
			&m_char_handles);
	APP_ERROR_CHECK(err_code);
}


int main(void)
{
	ble_stack_init();

	gap_params_init();
	advertising_init();
	services_init();
	advertising_start();

	while (1)
		power_manage();

	return 0;
}
