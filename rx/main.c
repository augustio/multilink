#include <stdint.h>
#include <string.h>

#include "softdevice_handler.h"
#include "ble_advertising.h"
#include "simple_uart.h"
#include "nrf_gpio.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "IRLED_cntrl.h"

#include "action.h"
#include "common.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0
#define APP_ADV_INTERVAL                   MSEC_TO_UNITS(50, UNIT_0_625_MS)
#define APP_ADV_TIMEOUT_IN_SECONDS         0

#define DEVICE_NAME TARGET_DEVICE_NAME
#define MIN_CONN_INTERVAL                  MSEC_TO_UNITS(500, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL                  MSEC_TO_UNITS(1000, UNIT_1_25_MS)          /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                      0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                   MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define MULTILINK_PERIPHERAL_BASE_UUID     {{0xB3, 0x58, 0x55, 0x40, 0x50, 0x60, 0x11, \
	                                           0xe3, 0x8f, 0x96, 0x08, 0x00, 0x00, 0x00,   \
	                                           0x9a, 0x66}}

#define MULTILINK_PERIPHERAL_SERVICE_UUID  0x9001
#define MULTILINK_PERIPHERAL_CHAR_UUID     0x900A

#define COMPANY_IDENTIFIER 0xDADA

#define ROOM_ID 0xF
#define DEVICE_ID 0x3
#define PRIMARY_CONTINUOUS 0
#define DEVICE_COMMANDS 5

#define RX_GREEN_LED_PIN (8)

#define APP_TIMER_PRESCALER	0
#define APP_TIMER_MAX_TIMERS	4
#define APP_TIMER_QUEUE_SIZE	8

#define DOOR_PULSE_TIMER_TICKS (APP_TIMER_TICKS(500, APP_TIMER_PRESCALER))
#define MOTOR_ON_TIMER_TICKS (APP_TIMER_TICKS(600, APP_TIMER_PRESCALER))

#define ROTATION_CW (1)
#define ROTATION_CCW (2)

#define DEVICE_TYPE_SWITCH		(1)
#define DEVICE_TYPE_PULSE_GENERATOR	(2)
#define DEVICE_TYPE_MOTOR_CONTROL	(3)
#define DEVICE_TYPE_IR_CONTROL		(4)

//#define DEVICE_TYPE DEVICE_TYPE_SWITCH
//#define DEVICE_TYPE DEVICE_TYPE_PULSE_GENERATOR
//#define DEVICE_TYPE DEVICE_TYPE_MOTOR_CONTROL
#define DEVICE_TYPE DEVICE_TYPE_IR_CONTROL

#define HIGH_SIDE_CONNECTOR_PIN_7 (9)
#define HIGH_SIDE_CONNECTOR_PIN_8 (10)
#define LOW_SIDE_CONNECTOR_PIN_5 (2)
#define LOW_SIDE_CONNECTOR_PIN_4 (1)

#define PULSE_SWITCH_PIN LOW_SIDE_CONNECTOR_PIN_4 

#define ADV_BLINKING
#undef ADV_BLINKING

#ifdef ADV_BLINKING
#define ADV_BLINKING_INTERVAL	APP_TIMER_TICKS(100, APP_TIMER_PRESCALER) 
app_timer_id_t m_advblink_timer;
#endif

#if ((DEVICE_TYPE == DEVICE_TYPE_PULSE_GENERATOR) || (DEVICE_TYPE == DEVICE_TYPE_MOTOR_CONTROL))
app_timer_id_t m_pulse_generator_timer;
#endif

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

static uint8_t m_base_uuid_type;
static ble_gatts_char_handles_t  m_char_handles;
static dm_application_instance_t m_app_handle;

static bool output_busy = false;

static void advertising_start()
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

#ifdef ADV_BLINKING
	err_code = app_timer_start(m_advblink_timer, ADV_BLINKING_INTERVAL, NULL);
#endif
}

static void process_data(uint8_t data)
{
#if ((DEVICE_TYPE == DEVICE_TYPE_PULSE_GENERATOR) || (DEVICE_TYPE == DEVICE_TYPE_MOTOR_CONTROL))
	uint32_t err_code;
#endif

	if (output_busy)
		return;

	switch (data) {
	case ACTION_ARM_UP:
		simple_uart_putstring((const uint8_t*) "UP\r\n");
#if (DEVICE_TYPE == DEVICE_TYPE_SWITCH)
		nrf_gpio_pin_toggle(PULSE_SWITCH_PIN);
#elif (DEVICE_TYPE == DEVICE_TYPE_PULSE_GENERATOR)
		{
			nrf_gpio_pin_set(PULSE_SWITCH_PIN);
			err_code = app_timer_start(m_pulse_generator_timer, DOOR_PULSE_TIMER_TICKS, NULL);
			APP_ERROR_CHECK(err_code);
			output_busy = true;
		}
#elif (DEVICE_TYPE == DEVICE_TYPE_IR_CONTROL)
		Sony_control(SONY_CMD_ON_OFF);
#endif
	break;

	case ACTION_ROTATION_CW:
	simple_uart_putstring((const uint8_t*) "CW\r\n");
#if (DEVICE_TYPE == DEVICE_TYPE_MOTOR_CONTROL)
	nrf_gpio_pin_set(LOW_SIDE_CONNECTOR_PIN_5);
	nrf_gpio_pin_set(HIGH_SIDE_CONNECTOR_PIN_7);
	err_code = app_timer_start(m_pulse_generator_timer, MOTOR_ON_TIMER_TICKS, (void *)ROTATION_CW);
	APP_ERROR_CHECK(err_code);
	output_busy = true;
#elif (DEVICE_TYPE == DEVICE_TYPE_IR_CONTROL)
	Sony_control(SONY_CMD_VOL_UP);
#endif
	break;

	case ACTION_ROTATION_CCW:
#if (DEVICE_TYPE == DEVICE_TYPE_MOTOR_CONTROL)
	nrf_gpio_pin_set(LOW_SIDE_CONNECTOR_PIN_4);
	nrf_gpio_pin_set(HIGH_SIDE_CONNECTOR_PIN_8);
	err_code = app_timer_start(m_pulse_generator_timer, MOTOR_ON_TIMER_TICKS, (void *)ROTATION_CCW);
	APP_ERROR_CHECK(err_code);
	output_busy = true;
#elif (DEVICE_TYPE == DEVICE_TYPE_IR_CONTROL)
	Sony_control(SONY_CMD_VOL_DOWN);
#endif
	simple_uart_putstring((const uint8_t*) "CCW\r\n");
	break;

	case ACTION_SWIPE_RIGHT:
#if (DEVICE_TYPE == DEVICE_TYPE_IR_CONTROL)
	Sony_control(SONY_CMD_P_UP);
#endif
		simple_uart_putstring((const uint8_t*) "RIGHT\r\n");
	break;

	case ACTION_SWIPE_LEFT:
#if (DEVICE_TYPE == DEVICE_TYPE_IR_CONTROL)
	Sony_control(SONY_CMD_P_DOWN);
#endif
	simple_uart_putstring((const uint8_t*) "LEFT\r\n");
	break;

	default:
		simple_uart_putstring((const uint8_t*) "SHOULDN'T HAPPEN\r\n");
	break;
	}
}

static void on_ble_evt(ble_evt_t *p_ble_evt)
{
	char buf[32];
	ble_gap_addr_t *peer_addr;

	switch (p_ble_evt->header.evt_id) {
	case BLE_GAP_EVT_CONNECTED:
		
	peer_addr = &p_ble_evt->evt.gap_evt.params.connected.peer_addr;
	sprintf(buf, "%02X %02X %02X %02X %02X %02X\r\n",
			peer_addr->addr[0], peer_addr->addr[1], peer_addr->addr[2],
			peer_addr->addr[3], peer_addr->addr[4], peer_addr->addr[5]);

		simple_uart_putstring((const uint8_t*) "Connected to ");
		simple_uart_putstring((const uint8_t*) buf);

		if ((peer_addr->addr[0] != 0x5e) ||
			(peer_addr->addr[2] != 0x36) ||
			(peer_addr->addr[3] != 0x03) ||
			(peer_addr->addr[4] != 0xc9) ||
			(peer_addr->addr[5] != (0xc1 | 0xc0)))
			sd_nvic_SystemReset();
#ifdef ADV_BLINKING
		app_timer_stop(m_advblink_timer);
#endif
		nrf_gpio_pin_set(RX_GREEN_LED_PIN);
	break;

	case BLE_GAP_EVT_DISCONNECTED:
		simple_uart_putstring((const uint8_t*) "Disconnected\r\n");
		nrf_gpio_pin_clear(RX_GREEN_LED_PIN);
		advertising_start();
	break;

	case BLE_GATTS_EVT_WRITE:
		process_data(p_ble_evt->evt.gatts_evt.params.write.data[0]);
	break;

	default:
	break;
	}
}

static void ble_evt_dispatch(ble_evt_t *p_ble_evt)
{
	dm_ble_evt_handler(p_ble_evt);
	on_ble_evt(p_ble_evt);
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

	ble_advdata_manuf_data_t manuf_data;
	static uint8_t data_data[2] = {0xA1, 0x05};

	memset(&advdata, 0, sizeof(advdata));

	advdata.name_type               = BLE_ADVDATA_FULL_NAME;
	advdata.include_appearance      = true;
	advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

	options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
	options.ble_adv_fast_interval = APP_ADV_INTERVAL;
	options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

	/* Manufacturer's own data begins */

	data_data[0] = (ROOM_ID << 4) | DEVICE_ID;
	data_data[1] = (PRIMARY_CONTINUOUS << 4) | DEVICE_COMMANDS;

	manuf_data.company_identifier = COMPANY_IDENTIFIER;
	manuf_data.data.size = 2;
	manuf_data.data.p_data = data_data;

	advdata.p_manuf_specific_data = &manuf_data;

	/* Manufacturer's own data ends */

	err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
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

#ifdef ADV_BLINKING
static void advblink_handler(void *p_context)
{
	nrf_gpio_pin_toggle(RX_GREEN_LED_PIN);
}
#endif

static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);
    return NRF_SUCCESS;
}

static void device_manager_init(bool erase_bonds)
{
	uint32_t               err_code;
	dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
	dm_application_param_t register_param;

	// Initialize persistent storage module.
	err_code = pstorage_init();
	APP_ERROR_CHECK(err_code);

	err_code = dm_init(&init_param);
	APP_ERROR_CHECK(err_code);

	memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

	register_param.sec_param.bond         = SEC_PARAM_BOND;
	register_param.sec_param.mitm         = SEC_PARAM_MITM;
	register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
	register_param.sec_param.oob          = SEC_PARAM_OOB;
	register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
	register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
	register_param.evt_handler            = device_manager_evt_handler;
	register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

	err_code = dm_register(&m_app_handle, &register_param);
	APP_ERROR_CHECK(err_code);
}

static void pulse_timer_handler(void *p_context)
{
#if (DEVICE_TYPE == DEVICE_TYPE_PULSE_GENERATOR)
	nrf_gpio_pin_clear(PULSE_SWITCH_PIN);
#elif (DEVICE_TYPE == DEVICE_TYPE_MOTOR_CONTROL)
	{
		uint32_t direction = (uint32_t)p_context;
		if (ROTATION_CW == direction) {
			nrf_gpio_pin_clear(HIGH_SIDE_CONNECTOR_PIN_7);
			nrf_gpio_pin_clear(LOW_SIDE_CONNECTOR_PIN_5);
		} else if (ROTATION_CCW == direction) {
			nrf_gpio_pin_clear(HIGH_SIDE_CONNECTOR_PIN_8);
			nrf_gpio_pin_clear(LOW_SIDE_CONNECTOR_PIN_4);
		}
	}
#endif
	output_busy = false;
}

static void actuators_init(void)
{

#if ((DEVICE_TYPE == DEVICE_TYPE_PULSE_GENERATOR) || (DEVICE_TYPE == DEVICE_TYPE_MOTOR_CONTROL))
	uint32_t err_code;
	err_code = app_timer_create(&m_pulse_generator_timer, APP_TIMER_MODE_SINGLE_SHOT, pulse_timer_handler);
	APP_ERROR_CHECK(err_code);
#endif

#if ((DEVICE_TYPE == DEVICE_TYPE_SWITCH) || (DEVICE_TYPE == DEVICE_TYPE_PULSE_GENERATOR))
	nrf_gpio_cfg_output(PULSE_SWITCH_PIN);
	nrf_gpio_pin_clear(PULSE_SWITCH_PIN);
#endif

#if (DEVICE_TYPE == DEVICE_TYPE_MOTOR_CONTROL)
	nrf_gpio_cfg_output(HIGH_SIDE_CONNECTOR_PIN_7);
	nrf_gpio_cfg_output(HIGH_SIDE_CONNECTOR_PIN_8);
	nrf_gpio_cfg_output(LOW_SIDE_CONNECTOR_PIN_5);
	nrf_gpio_cfg_output(LOW_SIDE_CONNECTOR_PIN_4);
	nrf_gpio_pin_clear(HIGH_SIDE_CONNECTOR_PIN_7);
	nrf_gpio_pin_clear(HIGH_SIDE_CONNECTOR_PIN_8);
	nrf_gpio_pin_clear(LOW_SIDE_CONNECTOR_PIN_5);
	nrf_gpio_pin_clear(LOW_SIDE_CONNECTOR_PIN_4);
#endif

#ifdef ADV_BLINKING
	err_code = app_timer_create(&m_advblink_timer, APP_TIMER_MODE_REPEATED, advblink_handler);
	APP_ERROR_CHECK(err_code);
#endif
}

int main(void)
{

	simple_uart_config(6, 10, 5, 4, false);

	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_QUEUE_SIZE, NULL);

	nrf_gpio_cfg_output(RX_GREEN_LED_PIN);
	actuators_init();

	ble_stack_init();
	device_manager_init(true);
	gap_params_init();
	advertising_init();
	services_init();
	advertising_start();

	simple_uart_putstring((const uint8_t *)"RX goes main loop\r\n");

	while (1)
		power_manage();

	return 0;
}
