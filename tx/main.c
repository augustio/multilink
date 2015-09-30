#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <math.h>

#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "simple_uart.h"
#include "softdevice_handler.h"
#include "device_manager.h"
#include "pstorage.h"

#include "sw_spi.h"
#include "vibrate.h"
#include "gesture.h"
#include "action.h"

#define APP_GPIOTE_MAX_USERS 2

#define SCAN_INTERVAL 0x00A0  /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW   0x0050  /**< Determines scan window in units of 0.625 millisecond. */

#define MIN_CONNECTION_INTERVAL    MSEC_TO_UNITS(7.5, UNIT_1_25_MS)
#define MAX_CONNECTION_INTERVAL    MSEC_TO_UNITS(30, UNIT_1_25_MS)
#define SUPERVISION_TIMEOUT        MSEC_TO_UNITS(4000, UNIT_10_MS)

#define SEC_PARAM_BOND             0                                  /**< Perform bonding. */
#define SEC_PARAM_MITM             0                                  /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES  BLE_GAP_IO_CAPS_NONE               /**< No I/O capabilities. */
#define SEC_PARAM_OOB              0                                  /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE     7                                  /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE     16                                 /**< Maximum encryption key size. */

#define TARGET_DEVICE_NAME "Electria_receiver"

#define MULTILINK_PERIPHERAL_BASE_UUID     {{0xB3, 0x58, 0x55, 0x40, 0x50, 0x60, 0x11, \
						0xe3, 0x8f, 0x96, 0x08, 0x00, 0x00, 0x00,   \
						0x9a, 0x66}}

#define MULTILINK_PERIPHERAL_SERVICE_UUID  0x9001
#define MULTILINK_PERIPHERAL_CHAR_UUID     0x900A

static ble_gap_scan_params_t m_scan_param; /**< Scan parameters requested for scanning and connection. */

static const ble_gap_conn_params_t m_connection_param =
{
	(uint16_t)MIN_CONNECTION_INTERVAL,   // Minimum connection
	(uint16_t)MAX_CONNECTION_INTERVAL,   // Maximum connection
	0,                                   // Slave latency
	(uint16_t)SUPERVISION_TIMEOUT        // Supervision time-out
};


static bool memory_access_in_progress = false;
static bool vibrating = false;
static bool is_connecting = false;

static dm_application_instance_t    m_dm_app_id;
static uint8_t   m_base_uuid_type;
static uint16_t m_conn_handle;
static dm_handle_t m_dm_device_handle;

#define MAX_DEVICE_COUNT 16
static uint8_t m_device_count;

typedef struct {
	ble_gap_addr_t peer_addr;
	int8_t         rssi;
} electria_device_t;

static electria_device_t device_list[MAX_DEVICE_COUNT];
static ble_gap_addr_t target_addr;

#define APP_TIMER_PRESCALER	0
#define APP_TIMER_MAX_TIMERS	8
#define APP_TIMER_QUEUE_SIZE	12

#define POLLING_INTERVAL APP_TIMER_TICKS(10, APP_TIMER_PRESCALER)

#define IMU_DOUBLE_TAP_PIN (9)

static app_timer_id_t m_polling_timer;
static app_gpiote_user_id_t m_gpiote_user_id;

static ble_db_discovery_t m_ble_db_discovery;

typedef struct
{
	uint8_t  *p_data;
	uint16_t data_len;
} data_t;

enum {
	STATE_SLEEP,
	STATE_CONFIG,
	STATE_RX_SELECT,
	STATE_RX_CHANGE,
	STATE_RX_GATHER,
	STATE_RX_CONTROL,
} global_state ;

static bool double_tap_occurred = false;
static bool is_connected = false;

enum {
	ORIENTATION_UNDEFINED = -1,
	ORIENTATION_NEUTRAL = 0,
	ORIENTATION_UP = 1,
	ORIENTATION_CCW,
	ORIENTATION_CW,
	ORIENTATION_DOWN,
};

static int getAction(void)
{
	static int orientation = ORIENTATION_UNDEFINED;

	getXYZValues();

	if (armIsStationary())
	{
		if (armIsNeutral())
		{
			orientation = ORIENTATION_NEUTRAL;
			return ACTION_NO_ACTION;
		}
		else if (orientation != ORIENTATION_UP && armIsUp())
		{
			orientation = ORIENTATION_UP;
			return ACTION_ARM_UP;
		}
		else if (orientation != ORIENTATION_UP && (armRotation() == -1))
		{
			if (orientation != ORIENTATION_CCW)
			{
				orientation = ORIENTATION_CCW;
				return ACTION_ROTATION_CCW;
			}
		}
		else if (orientation != ORIENTATION_UP && (armRotation() == 1))
		{
			if (orientation != ORIENTATION_CW)
			{
				orientation = ORIENTATION_CW;
				return ACTION_ROTATION_CW;
			}
		}
		else if (orientation != ORIENTATION_DOWN && armIsDown())
		{ 
			orientation = ORIENTATION_DOWN;
			return ACTION_ARM_DOWN;
		}
	}

	// neutral orientation prerequisite for accepting swipes
	if (isGyroEnabled() && orientation == ORIENTATION_NEUTRAL) {

		getGyroValues();

		if (swipeLeft())
		{
			return ACTION_SWIPE_LEFT;
		}
		else if (swipeRight())
		{
			return ACTION_SWIPE_RIGHT;
		}
	}

	return ACTION_NO_ACTION;
}

static void send_data(uint8_t data)
{
	uint32_t err_code;
	ble_gattc_write_params_t write_params;
	uint8_t buf[2];

	uint16_t handle = 0;

	buf[0] = data;
	buf[1] = 0;

	write_params.write_op = BLE_GATT_OP_WRITE_CMD;
	write_params.handle = 0xe;
	write_params.offset = 0;
	write_params.len = sizeof(buf);
	write_params.p_value = buf;

	err_code = sd_ble_gattc_write(handle, &write_params);
	APP_ERROR_CHECK(err_code);
	if (err_code != NRF_SUCCESS) {
		char buf2[32];
		sprintf(buf2, "DATA NOT SENT, REASON %d\r\n", (int)err_code);
		simple_uart_putstring((const uint8_t *)buf2);
	} else {
		simple_uart_putstring((const uint8_t *)"sd_ble_gattc_write() SUCCESSFUL\r\n");
	}
}

static uint8_t get_rx_number_of_commands()
{
	return 5;
}

static void polling_timer_handler(void *p_context)
{
	if (vibrating)
		return;

	uint32_t err_code;

	switch (getAction()) {

	case ACTION_NO_ACTION:
	break;

	case ACTION_ARM_UP:
		simple_uart_putstring((const uint8_t *)"UP\r\n");
		if (STATE_RX_SELECT == global_state) {
			simple_uart_putstring((const uint8_t *)"RX SELECTED\r\n");
			do_vibrate(VIBRATE_DURATION_SHORT,
					VIBRATE_PAUSE_DURATION_NORMAL,
					&vibrating);

			if (get_rx_number_of_commands() == 5)
				gyroEnable();

			global_state = STATE_RX_CONTROL;
		} else if (STATE_RX_CONTROL == global_state) {
			send_data(ACTION_ARM_UP);
			do_vibrate(VIBRATE_DURATION_SHORT,
					VIBRATE_PAUSE_DURATION_NORMAL,
					&vibrating);
		}
	break;

	case ACTION_ROTATION_CCW:
		simple_uart_putstring((const uint8_t *)"CCW\r\n");
		send_data(ACTION_ROTATION_CCW);

		do_vibrate(VIBRATE_DURATION_SHORT,
				VIBRATE_PAUSE_DURATION_NORMAL,
				&vibrating);
	break;

	case ACTION_ROTATION_CW:
		simple_uart_putstring((const uint8_t *)"CW\r\n");
		send_data(ACTION_ROTATION_CW);

		do_vibrate(VIBRATE_DURATION_SHORT,
				VIBRATE_PAUSE_DURATION_NORMAL,
				&vibrating);
	break;

	case ACTION_ARM_DOWN:
		simple_uart_putstring((const uint8_t *)"DOWN\r\n");

		err_code = app_timer_stop(m_polling_timer);
		APP_ERROR_CHECK(err_code);

		if (isGyroEnabled())
			gyroDisable();

		global_state = STATE_SLEEP;
		err_code = sd_ble_gap_disconnect(m_conn_handle,
				BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
	break;

	case ACTION_SWIPE_RIGHT:
		simple_uart_putstring((const uint8_t *)"RIGHT\r\n");
		send_data(ACTION_SWIPE_RIGHT);

		do_vibrate(VIBRATE_DURATION_LONG,
				VIBRATE_PAUSE_DURATION_EXTRA_LONG,
				&vibrating);
	break;

	case ACTION_SWIPE_LEFT:
		simple_uart_putstring((const uint8_t *)"LEFT\r\n");
		send_data(ACTION_SWIPE_LEFT);

		do_vibrate(VIBRATE_DURATION_LONG,
				VIBRATE_PAUSE_DURATION_EXTRA_LONG,
				&vibrating);
	break;

	default:
		simple_uart_putstring((const uint8_t *)"SHOULDN'T HAPPEN\r\n");
	break;
	}
}

#define CHECK_ERROR_CODE do { \
	char buf[8]; \
	sprintf(buf, "%d\r\n", __LINE__); \
	simple_uart_putstring((const uint8_t*)buf); \
} while (0);

static void scan_start(void)
{
	/* For simplicity, we always do nonselective scan */

	uint32_t err_code;
	uint32_t count;

	err_code = pstorage_access_status_get(&count);
	APP_ERROR_CHECK(err_code);

	if (count) {
		memory_access_in_progress = true;
		return;
	}

	m_scan_param.active       = 0;             // Active scanning set.
	m_scan_param.selective    = 0;             // Selective scanning not set.
	m_scan_param.interval     = SCAN_INTERVAL; // Scan interval.
	m_scan_param.window       = SCAN_WINDOW;   // Scan window.
	m_scan_param.p_whitelist  = NULL;          // Provide whitelist.
	m_scan_param.timeout      = 0x0001;        // No timeout.

	err_code = sd_ble_gap_scan_start(&m_scan_param);
	APP_ERROR_CHECK(err_code);

	if (err_code != NRF_SUCCESS)
		CHECK_ERROR_CODE;
}

static uint32_t adv_report_parse(uint8_t type, data_t *p_advdata, data_t *p_typedata)
{
	uint32_t  index = 0;
	uint8_t * p_data;

	p_data = p_advdata->p_data;

	while (index < p_advdata->data_len)
	{
		uint8_t field_length = p_data[index];
		uint8_t field_type   = p_data[index+1];

		if (field_type == type)
		{
			p_typedata->p_data   = &p_data[index+2];
			p_typedata->data_len = field_length-1;
			return NRF_SUCCESS;
		}
		index += field_length + 1;
	}
	return NRF_ERROR_NOT_FOUND;
}

static bool is_our_target_device(data_t *type_data)
{
	return !memcmp(type_data->p_data, TARGET_DEVICE_NAME, type_data->data_len - 1);
}

static bool is_same_peer_addr(const ble_gap_addr_t *src, const ble_gap_addr_t *dst)
{
	int i;

	if (src->addr_type != dst->addr_type)
		return false;

	for (i = 0; i < BLE_GAP_ADDR_LEN; i++)
		if (src->addr[i] != dst->addr[i])
			return false;

	return true;
}

static bool is_address_in_table(const ble_gap_addr_t *addr)
{
	int i;

	for (i = 0; i < m_device_count; i++)
		if (is_same_peer_addr(addr, &device_list[i].peer_addr))
			return true;

	return false;
}

static int compare_rssi(const void *src, const void *dst)
{
	electria_device_t *srcd = (electria_device_t *)src;
	electria_device_t *dstd = (electria_device_t *)dst;
	return dstd->rssi - srcd->rssi;
}

static void on_ble_evt(ble_evt_t *p_ble_evt)
{
	const ble_gap_evt_t *p_gap_evt = &p_ble_evt->evt.gap_evt;

	switch (p_ble_evt->header.evt_id) {
	case BLE_GAP_EVT_ADV_REPORT:
	{
		uint32_t err_code;
		data_t adv_data;
		data_t type_data;

#if 0
		int i;
		for (i = 0; i < p_ble_evt->evt.gap_evt.params.adv_report.dlen; i++) {
			char buf[8];
			sprintf(buf, "%x ", p_ble_evt->evt.gap_evt.params.adv_report.data[i]);
			simple_uart_putstring((const uint8_t *)buf);
		}

		simple_uart_putstring((const uint8_t *)"\r\n");
#endif
		adv_data.p_data = (uint8_t *)p_ble_evt->evt.gap_evt.params.adv_report.data;
		adv_data.data_len = p_gap_evt->params.adv_report.dlen;

		/* If we want to detect our target device using something else than the local name,
		 * change the first argument to that type of data that you are looking for, and
		 * _simultaneously_ change the is_our_device() function above. For example, we
		 * might want to look for a specific room number and such. -DV
		 */
		err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &type_data);

		if (err_code == NRF_SUCCESS) {
#if 0

			simple_uart_putstring((const uint8_t *)"Found good data, which is \r\n");
			sprintf(buf, "\t%s\r\n", type_data.p_data);
			simple_uart_putstring((const uint8_t *)buf);
#endif

			if (is_our_target_device(&type_data)) {

				if (STATE_RX_GATHER == global_state) {
					if (m_device_count < MAX_DEVICE_COUNT && !is_address_in_table(&p_gap_evt->params.adv_report.peer_addr)) {
						char buf[48];
						device_list[m_device_count].rssi = p_gap_evt->params.adv_report.rssi;
						device_list[m_device_count].peer_addr = p_gap_evt->params.adv_report.peer_addr;
						m_device_count++;
						sprintf(buf, "Total device count: %d\r\n", m_device_count);
						simple_uart_putstring((const uint8_t *)buf);
					}
					qsort((void *)device_list, m_device_count, sizeof (electria_device_t), compare_rssi);
				} else if (STATE_RX_CHANGE == global_state && !is_connecting && is_same_peer_addr(&target_addr, &p_gap_evt->params.adv_report.peer_addr)) {

					err_code = sd_ble_gap_connect(&target_addr,
							&m_scan_param,
							&m_connection_param);

					if (err_code != NRF_SUCCESS) {
						char buf[8];
						simple_uart_putstring((const uint8_t *)"Connection failed. Reason: ");
						sprintf(buf, "0x%x\r\n", (unsigned int)err_code);
						simple_uart_putstring((const uint8_t *)buf);
						global_state = STATE_SLEEP;
						do_vibrate(VIBRATE_DURATION_EXTRA_LONG,
								VIBRATE_PAUSE_DURATION_SHORT,
								&vibrating);
					} else {
						is_connecting = true;
						simple_uart_putstring((const uint8_t *)"Started connection process. Please check the RX console now.\r\n");
					}

				}


#if 0
				{
					for (i = 0; i < m_device_count; i++) {
						sprintf(buf, "dev[%d] %02x:%02x:%02x:%02x:%02x:%02x rssi = %d\n\r",
								i,
								device_list[i].peer_addr.addr[0],
								device_list[i].peer_addr.addr[1],
								device_list[i].peer_addr.addr[2],
								device_list[i].peer_addr.addr[3],
								device_list[i].peer_addr.addr[4],
								device_list[i].peer_addr.addr[5],
								device_list[i].rssi);
						simple_uart_putstring((const uint8_t *)buf);
					}

				}

				if (!in_connection) {
					simple_uart_putstring((const uint8_t *)"Not yet in connection, go connect\r\n");
					// not yet in connection, go connect
					in_connection = true;

#if 0
					err_code = sd_ble_gap_connect(&p_gap_evt->params.adv_report.peer_addr,
#else
					err_code = sd_ble_gap_connect(&device_list[0].peer_addr,
#endif
									&m_scan_param,
									&m_connection_param);

					if (err_code != NRF_SUCCESS) {
						simple_uart_putstring((const uint8_t *)"Connection failed. Reason: ");
						sprintf(buf, "0x%x\r\n", (unsigned int)err_code);
						simple_uart_putstring((const uint8_t *)buf);
						in_connection = false;
					} else {
						simple_uart_putstring((const uint8_t *)"Connected. Please check the RX console now.\r\n");
					}

				}
#endif
			}
		}

	}
	break;
	case BLE_GAP_EVT_TIMEOUT:
		if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN) {
			simple_uart_putstring((const uint8_t *)"Scan timeout\r\n");
			if (STATE_RX_GATHER == global_state) {
				if (m_device_count) {
					target_addr = device_list[0].peer_addr;
					global_state = STATE_RX_CHANGE;
					simple_uart_putstring((const uint8_t *)"STATE GATHER->CHANGE\r\n");
					scan_start();
				} else {
					simple_uart_putstring((const uint8_t *)"STATE GATHER->SLEEP\r\n");
					global_state = STATE_SLEEP;
					do_vibrate(VIBRATE_DURATION_EXTRA_LONG,
							VIBRATE_PAUSE_DURATION_SHORT,
							&vibrating);
				}
			} else if (STATE_RX_CHANGE == global_state &&
					!is_connecting) {
					global_state = STATE_SLEEP;
					simple_uart_putstring((const uint8_t *)"STATE CHANGE->SLEEP\r\n");
					do_vibrate(VIBRATE_DURATION_EXTRA_LONG,
							VIBRATE_PAUSE_DURATION_SHORT,
							&vibrating);
			}
		}
		else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
			simple_uart_putstring((const uint8_t *)"Conn request timeout\r\n");
		}
	break;
	}
}

static void client_handling_ble_evt_handler(ble_evt_t *p_ble_evt)
{
	switch (p_ble_evt->header.evt_id) {
	case BLE_GATTC_EVT_WRITE_RSP:
		simple_uart_putstring((const uint8_t*)"BLE_GATTC_EVT_WRITE_RSP\r\n");
		{
			char buf[32];
			sprintf(buf, "WRITE_RSP: status = 0x%hx\r\n", p_ble_evt->evt.gattc_evt.gatt_status);
			simple_uart_putstring((const uint8_t*)buf);
		}
	break;

	case BLE_GATTC_EVT_HVX:
		simple_uart_putstring((const uint8_t*)"BLE_GATTC_EVT_HVX\r\n");
	break;

	case BLE_GATTC_EVT_TIMEOUT:
	break;

	default:
	break;
	}
}

static void on_sys_evt(uint32_t sys_evt)
{
	switch (sys_evt)
	{
	case NRF_EVT_FLASH_OPERATION_SUCCESS:
		/* fall through */
	case NRF_EVT_FLASH_OPERATION_ERROR:

		if (memory_access_in_progress) {
			memory_access_in_progress = false;
			scan_start();
		}
	break;

	default:
	break;
	}
}

static void ble_evt_dispatch(ble_evt_t *p_ble_evt)
{
	dm_ble_evt_handler(p_ble_evt);
	ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
	client_handling_ble_evt_handler(p_ble_evt);
	on_ble_evt(p_ble_evt);
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
	pstorage_sys_event_handler(sys_evt);
	on_sys_evt(sys_evt);
}

static void ble_stack_init()
{
	uint32_t err_code;

	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

	ble_enable_params_t ble_enable_params;

	memset(&ble_enable_params, 0, sizeof(ble_enable_params));

	ble_enable_params.gatts_enable_params.service_changed = false;
	ble_enable_params.gap_enable_params.role              = BLE_GAP_ROLE_CENTRAL;

	err_code = sd_ble_enable(&ble_enable_params);
	APP_ERROR_CHECK(err_code);
	if (err_code != NRF_SUCCESS)
		CHECK_ERROR_CODE;

	err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
	APP_ERROR_CHECK(err_code);
	if (err_code != NRF_SUCCESS)
		CHECK_ERROR_CODE;

	err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
	APP_ERROR_CHECK(err_code);
	if (err_code != NRF_SUCCESS)
		CHECK_ERROR_CODE;
}

static void power_manage(void)
{
	uint32_t err_code = sd_app_evt_wait();
	APP_ERROR_CHECK(err_code);
}

static void gpiote_init(void)
{
	nrf_gpio_cfg_input(IMU_DOUBLE_TAP_PIN, NRF_GPIO_PIN_PULLUP);
	APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}

static void gpiote_event_handler(uint32_t lth, uint32_t htl)
{
	if (vibrating)
		return;

	if (lth)
		simple_uart_putstring((const uint8_t *)"LTH\r\n");

	if (htl) {
		simple_uart_putstring((const uint8_t *)"HTL\r\n");
		double_tap_occurred = true;
	}
}

static void gpiote_start(void)
{
	uint32_t err_code;
	err_code = app_gpiote_user_register(&m_gpiote_user_id,
						0,
						1 << IMU_DOUBLE_TAP_PIN,
						gpiote_event_handler);
	if (err_code != NRF_SUCCESS) {
		simple_uart_putstring((const uint8_t *)"Could not register with GPIOTE\r\n");
	}

	err_code = app_gpiote_user_enable(m_gpiote_user_id);
	if (err_code != NRF_SUCCESS) {
		simple_uart_putstring((const uint8_t *)"Could not enable GPIOTE\r\n");
	}
}

static ret_code_t device_manager_event_handler(const dm_handle_t *p_handle,
						const dm_event_t *p_event,
						const ret_code_t  event_result)
{
	switch (p_event->event_id) {
	char buf[32];
	uint32_t err_code;
	case DM_EVT_CONNECTION:
	{
		ble_gap_addr_t *peer_addr = &p_event->event_param.p_gap_param->params.connected.peer_addr;

		sprintf(buf, "%02X %02X %02X %02X %02X %02X\r\n",
			peer_addr->addr[0], peer_addr->addr[1], peer_addr->addr[2],
			peer_addr->addr[3], peer_addr->addr[4], peer_addr->addr[5]);

		simple_uart_putstring((const uint8_t *)"Connected to ");
		simple_uart_putstring((const uint8_t *)buf);

		m_conn_handle = p_event->event_param.p_gap_param->conn_handle;
		m_dm_device_handle = (*p_handle);

		// Discover peer's services.
		err_code = ble_db_discovery_start(&m_ble_db_discovery,
				p_event->event_param.p_gap_param->conn_handle);
		APP_ERROR_CHECK(err_code);
	}
	break;

	case DM_EVT_DISCONNECTION:
		is_connected = false;

		memset(&m_ble_db_discovery, 0 , sizeof (m_ble_db_discovery));
		m_conn_handle = BLE_CONN_HANDLE_INVALID;

		if (STATE_RX_CHANGE == global_state) {
			scan_start();
		} else {
			err_code = app_timer_start(m_polling_timer, POLLING_INTERVAL, NULL);
			APP_ERROR_CHECK(err_code);

			if (isGyroEnabled()) {
				gyroDisable();
			}

			global_state = STATE_SLEEP;

			do_vibrate(VIBRATE_DURATION_EXTRA_LONG,
					VIBRATE_PAUSE_DURATION_NORMAL,
					&vibrating);
		}

		simple_uart_putstring((const uint8_t *)"DM_EVT_DISCONNECTION\r\n");
	break;

	case DM_EVT_SECURITY_SETUP:
		simple_uart_putstring((const uint8_t *)"DM_EVT_SECURITY_SETUP\r\n");
	break;

	case DM_EVT_SECURITY_SETUP_COMPLETE:
		simple_uart_putstring((const uint8_t *)"DM_EVT_SECURITY_SETUP_COMPLETE\r\n");

		is_connecting = false;
		is_connected = true;
		global_state = STATE_RX_SELECT;

		do_vibrate(VIBRATE_DURATION_SHORT,
				VIBRATE_PAUSE_DURATION_NORMAL,
				&vibrating);

		err_code = app_timer_start(m_polling_timer, POLLING_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code);

	break;

	case DM_EVT_LINK_SECURED:
		simple_uart_putstring((const uint8_t *)"DM_EVT_LINK_SECURED\r\n");
	break;

	case DM_EVT_DEVICE_CONTEXT_LOADED:
		simple_uart_putstring((const uint8_t *)"DM_EVT_DEVICE_CONTEXT_LOADED\r\n");
	break;

	case DM_EVT_DEVICE_CONTEXT_STORED:
		simple_uart_putstring((const uint8_t *)"DM_EVT_DEVICE_CONTEXT_STORED\r\n");
	break;

	case DM_EVT_DEVICE_CONTEXT_DELETED:
		simple_uart_putstring((const uint8_t *)"DM_EVT_DEVICE_CONTEXT_DELETED\r\n");
	break;

	default:
		simple_uart_putstring((const uint8_t *)"UNKNOWN EVENT\r\n");
	break;
	}
}

static void device_manager_init(bool erase_bonds)
{
	uint32_t err_code;
	dm_init_param_t init_param = {.clear_persistent_data = erase_bonds};
	dm_application_param_t register_param;

	err_code = pstorage_init();
	APP_ERROR_CHECK(err_code);

	err_code = dm_init(&init_param);
	APP_ERROR_CHECK(err_code);

	memset(&register_param.sec_param, 0, sizeof (ble_gap_sec_params_t));

	// Event handler to be registered with the module.
	register_param.evt_handler            = device_manager_event_handler;

	// Service or protocol context for device manager to load, store and apply on behalf of application.
	// Here set to client as application is a GATT client.
	register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_CLI_ID;

	// Secuirty parameters to be used for security procedures.
	register_param.sec_param.bond         = SEC_PARAM_BOND;
	register_param.sec_param.mitm         = SEC_PARAM_MITM;
	register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
	register_param.sec_param.oob          = SEC_PARAM_OOB;
	register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
	register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
	register_param.sec_param.kdist_periph.enc = 1;
	register_param.sec_param.kdist_periph.id  = 1;

	err_code = dm_register(&m_dm_app_id, &register_param);
	APP_ERROR_CHECK(err_code);
}

static void timers_init()
{
	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_QUEUE_SIZE, NULL);
}

static void timers_create()
{
	uint32_t err_code;

	err_code = app_timer_create(&m_polling_timer, APP_TIMER_MODE_REPEATED, polling_timer_handler);
	APP_ERROR_CHECK(err_code);
}

#if 0
static void notif_enable(int index)
{
	uint32_t                 err_code;
	ble_gattc_write_params_t write_params;
	uint8_t                  buf[BLE_CCCD_VALUE_LEN];

	buf[0] = BLE_GATT_HVX_NOTIFICATION;
	buf[1] = 0;

	write_params.write_op = BLE_GATT_OP_WRITE_REQ;
	write_params.handle   = m_ble_db_discovery.services[0].charateristics[index].cccd_handle;
	write_params.offset   = 0;
	write_params.len      = sizeof(buf);
	write_params.p_value  = buf;

	err_code = sd_ble_gattc_write(m_ble_db_discovery.conn_handle, &write_params);
	if (err_code != NRF_SUCCESS)
		simple_uart_putstring((const uint8_t *)"sd_ble_gattc_write() FAILED\r\n");
	else
		simple_uart_putstring((const uint8_t *)"NOTIFICATIONS ENABLED\r\n");

	APP_ERROR_CHECK(err_code);
}
#endif

static void db_discovery_evt_handler(ble_db_discovery_evt_t *p_evt)
{
	uint32_t err_code;
	int index;
	bool target_characteristic_found = false;

	if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE) {
		int i;
		for (i = 0; i < p_evt->params.discovered_db.char_count; i++) {
			ble_db_discovery_char_t *p_characteristic;
			p_characteristic = &(p_evt->params.discovered_db.charateristics[i]);

			if ((p_characteristic->characteristic.uuid.uuid == MULTILINK_PERIPHERAL_CHAR_UUID)
					&&
					(p_characteristic->characteristic.uuid.type == m_base_uuid_type)) {
				// Characteristic found. Store the information needed and break.
				simple_uart_putstring((const uint8_t *)"FOUND THE TARGET CHARACTERISTIC\r\n");

				target_characteristic_found = true;
				if (target_characteristic_found)
					index = i;
			}
		}
		simple_uart_putstring((const uint8_t *)"DISCOVERY COMPLETE\r\n");
	} else {
		simple_uart_putstring((const uint8_t *)"DISCOVERY FAILED\r\n");
	}

	if (target_characteristic_found) {
		err_code = dm_security_setup_req(&m_dm_device_handle);
		if (err_code != NRF_SUCCESS) {
				simple_uart_putstring((const uint8_t *)"SECURITY SETUP REQUEST NOT SUCCESSFUL\r\n");
		}
		index = index;
#if 0
		notif_enable(index);
#endif
	}
}

static void service_init()
{
	uint32_t err_code;
	ble_uuid128_t base_uuid = MULTILINK_PERIPHERAL_BASE_UUID;
	ble_uuid_t uuid;

	err_code = ble_db_discovery_init();
	APP_ERROR_CHECK(err_code);

	err_code = sd_ble_uuid_vs_add(&base_uuid, &m_base_uuid_type);
	APP_ERROR_CHECK(err_code);

	uuid.type = m_base_uuid_type;
	uuid.uuid = MULTILINK_PERIPHERAL_SERVICE_UUID;

	err_code = ble_db_discovery_evt_register(&uuid,
			db_discovery_evt_handler);
	APP_ERROR_CHECK(err_code);
}

static bool is_multiple_rooms(void)
{
	// FIXME
	return true;
}

static ble_gap_addr_t get_best_rx_next_room(void)
{
	return device_list[0].peer_addr;
}

int main(void)
{
	global_state = STATE_CONFIG;

	simple_uart_config(11, 12, 11, 11, false);

	ble_stack_init();
	device_manager_init(true);

	service_init();

	spi_sw_master_init();
	spi_register_conf();
	timers_init();
	timers_create();
	vibrate_init();
	gpiote_init();
	gpiote_start();

	global_state = STATE_SLEEP;

	simple_uart_putstring((const uint8_t *)"\r\nTX goes main loop\r\n");

	while (1) {
		if (double_tap_occurred) {
			double_tap_occurred = false;
			if (STATE_SLEEP == global_state) {
				global_state = STATE_RX_GATHER;
				m_device_count = 0;
				do_vibrate(VIBRATE_DURATION_SHORT,
						VIBRATE_PAUSE_DURATION_SHORT,
						&vibrating);
				scan_start();
			} else if (STATE_RX_SELECT == global_state &&
					is_multiple_rooms()) {
				global_state = STATE_RX_CHANGE;
				if (is_connected) {
					uint32_t err_code;
					err_code = app_timer_stop(m_polling_timer);
					APP_ERROR_CHECK(err_code);
					target_addr = get_best_rx_next_room();
					err_code = sd_ble_gap_disconnect(m_conn_handle,
							BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
					APP_ERROR_CHECK(err_code);
				}
			}
		}
		power_manage();
	}

	return 0;
}
