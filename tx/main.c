#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <math.h>

#include "ble_hci.h"
#include "app_timer.h"
#include "simple_uart.h"
#include "softdevice_handler.h"
#include "sw_spi.h"
#include "device_manager.h"
#include "pstorage.h"

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

static ble_gap_scan_params_t m_scan_param; /**< Scan parameters requested for scanning and connection. */

static dm_application_instance_t    m_dm_app_id;
static const ble_gap_conn_params_t m_connection_param =
{
	(uint16_t)MIN_CONNECTION_INTERVAL,   // Minimum connection
	(uint16_t)MAX_CONNECTION_INTERVAL,   // Maximum connection
	0,                                   // Slave latency
	(uint16_t)SUPERVISION_TIMEOUT        // Supervision time-out
};

static bool start_scan = false;
static bool scanning = false;
static bool in_connection = false;

static uint16_t m_conn_handle;

#define APP_TIMER_PRESCALER	0
#define APP_TIMER_MAX_TIMERS	6
#define APP_TIMER_QUEUE_SIZE	10

#define POLLING_INTERVAL APP_TIMER_TICKS(10, APP_TIMER_PRESCALER)

static app_timer_id_t m_polling_timer;

typedef struct
{
	uint8_t  *p_data;
	uint16_t data_len;
} data_t;

#define SAVED_ACCELERATION_BUFFER_SIZE (2)
#define n (SAVED_ACCELERATION_BUFFER_SIZE - 1)

typedef struct accel {
	double x[SAVED_ACCELERATION_BUFFER_SIZE];
	double y[SAVED_ACCELERATION_BUFFER_SIZE];
	double z[SAVED_ACCELERATION_BUFFER_SIZE];
} accel_t;

static accel_t acc, gyro;

static double calcDx()
{
	double dxv = acc.x[n] - acc.x[n - 1];
	return dxv;
}

static double dx;
static double dy;
static double dz;
static double g;
static double dg;
static double theta; // Deviation from horizontal in degrees (-90,90). Up positive.
static double phi; // Deviation from semipronation in degrees (-90,90). Clockwise positive.
static double yawv;

static double yaw = 0.0; // FIXME remember to connect this to gyro state

static const double theta_high = 45.0; // degree
static const double theta_low = -60.0; // degree
static const double phi_neutral_cw = 20; //clockwise activation angle, phi_activate/2 degree
static const double phi_neutral_ccw = -20; //counterclockwise activation angle, phi_activate/2 degree
static const double theta_neutral_high = 20.0; // theta_high/2 degree
static const double theta_neutral_low = -45.0; // degree
static const double yaw_limit = 50;
static const double theta_limit_high = 35.0; // degree, above this blocks phi recognition
static const double theta_limit_low = -45.0; // degree, below this blocks phi recognition
static const double phi_cw = 40.0; // degree
static const double phi_ccw = -40.0; // degree
static const double yaw_activate = 200.0; // degree per second

static bool gyro_enabled = false;

//delta y from last value
static double calcDy()
{
	double dyv = acc.y[n] - acc.y[n - 1];
	return dyv;
}

//delta z from last value
static double calcDz()
{
	double dzv = acc.z[n] - acc.z[n - 1];
	return dzv;
}

static double calcG()
{
	double gv = sqrt(pow(acc.x[n], 2.0) + pow(acc.y[n], 2.0) + pow(acc.z[n], 2.0));
	return gv;
}

static double calcDg()
{
	double dgv = sqrt(pow(dx, 2.0) + pow(dy, 2.0) + pow(dz, 2.0));
	return dgv;
}

static double calcTheta()
{
	double thetav = asin(acc.x[n] / g) * 180 / M_PI;
	return thetav;
}

static double calcPhi()
{
	double phiv = atan(
			acc.z[n] / sqrt(0.001 * pow(acc.x[n], 2.0) + pow(acc.y[n], 2.0)))
			* 180 / M_PI;
	if (acc.y[n] > 0) {
		return -phiv;
	} else {
		return phiv;
	}
}

static void getXYZValues()
{
	uint16_t tx_buffer[6];
	uint8_t rx_buffer[6];
	int16_t tmpx, tmpy, tmpz;

	tx_buffer[0] = 0x2800 | 0x8000;
	tx_buffer[1] = 0x2900 | 0x8000;
	tx_buffer[2] = 0x2A00 | 0x8000;
	tx_buffer[3] = 0x2B00 | 0x8000;
	tx_buffer[4] = 0x2C00 | 0x8000;
	tx_buffer[5] = 0x2D00 | 0x8000;

	spi_sw_master_send_bytes(tx_buffer, rx_buffer, 6);

	acc.x[0] = acc.x[1];
	acc.y[0] = acc.y[1];
	acc.z[0] = acc.z[1];

	tmpx = (rx_buffer[0] | (rx_buffer[1] << 8));
	tmpy = (rx_buffer[2] | (rx_buffer[3] << 8));
	tmpz = (rx_buffer[4] | (rx_buffer[5] << 8));

	// Normitus s.e. g = 1 ja poistetaan datasta nollat
	acc.x[1] = -tmpx / (double) 0x2000 + 0.00001; // +x = kyynärpäätä kohti
	acc.y[1] =  tmpy / (double) 0x2000 + 0.00001; //
	acc.z[1] = -tmpz / (double) 0x2000 + 0.00001; // +z = kämmentä kohti
	dx = calcDx();
	dy = calcDy();
	dz = calcDz();
	g = calcG();
	dg = calcDg();
	theta = calcTheta();
	phi = calcPhi();
}

static int armIsUp()
{
	return (theta > theta_high);
}

static int armIsDown()
{
	return (theta < theta_low);
}

static int armIsNeutral()
{
	return (phi < phi_neutral_cw && phi > phi_neutral_ccw
			&& theta < theta_neutral_high && theta > theta_neutral_low);
}

static int armRotation()
{
	if (fabs(yaw) > yaw_limit || theta > theta_limit_high
			|| theta < theta_limit_low || fabs(phi) > 80) {
		return 0;
	} else if (phi > phi_cw) {
		return 1;
	} else if (phi < phi_ccw) {
		return -1;
	} else {
		return 0;
	}
}

static double calcYaw() 
{
	// This could be improved to take  orientation into account
	// Now assumes perfect semipronation
	if (acc.y[n] > 0) {
		yawv = gyro.y[n];
	} else {
		yawv = -gyro.y[n];
	}
	return yawv;
}

static int swipeRight()
{
	return (yaw > yaw_activate && fabs(fabs(acc.y[n]) - 1) < 0.2);
}

static int swipeLeft()
{
	return (yaw < -yaw_activate && fabs(fabs(acc.y[n]) - 1) < 0.2);
}

static void getGyroValues() 
{
	// Yaw only
	uint16_t tx_buffer[2];
	uint8_t rx_buffer[2];
	int16_t tmp;

	tx_buffer[0] = 0x2400 | 0x8000;
	tx_buffer[1] = 0x2500 | 0x8000;
	spi_sw_master_send_bytes(tx_buffer, rx_buffer, 2);

	gyro.y[0] = gyro.y[1];
	tmp = (rx_buffer[0] | (rx_buffer[1] << 8));

	gyro.y[1] = -tmp / 114.28 + 0.001; // Yaw in degrees per second
	yaw = calcYaw(); // If !gyro_enabled: yaw = 0
}

static void polling_timer_handler(void *p_context)
{
	getXYZValues();
	getGyroValues();

	if (armIsDown()) {
		uint32_t err_code;
		err_code = sd_ble_gap_disconnect(m_conn_handle,
				BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);

		simple_uart_putstring((const uint8_t *)"Down\r\n");
	}

#if 0
	if (armIsUp())
		simple_uart_putstring((const uint8_t *)"Up\r\n");

	if (armIsNeutral())
		simple_uart_putstring((const uint8_t *)"Neutral\r\n");
#endif
	if (swipeRight())
		simple_uart_putstring((const uint8_t *)"SRight\r\n");

	if (swipeLeft())
		simple_uart_putstring((const uint8_t *)"SLeft\r\n");
#if 0
	int rot;
	if ((rot = armRotation())) {
		char buf[16];
		sprintf(buf, "rot = %d\r\n", rot);
		simple_uart_putstring((const uint8_t *)buf);
	}
#endif
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

static void on_ble_evt(ble_evt_t *p_ble_evt)
{
	const ble_gap_evt_t *p_gap_evt = &p_ble_evt->evt.gap_evt;

	switch (p_ble_evt->header.evt_id) {
	case BLE_GAP_EVT_ADV_REPORT:
	{
		uint32_t err_code;
		data_t adv_data;
		data_t type_data;
		int i;

		simple_uart_putstring((const uint8_t *)"Adv event\r\n");

		for (i = 0; i < p_ble_evt->evt.gap_evt.params.adv_report.dlen; i++) {
			char buf[8];
			sprintf(buf, "%x ", p_ble_evt->evt.gap_evt.params.adv_report.data[i]);
			simple_uart_putstring((const uint8_t *)buf);
		}

		simple_uart_putstring((const uint8_t *)"\r\n");
		
		adv_data.p_data = (uint8_t *)p_ble_evt->evt.gap_evt.params.adv_report.data;
		adv_data.data_len = p_gap_evt->params.adv_report.dlen;

		/* If we want to detect our target device using something else than the local name,
		 * change the first argument to that type of data that you are looking for, and
		 * _simultaneously_ change the is_our_device() function above. For example, we
		 * might want to look for a specific room number and such. -DV
		 */
		err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &type_data);

		if (err_code == NRF_SUCCESS) {
			char buf[32];

			simple_uart_putstring((const uint8_t *)"Found good data, which is \r\n");
			sprintf(buf, "\t%s\r\n", type_data.p_data);
			simple_uart_putstring((const uint8_t *)buf);

			if (is_our_target_device(&type_data)) {
				simple_uart_putstring((const uint8_t *)"This is our guy, get connected\r\n");
				err_code = sd_ble_gap_scan_stop();
				if (err_code != NRF_SUCCESS) {
					simple_uart_putstring((const uint8_t *)"Failed to properly stop scanning\r\n");
				} else {
					scanning = false;
					in_connection = true;

					err_code = sd_ble_gap_connect(&p_gap_evt->params.adv_report.peer_addr,
									&m_scan_param,
									&m_connection_param);

					if (err_code != NRF_SUCCESS) {
						simple_uart_putstring((const uint8_t *)"Connection failed. Reason: ");
						sprintf(buf, "0x%x\r\n", (unsigned int)err_code);
						simple_uart_putstring((const uint8_t *)buf);
					} else {
						simple_uart_putstring((const uint8_t *)"Connected. Please check the RX console now.\r\n");
					}

				}
			}
		}

	}
	break;
	case BLE_GAP_EVT_TIMEOUT:
		if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN) {
			simple_uart_putstring((const uint8_t *)"Scan timeout\r\n");
			scanning = false;
		}
		else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
			simple_uart_putstring((const uint8_t *)"Conn request timeout\r\n");
			in_connection = false;
		}
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

#define CHECK_ERROR_CODE do { \
	char buf[8]; \
	sprintf(buf, "%d\r\n", __LINE__); \
	simple_uart_putstring((const uint8_t*)buf); \
} while (0);

static void scan_start()
{
	/* For simplicity, we always do nonselective scan */

	uint32_t err_code;

	m_scan_param.active       = 0;             // Active scanning set.
	m_scan_param.selective    = 0;             // Selective scanning not set.
	m_scan_param.interval     = SCAN_INTERVAL; // Scan interval.
	m_scan_param.window       = SCAN_WINDOW;   // Scan window.
	m_scan_param.p_whitelist  = NULL;          // Provide whitelist.
	m_scan_param.timeout      = 0x0000;        // No timeout.

	err_code = sd_ble_gap_scan_start(&m_scan_param);
	APP_ERROR_CHECK(err_code);

	if (err_code != NRF_SUCCESS)
		CHECK_ERROR_CODE;
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

static void spi_register_conf(void)
{
	uint16_t buf_len = 12;
	uint8_t rx_buffer[buf_len];
	uint16_t tx_buffer[buf_len];
	uint16_t *tx = tx_buffer;
	uint8_t *rx = rx_buffer;

	int index;
	for (index = 0; index < buf_len; index++) {
		tx_buffer[index] = 0;
		rx_buffer[index] = 0;
	}

	tx_buffer[0] = 0x1000 | 0x48; // CTRL1_XL: Set 104 Hz, 4 g full scale.
	tx_buffer[1] = 0x1500 | 0x10; // CTRL6_C: Disable high-performance mode for acceleration.
	tx_buffer[2] = 0x1600 | 0x80; // CTRL7_G: Disable high-performance mode for gyro.
	tx_buffer[3] = 0x1900 | 0x10; // CTRL10_C: Disable x (roll) and z (pitch) axes for gyro. 00zyx000
	tx_buffer[4] = 0x5800 | 0x06; // TAP_CFG: Enable tap interrupt in yz directions(011). No latch(0).
	tx_buffer[5] = 0x5900 | 0x08; // TAP_THS_6D: Tap threshold full scale/32 * 8 = 1g.
	tx_buffer[6] = 0x5A00 | 0x3C; // INT_DUR2: Dur(0011)=960ms,Quiet(11)=120ms,Shock(00)=40ms.
	tx_buffer[7] = 0x5B00 | 0xC4; // WAKE_UP_THS: Enable double tap and activity/inactivity (THS=0.25g). Double tap only: 80.
	tx_buffer[8] = 0x5C00 | 0x41; // WAKE_UP_DUR: Wake with 2 points above THS. Sleep in 5 s.
	tx_buffer[9] = 0x5E00 | 0x08; // MD1_CFG: Route double tap to INT1.
	tx_buffer[10] = 0x5F00 | 0x00; // MD2_CFG: Disable INT 2.
	tx_buffer[11] = 0x0000;

	spi_sw_master_send_bytes(tx, rx, buf_len);
}

static void gyroEnable()
{
	uint16_t tx_buffer[2]; //Transmit buffer to send data from SPI master with sample data.
	uint16_t *tx = tx_buffer;

	tx_buffer[0] = 0x1100 | 0x40; // CTRL2_G: Set gyro 104 Hz, 245 dps full scale.
	tx_buffer[1] = 0x0000;
	spi_sw_master_send_bytes(tx, NULL, 2);
	gyro_enabled = 1;
}

static void gyroDisable() 
{	
	uint16_t tx_buffer[2]; //Transmit buffer to send data from SPI master with sample data.
	uint16_t *tx = tx_buffer;

	tx_buffer[0] = 0x1100 | 0x00; // CTRL2_G: Set gyro power down.
	tx_buffer[1] = 0x0000;
	spi_sw_master_send_bytes(tx, NULL, 2);
	gyro_enabled = 0;
}

static void gpio_init(void)
{
	nrf_gpio_cfg_sense_input(9, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_HIGH);
	NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
	NVIC_EnableIRQ(GPIOTE_IRQn);
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

		gyroEnable();

		err_code = app_timer_start(m_polling_timer, POLLING_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code);
	}
	break;

	case DM_EVT_DISCONNECTION:
		simple_uart_putstring((const uint8_t *)"DM_EVENT_DISCONNECTION\r\n");
		err_code = app_timer_stop(m_polling_timer);
		APP_ERROR_CHECK(err_code);

		gyroDisable();

		m_conn_handle = BLE_CONN_HANDLE_INVALID;
		in_connection = false;
	break;

	case DM_EVT_SECURITY_SETUP:
		simple_uart_putstring((const uint8_t *)"DM_EVT_SECURITY_SETUP\r\n");
	break;

	case DM_EVT_SECURITY_SETUP_COMPLETE:
		simple_uart_putstring((const uint8_t *)"DM_EVT_SECURITY_SETUP_COMPLETE\r\n");
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

int main(void)
{
	simple_uart_config(11, 12, 11, 11, false);

	ble_stack_init();
	device_manager_init(true);

	spi_sw_master_init();
	spi_register_conf();
	timers_init();
	timers_create();
	gpio_init();

	simple_uart_putstring((const uint8_t *)"\r\nTX goes main loop\r\n");

	while (1) {
		if (start_scan && !scanning) {
			scanning = true;
			start_scan = false;
			simple_uart_putstring((const uint8_t *)"\r\ngoes scanning\r\n");
			scan_start();
		}
		power_manage();
	}

	return 0;
}

void GPIOTE_IRQHandler(void)
{
	if (NRF_GPIOTE->EVENTS_PORT)
	{
		NRF_GPIOTE->EVENTS_PORT = 0;
		simple_uart_putstring((const uint8_t *)"IRQ handler\r\n");

		if (in_connection)
			return;

		if (!scanning)
			start_scan = true;
	}
}
