#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "simple_uart.h"
#include "softdevice_handler.h"
#include "sw_spi.h"

#define SCAN_INTERVAL 0x00A0  /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW   0x0050  /**< Determines scan window in units of 0.625 millisecond. */

static ble_gap_scan_params_t m_scan_param; /**< Scan parameters requested for scanning and connection. */

static bool start_scan = false;
static bool scanning = false;

typedef struct
{
	uint8_t  *p_data;
	uint16_t data_len;
} data_t;

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

		err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &type_data);

		if (err_code == NRF_SUCCESS) {
			char buf[32];

			simple_uart_putstring((const uint8_t *)"Found good data, which is \r\n");
			sprintf(buf, "\t%s\r\n", type_data.p_data);
			simple_uart_putstring((const uint8_t *)buf);
		}

	}
	break;
	case BLE_GAP_EVT_TIMEOUT:
		if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN) {
			simple_uart_putstring((const uint8_t *)"Scan timeout\r\n");
		}
		else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
			simple_uart_putstring((const uint8_t *)"Conn request timeout\r\n");
		}
	break;
	}
}

static void ble_evt_dispatch(ble_evt_t *p_ble_evt)
{
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

static void gpio_init(void)
{
	nrf_gpio_cfg_sense_input(9, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_HIGH);
	NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
	NVIC_EnableIRQ(GPIOTE_IRQn);
}

int main(void)
{
	simple_uart_config(11, 12, 11, 11, false);

	ble_stack_init();

	spi_sw_master_init();
	spi_register_conf();
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
		if (!scanning)
			start_scan = true;
	}
}
