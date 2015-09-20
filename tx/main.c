#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "simple_uart.h"
#include "softdevice_handler.h"
#include "sw_spi.h"

#define SCAN_INTERVAL 0x00A0  /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW   0x0050  /**< Determines scan window in units of 0.625 millisecond. */

static ble_gap_scan_params_t m_scan_param; /**< Scan parameters requested for scanning and connection. */

static void on_ble_evt(ble_evt_t *p_ble_evt)
{
	const ble_gap_evt_t *p_gap_evt = &p_ble_evt->evt.gap_evt;

	switch (p_ble_evt->header.evt_id) {
	case BLE_GAP_EVT_ADV_REPORT:
	{
		int i;

		simple_uart_putstring((const uint8_t *)"Adv event\r\n");

		for (i = 0; i < p_ble_evt->evt.gap_evt.params.adv_report.dlen; i++) {
			char buf[8];
			sprintf(buf, "%x ", p_ble_evt->evt.gap_evt.params.adv_report.data[i]);
			simple_uart_putstring((const uint8_t *)buf);
		}

		simple_uart_putstring((const uint8_t *)"\r\n");
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

int main(void)
{
	simple_uart_config(11, 12, 11, 11, false);

	ble_stack_init();
	spi_sw_master_init();

	{
		uint16_t txb = 0x0F00 | 0x8000;
		uint8_t rxb[2] = {0};
		char buf[8];

		spi_sw_master_send_bytes(&txb, rxb, 1);

		sprintf(buf, "%x %x ", rxb[0], rxb[1]);
		simple_uart_putstring((const uint8_t *)buf);
	}
	//scan_start();

	simple_uart_putstring((const uint8_t *)"\r\nTX goes main loop\r\n");

	while (1) {
		power_manage();
	}

	return 0;
}
