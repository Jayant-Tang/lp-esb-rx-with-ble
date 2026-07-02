#include <zephyr/kernel.h>
#include <esb.h>
#include <zephyr/bluetooth/bluetooth.h>

#include "app_esb.h"

#define STACK_SIZE 4096
#define PRIORITY 1

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_esb_prx, CONFIG_APP_ESB_LOG_LEVEL);

static struct esb_payload rx_payload;

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS EVENT");
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED EVENT");
		break;
	case ESB_EVENT_RX_RECEIVED:
		if (esb_read_rx_payload(&rx_payload) == 0) {
			LOG_INF("Packet received, len %d : "
				"0x%02x, 0x%02x, 0x%02x, 0x%02x, "
				"0x%02x, 0x%02x, 0x%02x, 0x%02x",
				rx_payload.length, rx_payload.data[0],
				rx_payload.data[1], rx_payload.data[2],
				rx_payload.data[3], rx_payload.data[4],
				rx_payload.data[5], rx_payload.data[6],
				rx_payload.data[7]);
		} else {
			LOG_ERR("Error while reading rx packet");
		}
		break;
	case ESB_EVENT_TIMESLOT_FAILED:
		LOG_ERR("ESB timeslot failed");
		break;
	}
}

static int esb_initialize_and_rx(void)
{
	int err;
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler;
	config.selective_auto_ack = true;

	err = esb_init(&config);
	if (err) {
		LOG_ERR("ESB initialization failed, err %d", err);
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		LOG_ERR("ESB base address 0 set failed, err %d", err);
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err) {
		LOG_ERR("ESB base address 1 set failed, err %d", err);
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err) {
		LOG_ERR("ESB prefixes set failed, err %d", err);
		return err;
	}

	err = esb_start_rx();
	if (err) {
		LOG_ERR("ESB RX start failed, err %d", err);
		return err;
	}

	return 0;
}

static int app_esb_thread_entry(void)
{
	/* BLE initialization brings up MPSL; wait until it is ready
	 * before starting ESB which will request MPSL timeslots.
	 */
	while (!bt_is_ready()) {
		k_sleep(K_MSEC(100));
	}

	int err = esb_initialize_and_rx();

	if (err) {
		LOG_ERR("ESB init failed, err %d", err);
		return err;
	}

	LOG_INF("ESB started (MPSL timeslot mode)");

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}

K_THREAD_DEFINE(app_esb_id, STACK_SIZE, app_esb_thread_entry, NULL, NULL, NULL, PRIORITY, 0, 0);
