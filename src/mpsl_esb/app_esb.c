#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <mpsl_timeslot.h>
#include <mpsl.h>
#include <esb.h>
#include <zephyr/bluetooth/bluetooth.h>

#include "app_esb.h"
#include "app_mpsl.h"

#define STACK_SIZE 4096
#define PRIORITY 1

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_esb_prx, CONFIG_APP_ESB_LOG_LEVEL);

RING_BUF_DECLARE(timeslot_evt_ring_buf, 10);

/* Message queue for requesting MPSL API calls to non-preemptible thread */
K_MSGQ_DEFINE(app_timeslot_msgq, sizeof(app_ts_state_t), 10, 4);

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17);



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
	}
}

int esb_initialize_and_rx(void)
{
	int err;
	/* These are arbitrary default addresses. In end user products
	 * different addresses should be used for each set of devices.
	 */
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler;
	config.selective_auto_ack = true;
	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING)) {
		config.use_fast_ramp_up = true;
	}

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

int esb_uninitialize_and_stop_rx(void)
{
    int err = esb_stop_rx();
    if (err) {
        LOG_ERR("ESB RX stop failed, err %d", err);
        return err;
    }

    // err = esb_flush_rx();
    // if (err) {
    //     LOG_ERR("ESB RX flush failed, err %d", err);
    //     return err;
    // }

    // esb_disable();
    return 0;
}

void timeslot_callback(app_ts_state_t state)
{
    // int err = 0;
    // err = k_msgq_put(&app_timeslot_msgq, &state, K_NO_WAIT);
	// if (err) {
	// 	LOG_ERR("Message sent error: %d", err);
	// }
	switch (state) {
	case APP_TS_STARTED:{
		esb_initialize_and_rx();
		LOG_DBG("Callback: Timeslot start\n");
		break;
		}
	case APP_TS_STOPPED:{
		esb_uninitialize_and_stop_rx();
		LOG_DBG("Callback: Timeslot stopped\n");
		break;
		}
	default:{
		LOG_DBG("Callback: Other signal: %d\n", state);
		break;
		}
	}
}

static int app_esb_thread_entry()
{
    // during BLE initialization, the MPSL will also be initialized
    while(!bt_is_ready()){
        k_sleep(K_MSEC(100));
    }

    app_mpsl_timeslot_isr_register(timeslot_callback);
    app_mpsl_timeslot_start();

    while(1) {
		k_sleep(K_FOREVER);
        // app_ts_state_t state = 0;
        // int err = k_msgq_get(&app_timeslot_msgq, &state, K_FOREVER);
        // if (err){
        //     LOG_ERR("Message receive error: %d", err);
        //     continue;
        //     }
        // switch (state) {
        // case APP_TS_STARTED:{
        //     esb_initialize_and_rx();
        //     LOG_INF("Callback: Timeslot start\n");
        //     break;
        //     }
        // case APP_TS_STOPPED:{
        //     esb_uninitialize_and_stop_rx();
        //     LOG_INF("Callback: Timeslot stopped\n");
        //     break;
        //  }
        // default:{
        //     LOG_DBG("Callback: Other signal: %d\n", state);
        //     break;
        //     }
        // }
    }

    return 0;
}

K_THREAD_DEFINE(app_esb_id, STACK_SIZE, app_esb_thread_entry, NULL, NULL, NULL, PRIORITY, 0, 0);