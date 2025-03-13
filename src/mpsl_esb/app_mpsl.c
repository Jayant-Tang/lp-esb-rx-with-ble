/*
* Copyright (c) 2019 Nordic Semiconductor ASA
*
* SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
*/

#include <zephyr/kernel.h>
#include <zephyr/console/console.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/types.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#include <mpsl_timeslot.h>
#include <mpsl.h>
#include <hal/nrf_timer.h>

#include "app_mpsl.h"

LOG_MODULE_REGISTER(app_mpsl, CONFIG_APP_ESB_LOG_LEVEL);

/* MPSL API calls that can be requested for the non-preemptible thread */
enum mpsl_timeslot_call {
    OPEN_SESSION,
    MAKE_REQUEST,
    CLOSE_SESSION,
    RESCHEDULE,
};

#define TIMESLOT_REQUEST_DISTANCE_US CONFIG_APP_ESB_PRX_LISTEN_INTERVAL_US
#define TIMESLOT_LENGTH_US           CONFIG_APP_ESB_PRX_LISTEN_WINDOW_US
#define TIMER_EXPIRY_US (TIMESLOT_LENGTH_US - MPSL_TIMESLOT_EXTENSION_MARGIN_MIN_US - 100)

#define MPSL_THREAD_PRIO             CONFIG_MPSL_THREAD_COOP_PRIO
#define STACKSIZE                    CONFIG_MAIN_STACK_SIZE
#define THREAD_PRIORITY              K_LOWEST_APPLICATION_THREAD_PRIO

#if defined(CONFIG_SOC_SERIES_NRF53X)
    #define LOG_OFFLOAD_IRQn SWI1_IRQn
    #define MPSL_TIMER0 NRF_TIMER0
#elif defined(CONFIG_SOC_SERIES_NRF52X)
    #define LOG_OFFLOAD_IRQn SWI1_EGU1_IRQn
    #define MPSL_TIMER0 NRF_TIMER0
#elif defined(CONFIG_SOC_SERIES_NRF54LX)
    #define LOG_OFFLOAD_IRQn EGU10_IRQn
    #define MPSL_TIMER0 NRF_TIMER10
#elif defined(CONFIG_SOC_SERIES_NRF54HX)
    #define LOG_OFFLOAD_IRQn EGU020_IRQn
    #define MPSL_TIMER0 NRF_TIMER020
#endif /* CONFIG_SOC_SERIES_NRF53X */

/* if we need to continously request for new timeslot */
static bool request_in_cb = true;

static volatile app_mpsl_timeslot_cb_t m_mpsl_cb = NULL;
static volatile bool m_in_timeslot = false;

/* Timeslot requests */
static mpsl_timeslot_request_t timeslot_request_earliest = {
    .request_type = MPSL_TIMESLOT_REQ_TYPE_EARLIEST,
    .params.earliest.hfclk = MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED,
    .params.earliest.priority = MPSL_TIMESLOT_PRIORITY_NORMAL,
    .params.earliest.length_us = TIMESLOT_LENGTH_US,
    .params.earliest.timeout_us = MPSL_TIMESLOT_EARLIEST_TIMEOUT_MAX_US
};

static mpsl_timeslot_request_t timeslot_request_normal = {
    .request_type = MPSL_TIMESLOT_REQ_TYPE_NORMAL,
    .params.normal.hfclk = MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED,
    .params.normal.priority = MPSL_TIMESLOT_PRIORITY_NORMAL,
    .params.normal.distance_us = TIMESLOT_REQUEST_DISTANCE_US,
    .params.normal.length_us = TIMESLOT_LENGTH_US
};

static mpsl_timeslot_request_t timeslot_request_when_blocked = {
    .request_type = MPSL_TIMESLOT_REQ_TYPE_NORMAL,
    .params.normal.hfclk = MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED,
    .params.normal.priority = MPSL_TIMESLOT_PRIORITY_NORMAL,
    .params.normal.distance_us = TIMESLOT_REQUEST_DISTANCE_US * 2,
    .params.normal.length_us = TIMESLOT_LENGTH_US
};

static mpsl_timeslot_signal_return_param_t signal_callback_return_param;

/* Two ring buffers for printing the signal type with different priority from timeslot callback */
RING_BUF_DECLARE(callback_high_priority_ring_buf, 10);
RING_BUF_DECLARE(callback_low_priority_ring_buf, 10);

/* Message queue for requesting MPSL API calls to non-preemptible thread */
K_MSGQ_DEFINE(mpsl_api_msgq, sizeof(enum mpsl_timeslot_call), 10, 4);

static void set_timeslot_active_status(bool active)
{
    unsigned int key = irq_lock();
	if (active) {
		if (!m_in_timeslot) {
			m_in_timeslot = true;
            // NRF_P0->OUTSET = BIT(28);
			m_mpsl_cb(APP_TS_STARTED);
		}
	} else {
		if (m_in_timeslot) {
			m_in_timeslot = false;
            // NRF_P0->OUTCLR = BIT(28);
			m_mpsl_cb(APP_TS_STOPPED);
		}
	}
    irq_unlock(key);
}


void app_mpsl_session_operation(enum mpsl_timeslot_call api_call)
{
    int err = 0;
    err = k_msgq_put(&mpsl_api_msgq, &api_call, K_NO_WAIT);
	if (err) {
		LOG_ERR("Message sent error: %d", err);
		k_oops();
	}
}

void app_mpsl_timeslot_start()
{
    app_mpsl_session_operation(OPEN_SESSION);
    app_mpsl_session_operation(MAKE_REQUEST);
}

void app_mpsl_timeslot_isr_register(app_mpsl_timeslot_cb_t cb)
{
    if(cb == NULL) {
        LOG_ERR("Callback is NULL!");
        k_oops();
    }
    m_mpsl_cb = cb;
}

static mpsl_timeslot_signal_return_param_t *mpsl_timeslot_callback(
    mpsl_timeslot_session_id_t session_id,
    uint32_t signal_type)
{
    (void) session_id; /* unused parameter */
    uint8_t input_data = (uint8_t)signal_type;
    uint32_t input_data_len;

    mpsl_timeslot_signal_return_param_t *p_ret_val = NULL;

    signal_callback_return_param.callback_action =
            MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
        p_ret_val = &signal_callback_return_param;

    switch (signal_type) {

    case MPSL_TIMESLOT_SIGNAL_START:
        /* No return action */
        signal_callback_return_param.callback_action =
            MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
        p_ret_val = &signal_callback_return_param;
        
        // Reset the radio to make sure no configuration remains from BLE
        NVIC_ClearPendingIRQ(RADIO_IRQn);
        NRF_RADIO->POWER = RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos;
        NRF_RADIO->POWER = RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos;
        NVIC_ClearPendingIRQ(RADIO_IRQn);

        /* Setup timer to trigger an interrupt (and thus the TIMER0
        * signal) before timeslot end.
        */
        nrf_timer_cc_set(MPSL_TIMER0, NRF_TIMER_CC_CHANNEL0,
            TIMER_EXPIRY_US);
        nrf_timer_int_enable(MPSL_TIMER0, NRF_TIMER_INT_COMPARE0_MASK);
        
        set_timeslot_active_status(true);
        break;
    case MPSL_TIMESLOT_SIGNAL_TIMER0:

        /* Clear event */
        nrf_timer_int_disable(MPSL_TIMER0, NRF_TIMER_INT_COMPARE0_MASK);
        nrf_timer_event_clear(MPSL_TIMER0, NRF_TIMER_EVENT_COMPARE0);

        if (request_in_cb) {
            /* Request new timeslot when callback returns */
            signal_callback_return_param.params.request.p_next =
                &timeslot_request_normal;
            signal_callback_return_param.callback_action =
                MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST;
        } else {
            /* Timeslot will be ended */
            signal_callback_return_param.callback_action =
                MPSL_TIMESLOT_SIGNAL_ACTION_END;
        }

        p_ret_val = &signal_callback_return_param;
        set_timeslot_active_status(false);
        break;

    case MPSL_TIMESLOT_SIGNAL_RADIO:
        /* No return action */
        input_data_len = ring_buf_put(&callback_high_priority_ring_buf, &input_data, 1);
        signal_callback_return_param.callback_action =
            MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
        p_ret_val = &signal_callback_return_param;

        // Do nothing. Since we have enabled CONFIG_MPSL_DYNAMIC_INTERRUPTS=y and CONFIG_ESB_DYNAMIC_INTERRUPTS=y

        break;

    
		case MPSL_TIMESLOT_SIGNAL_OVERSTAYED:
			LOG_WRN("something overstayed!");
			signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_END;
			p_ret_val = &signal_callback_return_param;
			set_timeslot_active_status(false);
			break;

		case MPSL_TIMESLOT_SIGNAL_CANCELLED:
			LOG_DBG("something cancelled!");
			signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
			p_ret_val = &signal_callback_return_param;
			set_timeslot_active_status(false);
			
			// In this case returning SIGNAL_ACTION_REQUEST causes hardfault. We have to request a new timeslot instead, from thread context. 
			app_mpsl_session_operation(RESCHEDULE);
			break;

		case MPSL_TIMESLOT_SIGNAL_BLOCKED:
			LOG_INF("something blocked!");
			set_timeslot_active_status(false);

			// Request a new timeslot in this case
            signal_callback_return_param.callback_action =
            MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
            p_ret_val = &signal_callback_return_param;
			app_mpsl_session_operation(RESCHEDULE);

            // // Why can't we request a new timeslot this way?
            // // it causes MPSL ASSERT: 106, 491
            // signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST;
            // signal_callback_return_param.params.request.p_next = &timeslot_request_when_blocked;
			// p_ret_val = &signal_callback_return_param;

			break;

		case MPSL_TIMESLOT_SIGNAL_INVALID_RETURN:
			LOG_WRN("something gave invalid return");
			signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_END;
			p_ret_val = &signal_callback_return_param;
			set_timeslot_active_status(false);
			break;

		case MPSL_TIMESLOT_SIGNAL_SESSION_IDLE:
			LOG_INF("idle");

			// Request a new timeslot in this case
			app_mpsl_session_operation(RESCHEDULE);

			signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
			p_ret_val = &signal_callback_return_param;
			set_timeslot_active_status(false);
			break;

		case MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED:
			LOG_INF("Session closed");

			signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
			p_ret_val = &signal_callback_return_param;
			set_timeslot_active_status(false);
			break;

		default:
			LOG_ERR("unexpected signal: %u", signal_type);
			k_oops();
			break;
    }

    return p_ret_val;
}


/* To ensure thread safe operation, call all MPSL APIs from a non-preemptible
* thread.
*/
static void mpsl_nonpreemptible_thread(void)
{
    int err;
    enum mpsl_timeslot_call api_call = 0;

    // NRF_P0->DIRSET = BIT(28);
	// NRF_P0->OUTCLR = BIT(28);

    /* Initialize to invalid session id */
    mpsl_timeslot_session_id_t session_id = 0xFFu;

    while (1) {
        if (k_msgq_get(&mpsl_api_msgq, &api_call, K_FOREVER) == 0) {
            switch (api_call) {
            case OPEN_SESSION:
                err = mpsl_timeslot_session_open(
                    mpsl_timeslot_callback,
                    &session_id);
                if (err) {
                    LOG_ERR("Timeslot session open error: %d", err);
                    k_oops();
                }
                break;
            case MAKE_REQUEST:
                err = mpsl_timeslot_request(
                    session_id,
                    &timeslot_request_earliest);
                if (err) {
                    LOG_ERR("Timeslot request error: %d", err);
                    k_oops();
                }
                break;
            case CLOSE_SESSION:
                err = mpsl_timeslot_session_close(session_id);
                if (err) {
                    LOG_ERR("Timeslot session close error: %d", err);
                    k_oops();
                }
                break;
            case RESCHEDULE:
                err = mpsl_timeslot_request(
                    session_id,
                    &timeslot_request_when_blocked);
                if (err) {
                    LOG_ERR("Timeslot request error: %d", err);
                    k_oops();
                }
                break;
            default:
                LOG_ERR("Wrong timeslot API call");
                k_oops();
                break;
            }
        }
    }
}

K_THREAD_DEFINE(mpsl_nonpreemptible_thread_id, STACKSIZE,
        mpsl_nonpreemptible_thread, NULL, NULL, NULL,
        K_PRIO_COOP(MPSL_THREAD_PRIO), 0, 0);
