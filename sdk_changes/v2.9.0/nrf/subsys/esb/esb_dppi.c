/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <hal/nrf_egu.h>
#include <hal/nrf_radio.h>
#include <hal/nrf_timer.h>

#include <nrfx_dppi.h>
#include <nrfx_ppib.h>

#include <zephyr/logging/log.h>

#include "esb_peripherals.h"
#include "esb_ppi_api.h"

LOG_MODULE_DECLARE(esb, CONFIG_ESB_LOG_LEVEL);

static uint8_t radio_address_timer_stop;
static uint8_t timer_compare0_radio_disable;
static uint8_t timer_compare1_radio_txen;
static uint8_t disabled_phy_end_egu;
static uint8_t egu_timer_start;
static uint8_t egu_ramp_up;
static uint8_t radio_end_timer_start;

static nrf_dppi_channel_group_t ramp_up_dppi_group;

static uint8_t dppi_timer_stop;
static uint8_t dppi_timer_compare0;
static uint8_t dppi_timer_compare1;
static uint8_t dppi_timer_start_e;
static uint8_t dppi_timer_start_r;

static uint8_t ppib_radio_address_timer_stop;
static uint8_t ppib_timer_compare0_radio_disable;
static uint8_t ppib_timer_compare1_radio_txen;
static uint8_t ppib_equ_timer_start;
static uint8_t ppib_radio_end_timer_start;

static const nrfx_dppi_t dppi = NRFX_DPPI_INSTANCE(10);
static const nrfx_dppi_t dppi20 = NRFX_DPPI_INSTANCE(20);
static const nrfx_ppib_interconnect_t ppib_11_21 = NRFX_PPIB_INTERCONNECT_INSTANCE(11, 21);


static void esb_ppib_subscribe_set(nrfx_ppib_t const * p_instance,
				   uint8_t ppib_channel,
				   uint8_t dppi_channel)
{
	nrfx_ppib_subscribe_set(p_instance,
				nrfx_ppib_send_task_get(p_instance, ppib_channel),
				dppi_channel);
}

static void esb_ppib_publish_set(nrfx_ppib_t const * p_instance,
				 uint8_t ppib_channel,
				 uint8_t dppi_channel)
{
	nrfx_ppib_publish_set(p_instance,
			      nrfx_ppib_receive_event_get(p_instance, ppib_channel),
			      dppi_channel);
}

static void esb_ppib_interconnect_set(nrfx_ppib_interconnect_t const * p_instance,
				      uint8_t ppib_channel,
				      uint8_t left_channel,
				      uint8_t right_channel,
				      bool left_to_right)
{
	if (left_to_right) {
		esb_ppib_subscribe_set(&p_instance->left,
				       ppib_channel,
				       left_channel);

		esb_ppib_publish_set(&p_instance->right,
				     ppib_channel,
				     right_channel);
	} else {
		esb_ppib_subscribe_set(&p_instance->right,
				       ppib_channel,
				       right_channel);

		esb_ppib_publish_set(&p_instance->left,
				     ppib_channel,
				     left_channel);
	}
}

static void esb_ppib_subscribe_clear(nrfx_ppib_t const * p_instance,
				     uint8_t ppib_channel)
{
	nrfx_ppib_subscribe_clear(p_instance,
				  nrfx_ppib_send_task_get(p_instance, ppib_channel));
}

static void esb_ppib_publish_clear(nrfx_ppib_t const * p_instance,
				   uint8_t ppib_channel)
{
	nrfx_ppib_publish_clear(p_instance,
				nrfx_ppib_receive_event_get(p_instance, ppib_channel));
}

static void esb_ppib_interconnect_clear(nrfx_ppib_interconnect_t const * p_instance,
				      uint8_t ppib_channel,
				      bool left_to_right)
{
	if (left_to_right) {
		esb_ppib_subscribe_clear(&p_instance->left, ppib_channel);

		esb_ppib_publish_clear(&p_instance->right, ppib_channel);
	} else {
		esb_ppib_subscribe_clear(&p_instance->right, ppib_channel);

		esb_ppib_publish_clear(&p_instance->left, ppib_channel);
	}
}

void esb_ppi_for_txrx_set(bool rx, bool timer_start, bool fast_switching)
{
	uint32_t channels_mask;

	nrf_egu_event_clear(ESB_EGU, ESB_EGU_EVENT);
	nrf_egu_event_clear(ESB_EGU, ESB_EGU_DPPI_EVENT);

	nrf_egu_publish_set(ESB_EGU, ESB_EGU_EVENT, egu_timer_start);
	nrf_egu_publish_set(ESB_EGU, ESB_EGU_DPPI_EVENT, egu_ramp_up);

	nrf_dppi_channels_include_in_group(ESB_DPPIC, BIT(egu_ramp_up), ramp_up_dppi_group);

	nrf_egu_subscribe_set(ESB_EGU, ESB_EGU_DPPI_TASK, egu_timer_start);
	nrf_radio_subscribe_set(NRF_RADIO, rx ? NRF_RADIO_TASK_RXEN : NRF_RADIO_TASK_TXEN,
				egu_ramp_up);
	nrf_dppi_subscribe_set(ESB_DPPIC,
				nrf_dppi_group_disable_task_get((uint8_t)ramp_up_dppi_group),
				egu_ramp_up);

	nrf_egu_subscribe_set(ESB_EGU, ESB_EGU_TASK, disabled_phy_end_egu);

	if (fast_switching) {
		nrf_radio_subscribe_set(NRF_RADIO, rx ? NRF_RADIO_TASK_TXEN : NRF_RADIO_TASK_RXEN,
					disabled_phy_end_egu);
	}

	if (timer_start) {
		esb_ppib_interconnect_set(&ppib_11_21,
					  ppib_equ_timer_start,
					  egu_timer_start,
					  dppi_timer_start_e,
					  true);

		nrf_timer_subscribe_set(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_START,
					dppi_timer_start_e);
	}

	channels_mask = (BIT(egu_timer_start) |
			 BIT(egu_ramp_up));

	nrf_dppi_channels_enable(ESB_DPPIC, channels_mask);

	channels_mask = BIT(dppi_timer_start_e);

	nrf_dppi_channels_enable(NRF_DPPIC20, channels_mask);
}

void esb_ppi_for_txrx_clear(bool rx, bool timer_start, bool fast_switching)
{
	uint32_t channels_mask;

	channels_mask = BIT(dppi_timer_start_e);

	nrf_dppi_channels_disable(NRF_DPPIC20, channels_mask);

	channels_mask = (BIT(egu_timer_start) |
			 BIT(egu_ramp_up));

	nrf_dppi_channels_disable(ESB_DPPIC, channels_mask);

	nrf_egu_publish_clear(ESB_EGU, ESB_EGU_EVENT);
	nrf_egu_publish_clear(ESB_EGU, ESB_EGU_DPPI_EVENT);

	nrf_egu_subscribe_clear(ESB_EGU, ESB_EGU_DPPI_TASK);
	nrf_radio_subscribe_clear(NRF_RADIO, rx ? NRF_RADIO_TASK_RXEN : NRF_RADIO_TASK_TXEN);
	nrf_dppi_subscribe_clear(ESB_DPPIC,
				 nrf_dppi_group_disable_task_get((uint8_t)ramp_up_dppi_group));
	nrf_egu_subscribe_clear(ESB_EGU, ESB_EGU_TASK);

	nrf_dppi_channels_remove_from_group(ESB_DPPIC, BIT(egu_ramp_up), ramp_up_dppi_group);

	if (fast_switching) {
		nrf_radio_subscribe_clear(NRF_RADIO, rx ? NRF_RADIO_TASK_TXEN :
							  NRF_RADIO_TASK_RXEN);
	}

	if (timer_start) {
		esb_ppib_interconnect_clear(&ppib_11_21, ppib_equ_timer_start, true);

		nrf_timer_subscribe_clear(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_START);
	}
}

void esb_ppi_for_fem_set(void)
{
	nrf_egu_publish_set(ESB_EGU, ESB_EGU_EVENT, egu_timer_start);
	esb_ppib_interconnect_set(&ppib_11_21,
				  ppib_equ_timer_start,
				  egu_timer_start,
				  dppi_timer_start_e,
				  true);
	nrf_timer_subscribe_set(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_START,
				dppi_timer_start_e);

	nrf_dppi_channels_enable(ESB_DPPIC, BIT(egu_timer_start));

	nrf_dppi_channels_enable(NRF_DPPIC20, BIT(dppi_timer_start_e));
}

void esb_ppi_for_fem_clear(void)
{
	nrf_dppi_channels_disable(NRF_DPPIC20, BIT(dppi_timer_start_e));
	nrf_dppi_channels_disable(ESB_DPPIC, BIT(egu_timer_start));

	nrf_egu_publish_clear(ESB_EGU, ESB_EGU_EVENT);
	esb_ppib_interconnect_clear(&ppib_11_21,
				    ppib_equ_timer_start,
				    true);
	nrf_timer_subscribe_clear(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_START);
}

void esb_ppi_for_retransmission_set(void)
{
	nrf_egu_event_clear(ESB_EGU, ESB_EGU_EVENT);

	nrf_timer_publish_set(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE1,
			      dppi_timer_compare1);
	esb_ppib_interconnect_set(&ppib_11_21,
				  ppib_timer_compare1_radio_txen,
				  timer_compare1_radio_txen,
				  dppi_timer_compare1,
				  false);

	nrf_radio_subscribe_set(NRF_RADIO, NRF_RADIO_TASK_TXEN, timer_compare1_radio_txen);
	nrf_egu_subscribe_set(ESB_EGU, ESB_EGU_TASK, disabled_phy_end_egu);

	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING)) {
		nrf_radio_subscribe_set(NRF_RADIO, NRF_RADIO_TASK_RXEN, disabled_phy_end_egu);
	}

	nrf_dppi_channels_enable(NRF_DPPIC20, BIT(dppi_timer_compare1));
	nrf_dppi_channels_enable(ESB_DPPIC, BIT(timer_compare1_radio_txen));
}

void esb_ppi_for_retransmission_clear(void)
{
	nrf_dppi_channels_disable(ESB_DPPIC, BIT(timer_compare1_radio_txen));
	nrf_dppi_channels_disable(NRF_DPPIC20, BIT(dppi_timer_compare1));

	nrf_timer_publish_clear(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE1);
	esb_ppib_interconnect_clear(&ppib_11_21,
				    ppib_timer_compare1_radio_txen,
				    false);

	nrf_radio_subscribe_clear(NRF_RADIO, NRF_RADIO_TASK_TXEN);
	nrf_egu_subscribe_clear(ESB_EGU, ESB_EGU_TASK);

	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING)) {
		nrf_radio_subscribe_clear(NRF_RADIO, NRF_RADIO_TASK_RXEN);
	}
}

void esb_ppi_for_wait_for_ack_set(void)
{
	uint32_t channels_mask;

	nrf_radio_publish_set(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS, radio_address_timer_stop);
	esb_ppib_interconnect_set(&ppib_11_21,
				  ppib_radio_address_timer_stop,
				  radio_address_timer_stop,
				  dppi_timer_stop,
				  true);

	nrf_timer_publish_set(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE0,
			      dppi_timer_compare0);
	esb_ppib_interconnect_set(&ppib_11_21,
				  ppib_timer_compare0_radio_disable,
				  timer_compare0_radio_disable,
				  dppi_timer_compare0,
				  false);

	nrf_timer_subscribe_set(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_STOP,
				dppi_timer_stop);

	nrf_radio_subscribe_set(NRF_RADIO, NRF_RADIO_TASK_DISABLE, timer_compare0_radio_disable);

	channels_mask = (BIT(radio_address_timer_stop) |
			 BIT(timer_compare0_radio_disable));

	nrf_dppi_channels_enable(ESB_DPPIC, channels_mask);

	channels_mask = (BIT(dppi_timer_compare0) |
			 BIT(dppi_timer_stop));

	nrf_dppi_channels_enable(NRF_DPPIC20, channels_mask);
}

void esb_ppi_for_wait_for_ack_clear(void)
{
	uint32_t channels_mask;

	channels_mask = (BIT(dppi_timer_compare0) |
			 BIT(dppi_timer_stop));

	nrf_dppi_channels_disable(NRF_DPPIC20, channels_mask);

	channels_mask = (BIT(radio_address_timer_stop) |
			 BIT(timer_compare0_radio_disable));

	nrf_dppi_channels_disable(ESB_DPPIC, channels_mask);

	nrf_radio_publish_clear(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS);
	esb_ppib_interconnect_clear(&ppib_11_21,
				    ppib_radio_address_timer_stop,
				    true);
	nrf_timer_publish_clear(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_EVENT_COMPARE0);
	esb_ppib_interconnect_clear(&ppib_11_21,
				    ppib_timer_compare0_radio_disable,
				    false);

	nrf_timer_subscribe_clear(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_STOP);

	nrf_radio_subscribe_clear(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
}

void esb_ppi_for_wait_for_rx_set(void)
{
	uint32_t channels_mask;

	nrf_radio_publish_set(NRF_RADIO, ESB_RADIO_EVENT_END, radio_end_timer_start);
	esb_ppib_interconnect_set(&ppib_11_21,
				  ppib_radio_end_timer_start,
				  radio_end_timer_start,
				  dppi_timer_start_r,
				  true);
	nrf_timer_subscribe_set(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_START,
				dppi_timer_start_r);

	channels_mask = (BIT(radio_end_timer_start));

	nrf_dppi_channels_enable(ESB_DPPIC, channels_mask);

	channels_mask = BIT(dppi_timer_start_r);

	nrf_dppi_channels_enable(NRF_DPPIC20, channels_mask);
}

void esb_ppi_for_wait_for_rx_clear(void)
{
	uint32_t channels_mask;

	channels_mask = BIT(dppi_timer_start_r);

	nrf_dppi_channels_disable(NRF_DPPIC20, channels_mask);

	channels_mask = (BIT(radio_end_timer_start));

	nrf_dppi_channels_disable(ESB_DPPIC, channels_mask);

	nrf_radio_publish_clear(NRF_RADIO, ESB_RADIO_EVENT_END);
	esb_ppib_interconnect_clear(&ppib_11_21,
				    ppib_radio_end_timer_start,
				    true);
	nrf_timer_subscribe_clear(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_START);
}

uint32_t esb_ppi_radio_disabled_get(void)
{
	return disabled_phy_end_egu;
}

int esb_ppi_init(void)
{
	nrfx_err_t err;

#if defined(ESB_DPPI_FIXED)

	radio_address_timer_stop = ESB_DPPI_FIRST_FIXED_CHANNEL + 0;
	timer_compare0_radio_disable = ESB_DPPI_FIRST_FIXED_CHANNEL + 1;
	timer_compare1_radio_txen = ESB_DPPI_FIRST_FIXED_CHANNEL + 2;
	disabled_phy_end_egu = ESB_DPPI_FIRST_FIXED_CHANNEL + 3;
	egu_timer_start = ESB_DPPI_FIRST_FIXED_CHANNEL + 4;
	egu_ramp_up = ESB_DPPI_FIRST_FIXED_CHANNEL + 5;
	radio_end_timer_start = ESB_DPPI_FIRST_FIXED_CHANNEL + 6;
	ramp_up_dppi_group = ESB_DPPI_FIRST_FIXED_GROUP + 0;

	ARG_UNUSED(err);

	return -ENOTSUP;

#else

	err = nrfx_dppi_channel_alloc(&dppi, &radio_address_timer_stop);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_alloc(&dppi, &timer_compare0_radio_disable);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_alloc(&dppi, &timer_compare1_radio_txen);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_alloc(&dppi, &disabled_phy_end_egu);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_alloc(&dppi, &egu_timer_start);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_alloc(&dppi, &egu_ramp_up);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	if (IS_ENABLED(CONFIG_ESB_NEVER_DISABLE_TX)) {
		err = nrfx_dppi_channel_alloc(&dppi, &radio_end_timer_start);
		if (err != NRFX_SUCCESS) {
			goto error;
		}
	}

	err = nrfx_dppi_group_alloc(&dppi, &ramp_up_dppi_group);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("gppi_group_alloc failed with: %d\n", err);
		return -ENODEV;
	}

	err = nrfx_dppi_channel_alloc(&dppi20, &dppi_timer_stop);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_alloc(&dppi20, &dppi_timer_compare0);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_alloc(&dppi20, &dppi_timer_compare1);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_alloc(&dppi20, &dppi_timer_start_e);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_alloc(&dppi20, &dppi_timer_start_r);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppib_channel_alloc(&ppib_11_21, &ppib_radio_address_timer_stop);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppib_channel_alloc(&ppib_11_21, &ppib_timer_compare0_radio_disable);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppib_channel_alloc(&ppib_11_21, &ppib_timer_compare1_radio_txen);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppib_channel_alloc(&ppib_11_21, &ppib_equ_timer_start);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppib_channel_alloc(&ppib_11_21, &ppib_radio_end_timer_start);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

#endif /* defined(ESB_DPPI_FIXED) */

	nrf_radio_publish_set(NRF_RADIO, NRF_RADIO_EVENT_DISABLED, disabled_phy_end_egu);
	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING)) {
		nrf_radio_publish_set(NRF_RADIO, NRF_RADIO_EVENT_PHYEND, disabled_phy_end_egu);
	}
	nrf_dppi_channels_enable(ESB_DPPIC, BIT(disabled_phy_end_egu));

	return 0;

#if !defined(ESB_DPPI_FIXED)
error:
	LOG_ERR("gppi_channel_alloc failed with: %d\n", err);
	return -ENODEV;
#endif /* !defined(ESB_DPPI_FIXED) */
}

void esb_ppi_disable_all(void)
{
	uint32_t channels_mask = (BIT(egu_ramp_up) |
				  BIT(disabled_phy_end_egu) |
				  BIT(egu_timer_start) |
				  BIT(radio_address_timer_stop) |
				  BIT(timer_compare0_radio_disable) |
				  BIT(radio_end_timer_start) |
				  (IS_ENABLED(CONFIG_ESB_NEVER_DISABLE_TX) ?
					BIT(timer_compare1_radio_txen) : 0));

	nrf_dppi_channels_disable(ESB_DPPIC, channels_mask);
}

void esb_ppi_deinit(void)
{
	nrfx_err_t err;

	nrf_dppi_channels_disable(ESB_DPPIC, BIT(disabled_phy_end_egu));
	nrf_radio_publish_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING)) {
		nrf_radio_publish_clear(NRF_RADIO, NRF_RADIO_EVENT_PHYEND);
	}

#if defined(ESB_DPPI_FIXED)

	ARG_UNUSED(err);

#else

	err = nrfx_dppi_channel_free(&dppi, radio_address_timer_stop);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_free(&dppi, timer_compare0_radio_disable);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_free(&dppi, timer_compare1_radio_txen);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_free(&dppi, disabled_phy_end_egu);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_free(&dppi, egu_timer_start);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_free(&dppi, egu_ramp_up);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	if (IS_ENABLED(CONFIG_ESB_NEVER_DISABLE_TX)) {
		err = nrfx_dppi_channel_free(&dppi, radio_end_timer_start);
		if (err != NRFX_SUCCESS) {
			goto error;
		}
	}

	err = nrfx_dppi_group_free(&dppi, ramp_up_dppi_group);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_free(&dppi20, dppi_timer_stop);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_free(&dppi20, dppi_timer_compare0);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_free(&dppi20, dppi_timer_compare1);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_free(&dppi20, dppi_timer_start_e);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_dppi_channel_free(&dppi20, dppi_timer_start_r);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppib_channel_free(&ppib_11_21, ppib_radio_address_timer_stop);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppib_channel_free(&ppib_11_21, ppib_timer_compare0_radio_disable);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppib_channel_free(&ppib_11_21, ppib_timer_compare1_radio_txen);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppib_channel_free(&ppib_11_21, ppib_equ_timer_start);
	if (err != NRFX_SUCCESS) {
		goto error;
	}

	err = nrfx_ppib_channel_free(&ppib_11_21, ppib_radio_end_timer_start);
	if (err != NRFX_SUCCESS) {
		goto error;
	}
#endif /* defined(ESB_DPPI_FIXED) */

	return;

#if !defined(ESB_DPPI_FIXED)
/* Should not happen. */
error:
	__ASSERT(false, "Failed to free DPPI resources");
#endif
}
