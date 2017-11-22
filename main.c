/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**
 * This file contains the source code for a sample application using both Nordic Gazell proprietary
 * radio protocol and Bluetooth Low Energy radio protocol. In Bluetooth mode, it behave as a Heart
 * Rate sensor, in Gazell mode it behaves as a 'device'.
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "ble_app_gzll_device.h"
#include "ble_app_gzll_hr.h"
#include "ble_app_gzll_ui.h"
#include "ble_app_gzll_common.h"
#include "app_timer.h"
#include "bsp.h"

//from esb sample
#include "sdk_common.h"
#include "nrf_esb.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "app_util.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

//create package payload
nrf_esb_payload_t tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00);
nrf_esb_payload_t rx_payload;

// Store current mode
volatile radio_mode_t running_mode = BLE;

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            NRF_LOG_DEBUG("TX SUCCESS EVENT\r\n");
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            NRF_LOG_DEBUG("TX FAILED EVENT\r\n");
            (void) nrf_esb_flush_tx();
            (void) nrf_esb_start_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            NRF_LOG_DEBUG("RX RECEIVED EVENT\r\n");
            while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
                if (rx_payload.length > 0)
                {
                    NRF_LOG_DEBUG("RX RECEIVED PAYLOAD\r\n");
                }
            }
            break;
    }
    NRF_GPIO->OUTCLR = 0xFUL << 12;
    NRF_GPIO->OUTSET = (p_event->tx_attempts & 0x0F) << 12;
}
uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_delay         = 600;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack       = false;

    err_code = nrf_esb_init(&nrf_esb_config);

    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);

    return err_code;
	}
void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

static void power_manage(void)
{
    if (running_mode == ESB)
    {
        tx_payload.noack = false;
        if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
        {
           tx_payload.data[1]++;
        }
        else
        {
            NRF_LOG_WARNING("Sending packet failed\r\n");
        }
        nrf_delay_us(7000); //retransmitt after 7ms
    }
    else if (running_mode == BLE)
    {
        uint32_t err_code;
        // Use SoftDevice API for power_management when in Bluetooth Mode.
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}

int main(void)
{
		radio_mode_t previous_mode = running_mode;
		
		uint32_t err_code;
		err_code = NRF_LOG_INIT(NULL); //Setup logger.
		APP_ERROR_CHECK(err_code);
		
		NRF_LOG_INFO("Application starts\r\n");
		NRF_LOG_INFO("BLE mode\r\n");
		uart_init();
		clocks_start(); //Start 16 MHz crystal oscillator
		ble_stack_start();
		NRF_LOG_FLUSH(); //process all entries from the buffer

		// Initialize timer module, making it use the scheduler.
		APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

		bsp_init_app();
		ble_hrs_app_start();

		while(true)
		{
			NRF_LOG_FLUSH();
			power_manage(); //main function for sending data
			
			if (bsp_button_is_pressed(BLE_BUTTON_ID))
			{
				running_mode = BLE;
			}
			else if (bsp_button_is_pressed(ESB_BUTTON_ID))
			{
				running_mode = ESB;
			}

			if (running_mode != previous_mode)
			{
				previous_mode = running_mode;
				//change mode from BLE -> ESB
				if (running_mode == ESB)
				{
					NRF_LOG_INFO("ESB transmitting mode\r\n");
					ble_hrs_app_stop(); //stop connection with mobile before disabling the SoftDevice.
					ble_stack_stop(); // sisable the softdevice stack
					
					err_code = bsp_indication_set(BSP_INDICATE_IDLE);
					APP_ERROR_CHECK(err_code);

					
					esb_init(); //enable ESB
				 }
				//change state from BLE -> ESB
				 else if (running_mode == BLE)
				 {
						NRF_LOG_INFO("BLE mode\r\n");
						// Disable ESB.
						nrf_esb_disable(); //takes up to 60ms
						err_code = bsp_indication_set(BSP_INDICATE_IDLE);
						APP_ERROR_CHECK(err_code);
						// Re-enable the softdevice stack.
						ble_stack_start();
						ble_hrs_app_start();
					}
			 }
		}
}
