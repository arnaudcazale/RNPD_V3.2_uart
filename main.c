/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "nrf_drv_gpiote.h"
#include "boards.h"
#include "nrf_drv_saadc.h"

//#define SIMU

#define ROW_COUNT                 48
#define COLUMN_COUNT              16
#define ROWS_PER_MUX              16
#define MUX_COUNT                 3
#define CHANNEL_PINS_PER_MUX      4
#define SAADC_CHANNEL 0
#define MIN_SEND_VALUE       0
#define MAX_SEND_VALUE       254

static uint16_t PACKET_START_BYTE = 0xFF;
static nrf_saadc_value_t sample;
//static uint8_t sample;
static int current_enabled_mux = MUX_COUNT - 1;  //init to number of last mux so enabled mux increments to first mux on first scan.
static volatile bool uart_tx_in_progress = false;
static volatile bool saadc_convert_in_progress = false;

typedef enum
{
   RUN,
   STOP,
   WAIT
}FSM_state_t;

static FSM_state_t state_FSM = WAIT;


//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }else if(p_event->evt_type == APP_UART_TX_EMPTY)
    {
        uart_tx_in_progress = false;
    }
}

/**
* @brief Function 
*/
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{

}

/**
* @brief Function 
*/
void gpio_init()
{
    ret_code_t err_code;
    
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
   
    err_code = nrf_drv_gpiote_out_init(MUX, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(SHIFT_REGISTER_DATA, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(SHIFT_REGISTER_CLOCK, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(PIN_MUX_CHANNEL_0, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(PIN_MUX_CHANNEL_1, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(PIN_MUX_CHANNEL_2, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(PIN_MUX_CHANNEL_3, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(MUX_INIB0, &out_config);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_out_set(MUX_INIB0);

    err_code = nrf_drv_gpiote_out_init(MUX_INIB1, &out_config);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_out_set(MUX_INIB1);

    err_code = nrf_drv_gpiote_out_init(MUX_INIB2, &out_config);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_out_set(MUX_INIB2);

    err_code = nrf_drv_gpiote_out_init(TEST, &out_config);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_out_clear(TEST);
}

/**********************************************************************************************************
* saadc_init() - 
**********************************************************************************************************/
void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);  //P0.03
        channel_config.acq_time = NRF_SAADC_ACQTIME_3US;
        channel_config.gain = NRF_SAADC_GAIN4;
        channel_config.reference = NRF_SAADC_REFERENCE_VDD4; //NRF_SAADC_REFERENCE_INTERNAL;


    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
}

/**********************************************************************************************************
* convert_Mux_nbr_to_Mux_addr() - Convert mux number to the associated pin address
**********************************************************************************************************/
int convert_mux_nbr_to_mux_addr(int current_enabled_mux)
{
    int addr;
    switch(current_enabled_mux)
    {
        case 0:
            addr = MUX_INIB0;
          break;
        case 1:
            addr = MUX_INIB1;
          break;
        case 2:
            addr = MUX_INIB2;
          break;
        default:
          break;
    }
    return addr;
}

/**********************************************************************************************************
* setRow() - Enable single mux IC and channel to read specified matrix row.
**********************************************************************************************************/
void setRow(int row_number)
{
    int addr;
    //NRF_LOG_INFO("CURRENT_MUX %d", current_enabled_mux);
    if((row_number % ROWS_PER_MUX) == 0)  //We've reached channel 0 of a mux IC, so disable the previous mux IC, and enable the next mux IC
    {
        //NRF_LOG_INFO("CHANNEL 0 REACHED");
        //NRF_LOG_INFO("current_enabled_mux %d", current_enabled_mux);
        addr = convert_mux_nbr_to_mux_addr(current_enabled_mux);
        //NRF_LOG_INFO("convert_mux_nbr_to_mux_addr %d", addr);
        nrf_drv_gpiote_out_set(addr);  //Muxes are enabled using offset from MUX_INHIBIT_0. This is why mux inhibits MUST be wired to consecutive Arduino pins!
        current_enabled_mux ++;

        if(current_enabled_mux >= MUX_COUNT)
        {
            current_enabled_mux = 0;
        }

        //NRF_LOG_INFO("current_enabled_mux %d", current_enabled_mux);
        addr = convert_mux_nbr_to_mux_addr(current_enabled_mux);
        //NRF_LOG_INFO("convert_mux_nbr_to_mux_addr %d", addr);
        nrf_drv_gpiote_out_clear(addr);  //enable the next mux, active low
    }

    for(int i = 0; i < CHANNEL_PINS_PER_MUX; i ++)
    {
        int bit = (row_number >> i) & 0x01;
        int channel = (PIN_MUX_CHANNEL_0 + i);
        if(bit)
        {
            nrf_drv_gpiote_out_set(channel);
        }else
        {
            nrf_drv_gpiote_out_clear(channel);
        }
    }

    //nrf_delay_ms(2);
}

/**********************************************************************************************************
* shiftColumn() - Shift out a high bit to drive first column, or increment column by shifting out a low
* bit to roll high bit through cascaded shift register outputs.
**********************************************************************************************************/
void shiftColumn(bool is_first)
{
    if(is_first)
    {
        nrf_drv_gpiote_out_set(SHIFT_REGISTER_DATA);
    }

    nrf_drv_gpiote_out_set(SHIFT_REGISTER_CLOCK);
    nrf_drv_gpiote_out_clear(SHIFT_REGISTER_CLOCK);

    if(is_first)
    {
        nrf_drv_gpiote_out_clear(SHIFT_REGISTER_DATA);
    }
}

/**********************************************************************************************************
* printFixed() - print a value padded with leading spaces such that the value always occupies a fixed
* number of characters / space in the output terminal.
**********************************************************************************************************/
/*void putword(uint16_t value)
{
  uart_tx_in_progress = true;
  putchar(value);
  while(uart_tx_in_progress);

  uart_tx_in_progress = true;
  putchar(value >> 8);
  while(uart_tx_in_progress);
}*/

/**********************************************************************************************************
* printFixed() - print a value padded with leading spaces such that the value always occupies a fixed
* number of characters / space in the output terminal.
**********************************************************************************************************/
void send(uint8_t value)
{
  uart_tx_in_progress = true;
  putchar(value);
  while(uart_tx_in_progress);
}

/**********************************************************************************************************
* printFixed() - print a value padded with leading spaces such that the value always occupies a fixed
* number of characters / space in the output terminal.
**********************************************************************************************************/
void printSerial(uint8_t value)
{
   
    #ifdef SIMU
      send(0xFE);
    #else
      if(value > MAX_SEND_VALUE)
      {
          value = 254;
      }
      send(value);
    #endif
}

#ifdef ENABLE_LOOPBACK_TEST
/* Use flow control in loopback test. */
#define UART_HWFC APP_UART_FLOW_CONTROL_ENABLED

/** @brief Function for setting the @ref ERROR_PIN high, and then enter an infinite loop.
 */
static void show_error(void)
{

    bsp_board_leds_on();
    while (true)
    {
        // Do nothing.
    }
}


/** @brief Function for testing UART loop back.
 *  @details Transmitts one character at a time to check if the data received from the loopback is same as the transmitted data.
 *  @note  @ref TX_PIN_NUMBER must be connected to @ref RX_PIN_NUMBER)
 */
static void uart_loopback_test()
{
    uint8_t * tx_data = (uint8_t *)("\r\nLOOPBACK_TEST\r\n");
    uint8_t   rx_data;

    // Start sending one byte and see if you get the same
    for (uint32_t i = 0; i < MAX_TEST_DATA_BYTES; i++)
    {
        uint32_t err_code;
        while (app_uart_put(tx_data[i]) != NRF_SUCCESS);

        nrf_delay_ms(10);
        err_code = app_uart_get(&rx_data);

        if ((rx_data != tx_data[i]) || (err_code != NRF_SUCCESS))
        {
            show_error();
        }
    }
    return;
}
#else
/* When UART is used for communication with the host do not use flow control.*/
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
#endif

void process_single_shot(void)
{
    uint32_t err_code;

    nrf_drv_gpiote_out_clear(MUX);

    send(PACKET_START_BYTE);
    
    for(int i = 0; i < ROW_COUNT; i ++)
    {
        setRow(i);
        //nrf_delay_ms(1);
        shiftColumn(true);
        shiftColumn(false);  //with SR clks tied, latched outputs are one clock behind
        nrf_delay_us(100);
        for(int j = 0; j < COLUMN_COUNT; j ++)
        {
            nrf_drv_gpiote_out_set(TEST);
            err_code = nrfx_saadc_sample_convert(SAADC_CHANNEL, &sample);
            APP_ERROR_CHECK(err_code);
            nrf_drv_gpiote_out_clear(TEST);

            if(sample < 0)
            {
                sample = 0;
            }
             
            uint8_t data = sample & 0xFF;

            shiftColumn(false);
            nrf_delay_us(100);

            printSerial(data);
        }
    }

    nrf_drv_gpiote_out_set(MUX);

    for(int i = 0; i < ROW_COUNT; i ++)
    {
        setRow(i);
        //nrf_delay_ms(1);
        shiftColumn(true);
        shiftColumn(false);  //with SR clks tied, latched outputs are one clock behind
        nrf_delay_us(100);
        for(int j = 0; j < COLUMN_COUNT; j ++)
        {
            nrf_drv_gpiote_out_set(TEST);
            err_code = nrfx_saadc_sample_convert(SAADC_CHANNEL, &sample);
            APP_ERROR_CHECK(err_code);
            nrf_drv_gpiote_out_clear(TEST);

            if(sample < 0)
            {
                sample = 0;
            }
             
            uint8_t data = sample & 0xFF;

            shiftColumn(false);
            nrf_delay_us(100);

            printSerial(data);
        }
    }
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    uint32_t err_code;

    gpio_init();
    saadc_init();
   
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          UART_HWFC,
          false,
#if defined (UART_PRESENT)
          NRF_UART_BAUDRATE_115200
#else
          NRF_UARTE_BAUDRATE_115200
#endif
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);

#ifndef ENABLE_LOOPBACK_TEST
    //printf("\r\nUART example started.\r\n");

    while (true)
    {
        uint8_t cr;
        //while (app_uart_get(&cr) != NRF_SUCCESS);
        //while (app_uart_put(cr) != NRF_SUCCESS);

        if(app_uart_get(&cr))
        {
            if(cr == 'S')
            {
              //printf(" \r\nRUN ADC\r\n");
              //state_FSM = RUN;
              process_single_shot();
            }
            else if(cr == 'M')
            {
              //printf(" \r\nSTOP ADC\r\n");
              //state_FSM = STOP;
            }
            cr = "";
        }

        /*switch(state_FSM)
        {
          case RUN:
              process();
            break;

          case STOP:

            break;

          case WAIT:

            break;

          default:
            break;
        }*/
    }
#else

    // This part of the example is just for testing the loopback .
    while (true)
    {
        uart_loopback_test();
    }
#endif
}


/** @} */
