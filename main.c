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
#include "app_timer.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

//#define SIMU

#define ROW_COUNT                 48
#define COLUMN_COUNT              16
#define COL_GROUP                 4
#define COL_BY_GROUP              4
#define OFFSET_SAADC_INPUT        NRF_SAADC_INPUT_AIN1
#define SAADC_CHANNEL             0
#define MIN_SEND_VALUE            0
#define MAX_SEND_VALUE            254

static uint16_t PACKET_START_BYTE = 0xFF;
static nrf_saadc_value_t sample;
//static uint8_t sample;
static volatile bool uart_tx_in_progress = false;
static volatile bool saadc_in_progress = false;

#define SAADC_SAMPLES_IN_BUFFER         4
//#define SAADC_SAMPLE_RATE               250 
static const nrf_drv_timer_t   m_timer = NRF_DRV_TIMER_INSTANCE(3);
static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t       m_ppi_channel;
static uint32_t                m_adc_evt_counter;
static uint8_t                 m_bufferData[SAADC_SAMPLES_IN_BUFFER];
static uint32_t                m_bufferDataMean[96][16];
static uint8_t                 m_bufferDataMeanByte[96][16];


typedef enum
{
   RUN,
   STOP,
   WAIT
}FSM_state_t;

enum
{
    COL_GROUP_0 = 0,	/* C1, C2, C3, C4 */
    COL_GROUP_1,	/* C5, C6, C7, C8 */
    COL_GROUP_2,	/* C9, C10, C11, C12 */
    COL_GROUP_3		/* C13, C14, C15, C16 */
};

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
void gpio_init()
{
    ret_code_t err_code;
    
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
   
    err_code = nrf_drv_gpiote_out_init(MUX, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(LS_1_16, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(LS_17_32, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(LS_33_48, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(LS_A3, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(LS_A2, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(LS_A1, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(LS_A0, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(SET_COL_B, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(SET_COL_A, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(DAC, &out_config);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_out_set(DAC);
}

/**********************************************************************************************************
* saadc_callback
**********************************************************************************************************/
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
        uint16_t adc_value;
        uint8_t value[SAADC_SAMPLES_IN_BUFFER*2];
        uint16_t bytes_to_send;
        uint8_t data;

        // set buffers
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
     			
        // print samples on hardware UART and parse data for BLE transmission
        //printf("ADC event number: %d\r\n",(int)m_adc_evt_counter);
        for (int i = 0; i < SAADC_SAMPLES_IN_BUFFER; i++)
        {
            //printf("buffer%d\r\n", p_event->data.done.p_buffer[i]);
            if(p_event->data.done.p_buffer[i] < 0)
            {
                p_event->data.done.p_buffer[i] = 0;
            }

            m_bufferData[i] = p_event->data.done.p_buffer[i];
            
            //adc_value = p_event->data.done.p_buffer[i];
            //value[i*2] = adc_value;
            //value[(i*2)+1] = adc_value >> 8;
            //printf("data%d\r\n", data);
        }

         // Send data over BLE via NUS service. Create string from samples and send string with correct length.
        /*uint8_t nus_string[50];
        bytes_to_send = sprintf(nus_string, 
                                "CH0: %d\r\nCH1: %d\r\nCH2: %d\r\nCH3: %d",
                                p_event->data.done.p_buffer[0],
                                p_event->data.done.p_buffer[1],
                                p_event->data.done.p_buffer[2],
                                p_event->data.done.p_buffer[3]);

        err_code = ble_nus_data_send(&m_nus, nus_string, &bytes_to_send, m_conn_handle);
        if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }*/
	
        m_adc_evt_counter++;
       
        saadc_in_progress = false;
    }
}

/**********************************************************************************************************
* saadc_init() - 
**********************************************************************************************************/
void saadc_init(void)
{
    ret_code_t err_code;
	
    nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_8BIT;
	
    nrf_saadc_channel_config_t channel_0_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    channel_0_config.gain = NRF_SAADC_GAIN1;
    channel_0_config.reference = NRF_SAADC_REFERENCE_VDD4;
	
    nrf_saadc_channel_config_t channel_1_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
    channel_1_config.gain = NRF_SAADC_GAIN1;
    channel_1_config.reference = NRF_SAADC_REFERENCE_VDD4;
	
    nrf_saadc_channel_config_t channel_2_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);
    channel_2_config.gain = NRF_SAADC_GAIN1;
    channel_2_config.reference = NRF_SAADC_REFERENCE_VDD4;
	
    nrf_saadc_channel_config_t channel_3_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);
    channel_3_config.gain = NRF_SAADC_GAIN1;
    channel_3_config.reference = NRF_SAADC_REFERENCE_VDD4;				
	
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(2, &channel_2_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(3, &channel_3_config);
    APP_ERROR_CHECK(err_code);	

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);   
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

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

/**********************************************************************************************************
* 
**********************************************************************************************************/
void lineSelect(uint8_t line)
{
    if( line < 16)
    {
          nrf_drv_gpiote_out_clear(LS_1_16);
          nrf_drv_gpiote_out_set(LS_17_32);
          nrf_drv_gpiote_out_set(LS_33_48);
    }
    else if( line < 32)
    {
          nrf_drv_gpiote_out_set(LS_1_16);
          nrf_drv_gpiote_out_clear(LS_17_32);
          nrf_drv_gpiote_out_set(LS_33_48);
    }
    else
    {
          nrf_drv_gpiote_out_set(LS_1_16);
          nrf_drv_gpiote_out_set(LS_17_32);
          nrf_drv_gpiote_out_clear(LS_33_48);
    }

    if(line &  1) nrf_drv_gpiote_out_set (LS_A0);
    else nrf_drv_gpiote_out_clear (LS_A0);
    if(line &  (1<<1)) nrf_drv_gpiote_out_set (LS_A1);
    else nrf_drv_gpiote_out_clear (LS_A1);
    if(line &  (1<<2)) nrf_drv_gpiote_out_set (LS_A2);
    else nrf_drv_gpiote_out_clear (LS_A2);
    if(line &  (1<<3)) nrf_drv_gpiote_out_set (LS_A3);
    else nrf_drv_gpiote_out_clear (LS_A3);
}

/**********************************************************************************************************
* 
**********************************************************************************************************/
void colSelect(uint8_t colgroup)
{
    switch( colgroup)
    {
    case COL_GROUP_0:
          nrf_drv_gpiote_out_clear(SET_COL_A);
          nrf_drv_gpiote_out_clear(SET_COL_B);
          break;

    case COL_GROUP_1:
          nrf_drv_gpiote_out_set(SET_COL_A);
          nrf_drv_gpiote_out_clear(SET_COL_B);
          break;

    case COL_GROUP_2:
          nrf_drv_gpiote_out_clear(SET_COL_A);
          nrf_drv_gpiote_out_set(SET_COL_B);
          break;

    case COL_GROUP_3:
          nrf_drv_gpiote_out_set(SET_COL_A);
          nrf_drv_gpiote_out_set(SET_COL_B);
          break;

    default:
          break;
    }
}

void adc_getData(void)
{           
    saadc_in_progress = true;
    NRF_SAADC->TASKS_SAMPLE = 1;
    /*
    // Alternatively, you can use the driver to trigger a sample:
    nrf_drv_saadc_sample();
    */
    while(saadc_in_progress);
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

void process_one_shot(void)
{
    uint32_t err_code;

    send(PACKET_START_BYTE);

    nrf_drv_gpiote_out_clear(MUX);

    for(uint8_t i = 0; i < ROW_COUNT; i ++)
    {
        lineSelect(i);
        nrf_delay_ms(3);
        for(uint8_t j = 0; j < COL_GROUP; j ++)
        {
            colSelect(j);
            adc_getData();
            
            //Send all bytes from column group
            printSerial(m_bufferData[0]);
            printSerial(m_bufferData[1]);
            printSerial(m_bufferData[2]);
            printSerial(m_bufferData[3]);
        }
    }

    nrf_drv_gpiote_out_set(MUX);

    for(uint8_t i = 0; i < ROW_COUNT; i ++)
    {
        lineSelect(i);
        nrf_delay_ms(3);
        for(uint8_t j = 0; j < COL_GROUP; j ++)
        {
            colSelect(j);
            adc_getData();
            
            //Send all bytes from column group
            printSerial(m_bufferData[0]);
            printSerial(m_bufferData[1]);
            printSerial(m_bufferData[2]);
            printSerial(m_bufferData[3]);
        }
    }
   
}

void reset_data()
{
    for(uint8_t i = 0; i < ROW_COUNT*2; i ++)
    {
        for(uint8_t j = 0; j < COL_GROUP; j ++)
        {
            m_bufferDataMean[i][j*4] = 0;
            m_bufferDataMean[i][(j*4)+1] = 0;
            m_bufferDataMean[i][(j*4)+2] = 0;
            m_bufferDataMean[i][(j*4)+3] = 0;
        }
    }
}

void process_multi_shot()
{
    reset_data();

    for(uint8_t i = 0; i < 5; i++)
    {
        nrf_drv_gpiote_out_clear(MUX);

        for(uint8_t i = 0; i < ROW_COUNT; i ++)
        {
            lineSelect(i);
            nrf_delay_ms(3);
            for(uint8_t j = 0; j < COL_GROUP; j ++)
            {
                colSelect(j);
                adc_getData();

                m_bufferDataMean[i][j*4]     += m_bufferData[0];
                m_bufferDataMean[i][(j*4)+1] += m_bufferData[1];
                m_bufferDataMean[i][(j*4)+2] += m_bufferData[2];
                m_bufferDataMean[i][(j*4)+3] += m_bufferData[3];
            }
        }

        nrf_drv_gpiote_out_set(MUX);

        for(uint8_t i = 0; i < ROW_COUNT; i ++)
        {
            lineSelect(i);
            nrf_delay_ms(3);
            for(uint8_t j = 0; j < COL_GROUP; j ++)
            {
                colSelect(j);
                adc_getData();

                m_bufferDataMean[i+ROW_COUNT][j*4]     += m_bufferData[0];
                m_bufferDataMean[i+ROW_COUNT][(j*4)+1] += m_bufferData[1];
                m_bufferDataMean[i+ROW_COUNT][(j*4)+2] += m_bufferData[2];
                m_bufferDataMean[i+ROW_COUNT][(j*4)+3] += m_bufferData[3];
            }
        }
    }

    //Mean
    for(uint8_t i = 0; i < ROW_COUNT*2; i ++)
    {
        for(uint8_t j = 0; j < COL_GROUP; j ++)
        {
            m_bufferDataMean[i][j*4]     /= 5;
            m_bufferDataMean[i][(j*4)+1] /= 5;
            m_bufferDataMean[i][(j*4)+2] /= 5;
            m_bufferDataMean[i][(j*4)+3] /= 5;
        }
    }

    //Send
    send(PACKET_START_BYTE);
    for(uint8_t i = 0; i < ROW_COUNT*2; i ++)
    {
        for(uint8_t j = 0; j < COL_GROUP; j ++)
        {
            m_bufferDataMeanByte[i][j*4]     = (uint8_t)m_bufferDataMean[i][j*4];
            m_bufferDataMeanByte[i][(j*4)+1] = (uint8_t)m_bufferDataMean[i][(j*4)+1];
            m_bufferDataMeanByte[i][(j*4)+2] = (uint8_t)m_bufferDataMean[i][(j*4)+2];
            m_bufferDataMeanByte[i][(j*4)+3] = (uint8_t)m_bufferDataMean[i][(j*4)+3];

            printSerial(m_bufferDataMeanByte[i][j*4]);
            printSerial(m_bufferDataMeanByte[i][(j*4)+1]);
            printSerial(m_bufferDataMeanByte[i][(j*4)+2]);
            printSerial(m_bufferDataMeanByte[i][(j*4)+3]);
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
    
    nrf_drv_gpiote_out_set(MUX);

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
              printf(" \r\nRUN ADC SINGLE\r\n");
              process_one_shot();
              //state_FSM = RUN_SINGLE;
            }
            else if(cr == 'M')
            {
              printf(" \r\nRUN ADC MULTI\r\n");
              process_multi_shot();
            }
            /*else if(cr == 'S')
            {
              printf(" \r\nSTOP ADC\r\n");
              state_FSM = STOP;
            }*/
            cr = "";
        }

        /*switch(state_FSM)
        {
          case RUN_SINGLE:
              process_one_shot();
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
