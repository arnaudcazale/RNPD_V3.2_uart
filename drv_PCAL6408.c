#include "drv_PCAL6408.h"

#include "nrf_delay.h"


static struct
{
    drv_PCAL6408_init_t        init;             ///< TWI configuration.
//    void                      (*cb)(void);      ///< Callback. Invoked when a pin interrupt is caught by GPIOTE.
    bool                      initialized;      ///< Module initialized.
    bool                      int_registered;   ///<
    bool                      enabled;          ///< Driver enabled.
//    uint32_t                  evt_sheduled;     ///< Number of scheduled events.
} m_PCAL6408 = {.initialized = false, .int_registered = false};

/**@brief Function to init / allocate the TWI module
 */
uint32_t drv_PCAL6408_init(drv_PCAL6408_init_t * p_params)
{
    if(p_params != NULL)
    {
        m_PCAL6408.init.p_twi_cfg      = p_params->p_twi_cfg;
        m_PCAL6408.init.p_twi_instance = p_params->p_twi_instance;
        m_PCAL6408.initialized         = true;    
    }

    uint32_t err_code;
    err_code = nrf_drv_twi_init(m_PCAL6408.init.p_twi_instance, m_PCAL6408.init.p_twi_cfg, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(m_PCAL6408.init.p_twi_instance);

    //Set to push_pull
    uint8_t data = 0x00;
    err_code = drv_PCAL6408_write(PCAL6408_ADDR, PCAL6408_OUTPUT_PORT_CONFIGURATION, &data);
    APP_ERROR_CHECK(err_code);

    //Configure to output
    data = 0x00;
    err_code = drv_PCAL6408_write(PCAL6408_ADDR, PCAL6408_CONFIGURATION, &data);
    APP_ERROR_CHECK(err_code);

    //Set all to 0
    data = 0x00;
    err_code = drv_PCAL6408_write(PCAL6408_ADDR, PCAL6408_OUTPUT_PORT, &data);
    APP_ERROR_CHECK(err_code);

    /*for(uint8_t i = 0; i<5; i++)
    {
      set_led(i);
      nrf_delay_ms(250);
    }*/
    
    return NRF_SUCCESS;
}

uint32_t set_led(uint32_t led_number)
{
    uint32_t err_code;

    //Set all to 0
    uint8_t data = 0x00;
    err_code = drv_PCAL6408_write(PCAL6408_ADDR, PCAL6408_OUTPUT_PORT, &data);
    APP_ERROR_CHECK(err_code);

    uint8_t pin_output = 1 << led_number;
    err_code = drv_PCAL6408_write(PCAL6408_ADDR, PCAL6408_OUTPUT_PORT, &pin_output);
    APP_ERROR_CHECK(err_code);

    return err_code;
}

uint32_t drv_PCAL6408_write(unsigned char slave_addr, unsigned char command, unsigned char const * p_data)
{
    uint32_t err_code;
    uint8_t buffer[2];
    buffer[0] = command;
    memcpy(&buffer[1], p_data, 1);

    err_code = nrf_drv_twi_tx( m_PCAL6408.init.p_twi_instance,
                               slave_addr,
                               buffer,
                               2,
                               false);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("drv_AD5263_write Failed!\r\n");
    }

    return err_code;
}

uint32_t drv_PCAL6408_read(unsigned char slave_addr, unsigned char * data)
{
    uint32_t err_code;

    err_code = nrf_drv_twi_rx( m_PCAL6408.init.p_twi_instance,
                               slave_addr,
                               data,
                               1 );
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("drv_AD5263_read Failed!\r\n");
    }

    return err_code;
}