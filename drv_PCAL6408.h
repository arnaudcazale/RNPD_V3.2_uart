#ifndef DRV_PCAL6408_H__
#define DRV_PCAL6408_H__

#include <stdint.h>
#include <stddef.h>
#include "nrf_drv_twi.h"
#include "nrf_log.h"

#define PCAL6408_ADDR                     0x20

#define PCAL6408_CONFIGURATION             0x03
#define PCAL6408_OUTPUT_PORT_CONFIGURATION 0x4F
#define PCAL6408_OUTPUT_PORT               0x01
 
enum
{
    DRV_PCAL6408_STATUS_CODE_SUCCESS,            ///< Successful.
    DRV_PCAL6408_STATUS_CODE_INVALID_PARAM,      ///< Invalid parameters.
    DRV_PCAL6408_STATUS_WRONG_DEVICE,            ///< Wrong device at I2C (TWI) address.
    DRV_PCAL6408_STATUS_UNINITALIZED,            ///< The driver is unitialized, please initialize.
};

/*enum
{
    AD5263_CHANNEL_1,            
    AD5263_CHANNEL_2,
    AD5263_CHANNEL_3,
    AD5263_CHANNEL_4,
};*/

/**@brief TWI communication initialization struct.
 */
typedef struct
{
    nrf_drv_twi_t         const * p_twi_instance;
    nrf_drv_twi_config_t  const * p_twi_cfg;
}drv_PCAL6408_init_t;


/**@brief Function for initializing the MPU-9250 driver.
 *
 * @param[in] p_params      Pointer to the init paramter structure.
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_PCAL6408_init(drv_PCAL6408_init_t * p_params);
uint32_t drv_PCAL6408_write(unsigned char slave_addr, unsigned char instruction_byte, unsigned char const * p_data);
uint32_t drv_PCAL6408_read(unsigned char slave_addr, unsigned char * p_data);

uint32_t set_led(uint32_t led_number);
uint32_t clear_leds(void);


#endif // DRV_PCAL6408_H__

/** @} */

