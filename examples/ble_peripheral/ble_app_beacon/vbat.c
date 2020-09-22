/*
 * nvram.c
 * Description: nvram managerment for system
 */

#include <nrfx.h>

#if NRFX_CHECK(NRFX_SAADC_ENABLED)
#include <nrfx_saadc.h>
#include "vbat.h"

#define SAADC_CHANNEL       6

nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;

nrf_saadc_channel_config_t saadc_channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);

void vbat_init(void)
{
    nrfx_err_t err_code;
    
    err_code = nrfx_saadc_init(&saadc_config, NULL);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrfx_saadc_channel_init(SAADC_CHANNEL, &saadc_channel_config);
    APP_ERROR_CHECK(err_code);    
}

uint8_t vbat_read_value(void)
{  
    uint8_t  grad;  
    
    float data_res = 0.00;
    
    nrfx_err_t err_code;
    
    nrf_saadc_value_t v_data;
    
    err_code = nrfx_saadc_sample_convert(SAADC_CHANNEL, &v_data);
    APP_ERROR_CHECK(err_code);
       
    data_res = (0.03 + v_data * 6 * 0.6/4096) * 100;
   
    if(data_res < 320)
       grad = 0;
    else if(data_res >= 365)
       grad = 10;
    else   
       grad = (uint8_t)((data_res - 320)/4.5);
          
    return grad;
}

#endif
