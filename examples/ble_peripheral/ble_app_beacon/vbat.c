/*
 * nvram.c
 * Description: nvram managerment for system
 */

#include <nrfx.h>

#if NRFX_CHECK(NRFX_SAADC_ENABLED)
#include <nrfx_saadc.h>
#include "vbat.h"

#define SAADC_CHANNEL       3

nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;

nrf_saadc_channel_config_t saadc_channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);

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

    data_res = ((v_data * 3.6) / 4096) * 130;
   
    if(data_res < 384)
       grad = 0x00;
    else if(data_res >= 450)
       grad = 0x07;
    else
        grad = (uint8_t)(((data_res - 384) / 10) + 1) | 0x00;

    return grad;
}

#endif
