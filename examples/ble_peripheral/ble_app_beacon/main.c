/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include <nrfx_nvmc.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf.h"
#include "nrf_uart.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_delay.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "app_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nvram.h"
#include "crc.h"
#include "ibeaconinf.h"

#if NRFX_CHECK(NRFX_WDT_ENABLED)
#include <nrfx_wdt.h>
#endif

#if NRFX_CHECK(NRFX_SAADC_ENABLED)
#include "vbat.h"
#endif

#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define SYS_RESTART_TIMEOUT             APP_TIMER_TICKS(600000) //10min

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256 

#define DATA_BUF_LEN                    200
#define DEFAULT_UART_AT_TEST_LEN        4
#define DEFAULT_UART_AT_RSP_LEN         6
#define DEFAULT_UART_AT_CMD_LEN         49

#define APP_ADV_INTERVAL                480                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 300 ms). */

#define APP_ADV_DURATION                0                                           /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_ADV_TXPOWER                 0

#define APP_BEACON_INFO_LENGTH          0x17                               /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                               /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xB5                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x004C                             /**< Company identifier for APPLE. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                         /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                         /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0xFD, 0xA5, 0x06, 0x93, \
                                        0xA4, 0xE2, 0x4F, 0xB1, \
                                        0xAF, 0xCF, 0xC6, 0xEB, \
                                        0x07, 0x64, 0x78, 0x25             /**< Proprietary UUID for Beacon. */ 

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                         /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

#if NRFX_CHECK(NRFX_SAADC_ENABLED)
static uint8_t   vbat_grad;
#endif

_APP_TIMER_DEF(sys_restart_timeout_id);

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

ibeaconinf_t sys_inf;

static uint8_t data_array[DATA_BUF_LEN];

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0
    }
};


static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

volatile uint8_t w_data_ptr = 0;
volatile uint8_t r_data_ptr = 0;
volatile uint8_t overflow   = 0;
/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. 
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY: 
            if(overflow < DATA_BUF_LEN)
            {
                UNUSED_VARIABLE(app_uart_get(&data_array[w_data_ptr++]));
                w_data_ptr = w_data_ptr%DATA_BUF_LEN;
                overflow ++;    
            }          
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_9600
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */

void uart_data_tx(uint8_t *buf, uint8_t len)
{
    if((buf == NULL) || (len == 0))
        return;
    
    do{
        app_uart_put(*buf++);     
    }while(--len);
}

const tx_pow_t  tx_power_table[]=
{
    {-30,0xA0}, {-20,0xAB},  {-16,0xB0},
    {-12,0xB1}, {-8,0xB2},   {-4,0xBD},
    {0,0xB5},   {3,0XC5},    {4,0XD0},
};

const tx_interval_t tx_interval_table[]=
{
    {1,1400},   //875ms
    {2,800},    //500ms
    {3,480},    //300ms
    {4,400},    //250ms
    {5,320},    //200ms
    {10,160},   //100ms
    {20,80},    //50ms
    {30,48},    //30ms
    {50,32}     //20ms
};
/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    
    ble_gap_addr_t     addr;
    
    uint32_t      adv_interval;    
    int8_t        tx_power_level;
    
    uint8_t       i;
    
    ble_advdata_t advdata;
    
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;    
    
    ble_advdata_manuf_data_t manuf_specific_data;
    
    if(sys_inf.txPower >= sizeof(tx_power_table)/sizeof(tx_pow_t))
    {
        tx_power_level = tx_power_table[APP_ADV_TXPOWER].power;
        m_beacon_info[22] = tx_power_table[APP_ADV_TXPOWER].rxp;
    }
    else
    {
        tx_power_level = tx_power_table[sys_inf.txPower].power;
        m_beacon_info[22] = tx_power_table[sys_inf.txPower].rxp;
    }    
    
    memcpy(&m_beacon_info[2], sys_inf.uuidValue, DEFAULT_UUID_LEN);
    memcpy(&m_beacon_info[18], sys_inf.majorValue, sizeof(uint32_t));

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    
    for(i=0; i<(sizeof(tx_interval_table)/sizeof(tx_interval_t)); i++)
    {
        if(sys_inf.txInterval == tx_interval_table[i].tx_index)
        {
                adv_interval = tx_interval_table[i].interval;
                break;
        }
    }
    if(i == sizeof(tx_interval_table)/sizeof(tx_interval_t))
        adv_interval = APP_ADV_INTERVAL;

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = adv_interval;
    m_adv_params.duration        = 0;       // Never time out.

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
    
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, tx_power_level); 
    APP_ERROR_CHECK(err_code);
    
    sd_ble_gap_addr_get(&addr);
    if(addr.addr_type != BLE_GAP_ADDR_TYPE_PUBLIC)
    {
        addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
        sd_ble_gap_addr_set(&addr);
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

//Consistent with setting dev_information
const uint8_t D_FRT[10] ={'2','0','2','0','-','0','9','-','3','0'};                
const uint8_t D_FR[14]={'F','M','V','E','R','S','I','O','N','_','0','0','0','4'}; 

const  uint8_t D_CKey[16]={0xDE,0x48,0x2B,0x1C,0x22,0x1C,0x6C,0x30,0x3C,0xF0,0x50,0xEB,0x00,0x20,0xB0,0xBD}; 
static uint8_t  data_res[DEFAULT_UART_AT_CMD_LEN];
void uart_at_handle(uint8_t *buf)
{
    uint8_t     mac_res[6];
    
    //uuid
    memcpy(&sys_inf.uuidValue, &data_res[5], DEFAULT_UUID_LEN);
    //major & minor
    memcpy(&sys_inf.majorValue, &data_res[21], sizeof(uint32_t));
    //hwvr
    memcpy(&sys_inf.hwvr, &data_res[35], sizeof(uint32_t));
    //txPower
    memcpy(&sys_inf.txPower, &data_res[39], sizeof(uint8_t));
    if(sys_inf.txPower >= sizeof(tx_power_table)/sizeof(tx_pow_t))
    {
        sys_inf.txPower = APP_ADV_TXPOWER;
        sys_inf.Rxp     = tx_power_table[APP_ADV_TXPOWER].rxp;
    }
    else
    {
        sys_inf.Rxp     = tx_power_table[sys_inf.txPower].rxp;
    }           
    //txInterval
    memcpy(&sys_inf.txInterval, &data_res[40], sizeof(uint8_t));
    if(sys_inf.txInterval > tx_interval_table[5].tx_index)
       sys_inf.txInterval =  tx_interval_table[5].tx_index;
    
    //mdate
    memcpy(&sys_inf.mDate, &data_res[25], sizeof(sys_inf.mDate));
    
    /***** Just to get through production. ***********/
    memcpy(mac_res, &data_res[41], sizeof(mac_res));
    memset(data_res, 0, sizeof(data_res));
    memcpy(data_res, mac_res, sizeof(mac_res));
    
    data_res[6] = sys_inf.txPower;
    data_res[7] = sys_inf.txInterval;
    
    memcpy(&data_res[8], sys_inf.majorValue, sizeof(uint32_t));
    memcpy(&data_res[12], sys_inf.uuidValue, DEFAULT_UUID_LEN);
    memcpy(&data_res[28], sys_inf.mDate, sizeof(sys_inf.mDate));
    
    data_res[38] = sys_inf.Rxp;
    
    memcpy(&data_res[39], &sys_inf.hwvr[0], sizeof(sys_inf.hwvr)); 
    
    if(nrfx_nvmc_page_erase(BL_BACKUP_ADD) != NRFX_SUCCESS)
        return;
    
    nrfx_nvmc_bytes_write(BL_BACKUP_ADD, data_res, 43);
    
    uart_data_tx((uint8_t *)"OK+1\r\n", 6);
}

void uart_at_resp(void)
{
    uint8_t const *dst;
    uint8_t crc_8;
    
    memset(data_res, 0, sizeof(data_res));
    
    dst = (const uint8_t *)BL_BACKUP_ADD;
    
    memcpy(data_res, dst, 43);
    
    uart_data_tx(((uint8_t *)"OK+"), 3);
    uart_data_tx(data_res, 43);
    uart_data_tx((uint8_t *)D_FRT,      sizeof(D_FRT));
    uart_data_tx((uint8_t *)&D_FR[10],  sizeof(uint32_t));
    uart_data_tx((uint8_t *)D_CKey,     sizeof(D_CKey));
    
    nrf_delay_ms(100); //wait for uart
    
    sys_inf.atFlag = AT_FLAG_DONE;
    memset(data_res, 0, sizeof(data_res));
    memcpy(data_res, &sys_inf.txPower, sizeof(sys_inf));
    crc_8 = crc8(0, &sys_inf.txPower, sizeof(sys_inf));
    data_res[sizeof(sys_inf)] = crc_8;
                            
    nvram_block_write(data_res, sizeof(sys_inf)+sizeof(uint8_t)); 

    NVIC_SystemReset();           
}

void uart_data_handle(void)
{
    uint8_t     r_heard;
    uint8_t     rx_cnt;
    uint8_t     data_len;
    uint8_t     *ptr;
    uint8_t     res;
    
    r_heard = r_data_ptr;
    
    ptr = data_array;
    
    rx_cnt  = overflow;
    
    if(rx_cnt < DEFAULT_UART_AT_TEST_LEN)
        return;
     
    if(*(ptr + r_heard) != 'A')
         goto  INVALID_PACKET;
    
    r_heard ++;
    r_heard = r_heard%DATA_BUF_LEN;
    
    if(*(ptr + r_heard) != 'T')
        goto  INVALID_PACKET;
    
    r_heard ++;
    r_heard = r_heard%DATA_BUF_LEN;    
    
    if(*(ptr + r_heard) == '\r')
    {
        r_heard ++;
        r_heard = r_heard%DATA_BUF_LEN;
        
        if(*(ptr + r_heard) == '\n')
        {
            uart_data_tx((uint8_t *)"OK\r\n", sizeof(uint32_t));
            
            data_len = DEFAULT_UART_AT_TEST_LEN;
            
            goto COMPLETE_PACKET;
        }
    }
    else if(*(ptr + r_heard) == '+')
    {
        r_heard ++;
        r_heard = r_heard%DATA_BUF_LEN;
        
        if(*(ptr + r_heard) == '1')
        {
            r_heard ++;
            r_heard = r_heard%DATA_BUF_LEN;

            if(*(ptr + r_heard) == '=')
            {
                if(rx_cnt >= DEFAULT_UART_AT_CMD_LEN)
                {
                    rx_cnt = DEFAULT_UART_AT_CMD_LEN;
                    
                    memset(data_res, 0, sizeof(data_res));
                    
                    if(r_data_ptr + rx_cnt <= DATA_BUF_LEN)
                    {       
                        r_heard += (rx_cnt - sizeof("AT+1=") +1);
                        memcpy(data_res, ptr + r_data_ptr, rx_cnt);
                    }
                    else
                    {
                        res = DATA_BUF_LEN - r_data_ptr;
                        r_heard = rx_cnt - res;
                        memcpy(data_res, ptr + r_data_ptr, res);
                        memcpy(data_res + res, ptr, r_heard);                        
                    }
                    
                    uart_at_handle(data_res);
                    
                    data_len = DEFAULT_UART_AT_CMD_LEN;
                    
                    goto COMPLETE_PACKET;  
                }
                else
                    return;
            }
            else
               return;
        }
        else if(*(ptr+r_heard) == '?')
        {       
            uart_at_resp();           
        }   
    }
    else
        goto INVALID_PACKET;
                  
      
INVALID_PACKET:
        r_data_ptr ++;
        r_data_ptr = r_data_ptr%DATA_BUF_LEN;
        overflow --;  
COMPLETE_PACKET:  
        overflow    -= data_len;
        r_data_ptr   = ++r_heard;
}

void sys_restart_timeout_handle(void * p_context)
{
    NVIC_SystemReset();   
}

#if NRFX_CHECK(NRFX_WDT_ENABLED)
nrfx_wdt_config_t wdt_config = NRFX_WDT_DEAFULT_CONFIG;
nrfx_wdt_channel_id wdt_channel_id;
#endif  

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    timers_init();
    
    nvram_init();

    nvram_block_read(&sys_inf, sizeof(sys_inf));
    if(sys_inf.atFlag == AT_FLAG_DEFAULT)
    {
        uart_init();
    }
    else
    {        
        power_management_init();
    }
    
#if NRFX_CHECK(NRFX_WDT_ENABLED)
    nrfx_wdt_init(&wdt_config, NULL);
    nrfx_wdt_channel_alloc(&wdt_channel_id);
    nrfx_wdt_enable();
#endif 
    
#if NRFX_CHECK(NRFX_SAADC_ENABLED)
    nrf_delay_ms(2);
    vbat_init();
    nrf_delay_ms(2);
    vbat_grad = vbat_read_value();    
#endif 
    
    ble_stack_init();
    
    advertising_init();

    advertising_start();
    
    app_timer_create(&sys_restart_timeout_id, APP_TIMER_MODE_SINGLE_SHOT, sys_restart_timeout_handle);

    app_timer_start(sys_restart_timeout_id, SYS_RESTART_TIMEOUT, NULL);

    for (;; )
    {
        if(sys_inf.atFlag == AT_FLAG_DEFAULT)
        {
            uart_data_handle();
        }
        else
        {
           idle_state_handle();
        }
        
#if NRFX_CHECK(NRFX_WDT_ENABLED)
        nrfx_wdt_feed();
#endif        
    }
}


/**
 * @}
 */
