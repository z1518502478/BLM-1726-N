/*
 * nvram.c
 * Description: nvram managerment for system
 */

#include <string.h>
#include "crc.h"
#include "nvram.h"
#include "ibeaconinf.h"
#include <nrfx_nvmc.h>

static int nvram_block_set_default(void);
static int nvram_block_check(void);
int nvram_block_write(void *w_buf, uint16_t size);
    
/* Init nvram */
int nvram_init(void)
{
    if(nvram_block_check() != 0)
    {
        uint8_t const *ptr;
        
        uint8_t     crc_8 = 0;
        
        ptr = (const uint8_t *)BL_BACKUP_ADD;
        crc_8 = *((const uint8_t *)(ptr + sizeof(ibeaconinf_t)));
        if(crc_8 != crc8(0, (uint8_t *)ptr, sizeof(ibeaconinf_t)))
        {
            nvram_block_set_default();
        }
        else
        {
            if(nrfx_nvmc_page_erase(BL_CONFIG_ADD) != NRFX_SUCCESS)
                return -1;
                        
            nrfx_nvmc_bytes_write(BL_CONFIG_ADD, ptr, sizeof(ibeaconinf_t) + sizeof(uint8_t));
                       
            if(nrfx_nvmc_page_erase(BL_BACKUP_ADD) != NRFX_SUCCESS)
                return -1;             
        }
    }
    
    return 0;
}

const uint8_t major[] = {0x20, 0x20};  //8224
const uint8_t minor[] = {0x21, 0x21};  //8481
const uint8_t uuid[]  = {0xFD, 0xA5, 0x06, 0x93, 0xA4, 0xE2, 0x4F, 0xB1,
	                     0xAF, 0xCF, 0xC6, 0xEB, 0x07, 0x64, 0x78, 0x25};

static int nvram_block_set_default(void)
{
    uint8_t buf_res[sizeof(ibeaconinf_t) + sizeof(uint8_t)] = {0};
    
    ibeaconinf_t *ptr = (ibeaconinf_t *)buf_res;
    
    ptr->atFlag = AT_FLAG_DEFAULT;
    memcpy(ptr->hwvr, "0001", sizeof(uint32_t));
    memcpy(ptr->majorValue, major, sizeof(major));
    memcpy(ptr->minorValue, minor, sizeof(minor));
    memcpy(ptr->mDate, "2020-09-30", sizeof(ptr->mDate));  
    ptr->Rxp        = 0xB5;
    ptr->txInterval = 3;
    ptr->txPower    = 6;
    memcpy(ptr->uuidValue, uuid, sizeof(uuid));
    
    buf_res[sizeof(ibeaconinf_t)] = crc8(0, (uint8_t *)ptr, sizeof(ibeaconinf_t));
    
    nvram_block_write(ptr, sizeof(buf_res));
    
    return 0;
}

int nvram_block_write(void *w_buf, uint16_t size)
{  
    if((w_buf == NULL) || (size == 0))
        return -1;
    
    if(size > sizeof(ibeaconinf_t) + sizeof(uint8_t))
        return -1;
    
    if(nrfx_nvmc_page_erase(BL_BACKUP_ADD) != NRFX_SUCCESS)
        return -1;

    nrfx_nvmc_bytes_write(BL_BACKUP_ADD, w_buf, size);
    
    if(nrfx_nvmc_page_erase(BL_CONFIG_ADD) != NRFX_SUCCESS)
        return -1;
    
    nrfx_nvmc_bytes_write(BL_CONFIG_ADD, w_buf, size);
    
    if(nrfx_nvmc_page_erase(BL_BACKUP_ADD) != NRFX_SUCCESS)
        return -1;      
    
    return 0;
}
    
static int nvram_block_check(void)
{
    uint8_t crc_8 = 0;
    
    uint8_t const *ptr;
    
    ptr = (const uint8_t *)BL_CONFIG_ADD;
       
    crc_8 = *((const uint8_t *)(ptr + sizeof(ibeaconinf_t)));
    
    if(crc_8 != crc8(0, (uint8_t *)ptr, sizeof(ibeaconinf_t)))
        return -1;
    
    return 0;
}

int nvram_block_read(void *r_buf, uint16_t size)
{
    uint8_t const *dst;
    
    if((r_buf == NULL) || (size == 0))
        return -1;
    
    if(size != sizeof(ibeaconinf_t))
        return -1;
     
    dst = (const uint8_t *)BL_CONFIG_ADD;
    
    memcpy(r_buf, dst, size);

    return size;
}
