/*
 * nvram.h
 */
#ifndef __NVRAM_H__
#define __NVRAM_H__

#include <stdint.h>
#include <stdbool.h>

#define BL_CONFIG_ADD   0x0002F000      //size: 4K 
#define BL_BACKUP_ADD   0x0002E000      //size: 4K

#define CRC_CHECK_SUCCESS 0x0000
#define ADDRESS_OFFSET    4

int nvram_init(void);

int nvram_block_write(void *w_buf, uint16_t size);

int nvram_block_read(void *r_buf, uint16_t size);

#endif
