/*
 * vbat.h
 */
#ifndef __VBAT_H__
#define __VBAT_H__

#include <stdint.h>
#include <stdbool.h>
#include <nrfx.h>

void vbat_init(void);

uint8_t vbat_read_value(void);

#endif
