/**
  @headerfile: ibeaconcfg.h
  $Date: 2019-03-21 $
  $Revision:    $
*/

#ifndef IBEACONINF_H
#define IBEACONINF_H

/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include <stdbool.h>

/*********************************************************************
 * CONSTANTS
 */
#define MANUFACTURE_DATE_LEN 11
#define DEFAULT_UUID_LEN     16

#define AT_FLAG_DEFAULT     0xFF>>2
#define AT_FLAG_DONE        0xFF>>3    
/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * MACROS
 */
typedef struct 
{  
    uint8_t txPower;                          
    uint8_t txInterval;                         
    uint8_t majorValue[2];
    uint8_t minorValue[2];
    uint8_t atFlag;  
	uint8_t Rxp;
    uint8_t uuidValue[16];
    uint8_t hwvr[4];
    uint8_t mDate[11];                
}ibeaconinf_t;

typedef struct
{
    int8_t  power;
    uint8_t rxp;
}tx_pow_t;

typedef struct 
{
    uint8_t tx_index;
    uint32_t interval;
}tx_interval_t;
#endif /* IBEACONCFG_H */
