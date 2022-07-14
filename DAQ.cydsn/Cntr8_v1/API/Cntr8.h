/* ========================================
 *
 * Copyright UCSC, 2020
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * ========================================
*/
#if !defined(`$INSTANCE_NAME`_Cntr8_H)
#define `$INSTANCE_NAME`_Cntr8_H
    
#include "`$INSTANCE_NAME`_defs.h"
#define `$INSTANCE_NAME`_Period_Reg  `$INSTANCE_NAME`_Cntr8_D0_REG
#define `$INSTANCE_NAME`_Result_Reg  `$INSTANCE_NAME`_Cntr8_A0_REG
    
void `$INSTANCE_NAME`_WritePeriod(uint8 period);
uint8 `$INSTANCE_NAME`_ReadPeriod();
uint8 `$INSTANCE_NAME`_ReadCount();
    
#endif
/* [] END OF FILE */
