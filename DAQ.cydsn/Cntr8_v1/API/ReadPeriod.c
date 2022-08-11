/* ========================================
 *
 * Copyright UCSC, 2022
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <cydevice_trm.h>
#include <CyLib.h>

#include "`$INSTANCE_NAME`_Cntr8.h"

uint8 `$INSTANCE_NAME`_ReadPeriod() {
    return 255 - `$INSTANCE_NAME`_Period_Reg;
    }
/* [] END OF FILE */
