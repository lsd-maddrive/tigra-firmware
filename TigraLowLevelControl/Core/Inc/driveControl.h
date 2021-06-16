#ifndef __DRIVE_CONTROL_H
#define __DRIVE_CONTROL_H

#include "stm32f7xx_hal.h"

typedef struct
{
    int16_t angularSpeed;//Reference angular speed
    int16_t angle;//Reference angle
}
driveData_t;

void setDriveUnitSpeed(int16_t angularSpeed);
 
#endif