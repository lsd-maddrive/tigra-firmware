#include "driveControl.h"

extern DAC_HandleTypeDef hdac;

void setDriveUnitSpeed(int16_t angularSpeed)
{
    HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,angularSpeed);
}
