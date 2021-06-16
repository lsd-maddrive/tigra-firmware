#ifndef __SPEED_MEASURE_H
#define __SPEED_MEASURE_H

#include "stm32f7xx_hal.h"

#define ENCODER_STEP_COUNT 720

void encoderInit(void);
int32_t getSpeed(void);

#endif