#ifndef __SPEED_MEASURE_H
#define __SPEED_MEASURE_H

#include "stm32f7xx_hal.h"


#define ENCODER_STEP_COUNT 720//Encoder resolution
#define SPEED_DIVIDER 8.3

void encoderStart(uint8_t diveder);
void encoderMeasureDate(void);
void encoderSetZeroSpeed(void);
float getSpeed(void);

#endif