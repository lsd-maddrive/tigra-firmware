#ifndef __SPEED_MEASURE_H
#define __SPEED_MEASURE_H

#include "stm32f7xx_hal.h"


#define ENCODER_STEP_COUNT 720//Encoder resolution
#define ALPHA 0.2
#define ALPHA_FILTER_PERIOD 40

void encoderStart(uint8_t diveder);
void encoderMeasureDate(void);
void encoderSetZeroSpeed(void);
float getSpeed(void);
void alphaFilter(void);
float getFilteredSpeed(void);

#endif