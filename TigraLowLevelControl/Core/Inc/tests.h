#ifndef __TESTS_H
#define __TESTS_H

#define ENCODER_TEST 0
#define DRIVE_TEST 1

#include "stm32f7xx_hal.h"
#include "speedMeasure.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void testProcess(void);
void encoderTest(void);
void driveTest(void);

#endif
