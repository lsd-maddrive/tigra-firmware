#ifndef __TESTS_H
#define __TESTS_H

#define ENCODER_TEST 0
#define DRIVE_TEST 0
#define TEST_SPEED_CONRTOL_SYSTEM 0
#define BREAK_TEST 0
#define RUDDER_COMMUNICATION_TEST 0


#include "stm32f7xx_hal.h"
#include "speedMeasure.h"
#include "driveControl.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void testProcess(void);
void encoderTest(void);
void driveTest(void);
void breakTest(void);
void printDebugMessage(uint8_t *str);
void rudderCommunicationTest(void);

#endif
