#ifndef __BRAKE_CONTROL_H
#define __BRAKE_CONTROL_H

#include "stm32f7xx_hal.h"
#include "main.h"
#include "speedMeasure.h"
#include "driveControl.h"

#define BRAKE_POWER 30000

typedef enum
{
    BRAKE_FORWARD,
    BRAKE_REALISE,
    BRAKE_STOP
}Brake_state_t;

/**
 * @brief   Set brake direction function.
 * @param   direction - brake direction
 * @param   power - pwm duty.
 */
void brakeSetState(Brake_state_t direction,uint16_t power);

/**
 * @brief   release the brakes when the limit switch is triggered.
 */
void brakeRealise(void);

Brake_state_t getBrakeState();
#endif