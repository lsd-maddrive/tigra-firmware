#ifndef __DRIVE_CONTROL_H
#define __DRIVE_CONTROL_H

#include "stm32f7xx_hal.h"
#include "speedMeasure.h"
#include "math.h"

typedef struct
{
    int16_t angularSpeed;//Reference angular speed
    int16_t angle;//Reference angle
}
driveData_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float prevError;
    float integralTerm;
    float controllerSaturation;
    float integralSaturation;
}PIDHandle_t;

void speedControlProcess(float refSpeed,PIDHandle_t* PID);
float PIDController(PIDHandle_t * PID,float error);

#endif