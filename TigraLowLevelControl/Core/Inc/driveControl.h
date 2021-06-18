#ifndef __DRIVE_CONTROL_H
#define __DRIVE_CONTROL_H

#include "stm32f7xx_hal.h"
#include "speedMeasure.h"
#include "math.h"

#define SPEED_MAX_VALUE 500

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

void speedControlProcess(PIDHandle_t* PID);
float PIDController(PIDHandle_t * PID,float error);
void setReferenceSpeed(float speed);

#endif