#ifndef __DRIVE_CONTROL_H
#define __DRIVE_CONTROL_H

#include "stm32f7xx_hal.h"
#include "main.h"
#include "speedMeasure.h"
#include "math.h"

#define SPEED_MAX_VALUE 500
#define BREAK_REF_CURRENT 1

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

typedef enum 
{
    NO_BREAK,//No break state
    BREAK,//Break state
    BREAK_DROP//Return the break
}brakeStatus_t;


void speedControlProcess(void);
float PIDController(PIDHandle_t * PID,float error);
void setReferenceSpeed(float speed);
void breakControl(void);
void breakRealise(void);
void setBreakStatus(brakeStatus_t status);

#endif