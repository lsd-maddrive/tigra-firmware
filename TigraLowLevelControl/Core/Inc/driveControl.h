#ifndef __DRIVE_CONTROL_H
#define __DRIVE_CONTROL_H

#include "stm32f7xx_hal.h"
#include "main.h"
#include "speedMeasure.h"
#include "math.h"
#include "brakeControl.h"
#include <stdbool.h>

#define SPEED_MAX_VALUE 500
#define BREAK_REF_CURRENT 1
#define MOTOR_CONSTANT_OFFSET 0

#define CURRENT_SENSOR_SENSITIVITY 0.21
#define CURRENS_SENSOR_OFFSET 2048
#define CURRENT_MEASURMENT_COUNT 10

#define RESET_MODE 0 //0 - без режима ПАУЗА 1 - с режимом ПАУЗА

typedef struct
{
    int16_t angularSpeed;//Reference angular speed
    int16_t angle;//Reference angle
}
driveData_t;

typedef enum
{
    STOP,
    RUN,
    REVERS,
    FAIL
}Drive_state_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float prevError;
    float integralTerm;
}PIDHandle_t;

typedef enum 
{
    NO_BREAK,//No break state
    BREAK,//Break state
    BREAK_DROP,//Return the break
    EMERGANSY_BRAKE,
    EMERGANSY_BRAKE_CHECK
}brakeStatus_t;

typedef enum
{
    MOTOR_DIRECTION_FORWARD,
    MOTOR_DIRECTION_BACKWARD
}Motor_direction_t;

void speedControlProcess(void);
float PIDController(PIDHandle_t * PID,float error);
void PIDClear(PIDHandle_t * PID);
void setReferenceSpeed(float speed);
inline void setMotorPower(uint16_t power);
void setMotorDirection(Motor_direction_t direction);
bool isEmergencyPressed(); // Really not pressed but button released
void startEmergencyCheck();
void reciveAngle(uint8_t byte);
int8_t getAngle(void);
void sendReferenceAngle(float refAngle);

#endif