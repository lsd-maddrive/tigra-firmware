#include "driveControl.h"

extern DAC_HandleTypeDef hdac;

float sign(float a){
	if (a>0) return 1;
	else if (a<0) return -1;
	else return 0;
}

/**
 * @brief   Carries out speed regulation.
 * @param   refSpeed - speed reference value.
 * @param   PID - structure with controller settings.
 */
void speedControlProcess(float refSpeed,PIDHandle_t* PID)
{
    //TODO - add reverse and brake control system.
    float controlImpact=PIDController(PID,getSpeed());
    HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)controlImpact);
}

/**
 * @brief   PID controller function.
 * @param   PID - structure with controller settings.
 * @param   error - current value of the following error.
 */
float PIDController(PIDHandle_t * PID,float error)
{
    float deltaError = error - PID->prevError;
    float controllerOut;
    PID->prevError=error;
    PID->integralTerm+=error;
    if(fabs(PID->integralTerm)>PID->integralSaturation)
        PID->integralTerm=sign(PID->integralTerm)*PID->integralSaturation;
    controllerOut=error*PID->kp+PID->integralTerm*PID->ki+deltaError*PID->kd;
    if(fabs(controllerOut)>PID->controllerSaturation)
        controllerOut=sign(controllerOut)*PID->controllerSaturation;
    return controllerOut;
}