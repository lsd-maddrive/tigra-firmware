#include "driveControl.h"

static brakeStatus_t breakFlag=NO_BREAK;
static float refSpeed;
static float breakRefCurrent=BREAK_REF_CURRENT;

extern DAC_HandleTypeDef hdac;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim9;

PIDHandle_t SpeedPID=
  {
    .kp=10,
    .ki=5,
    .kd=0,
    .integralSaturation=2000,
    .controllerSaturation=4095,
    .prevError=0,
    .integralTerm=0
  };

  PIDHandle_t breakCurrentPID=
  {
    .kp=10000,
    .ki=800,
    .kd=800,
    .integralSaturation=30000,
    .controllerSaturation=30000,
    .prevError=0,
    .integralTerm=0
  };

float sign(float a){
	if (a>0) return 1;
	else if (a<0) return -1;
	else return 0;
}

/**
 * @brief   Carries out speed regulation.
 */
void speedControlProcess(void)
{
    float controlImpact;
    if(refSpeed==0)
        HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0); 
    if(breakFlag==NO_BREAK)
    {
        controlImpact=PIDController(&SpeedPID,refSpeed-getSpeed());
        if(controlImpact<0)
            controlImpact=0;
        HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)controlImpact);  
    }
    else
    {
        breakControl();
    }   
}

/**
 * @brief   Break control system.
 */
void breakControl(void)
{

    if(breakFlag==BREAK || breakFlag==BREAK_DROP)
    {
        currentControl(breakRefCurrent);
        if((getSpeed()==0) && breakFlag==BREAK)
        {
            breakFlag=BREAK_DROP;
            breakRefCurrent*=-1;
        }
    }
}

/**
 * @brief   Current loop control system.
 * @param   refCurrent - referenceCurrentrefCurrent.
 */
void currentControl(float refCurrent)
{
    float controllBrakeDrive;
    float current;
    current=getBrakeCurrent();
    controllBrakeDrive=PIDController(&breakCurrentPID,(float)(refCurrent-(float)current));
    if(controllBrakeDrive>=0)
    {
        HAL_GPIO_WritePin(BREAK_DIRECTION_L_GPIO_Port,BREAK_DIRECTION_L_Pin,0);
        HAL_GPIO_WritePin(BREAK_DIRECTION_R_GPIO_Port,BREAK_DIRECTION_R_Pin,1);
        TIM9->CCR1=(uint16_t)controllBrakeDrive;
    }
    else if(controllBrakeDrive<0)
    {
        controllBrakeDrive=-1*controllBrakeDrive;
        HAL_GPIO_WritePin(BREAK_DIRECTION_L_GPIO_Port,BREAK_DIRECTION_L_Pin,1);
        HAL_GPIO_WritePin(BREAK_DIRECTION_R_GPIO_Port,BREAK_DIRECTION_R_Pin,0);
        TIM9->CCR1=(uint16_t)controllBrakeDrive;
    }
}

/**
 * @brief   release the brakes when the limit switch is triggered.
 */
void breakRealise(void)
{
    if(breakFlag==BREAK_DROP)
    {
        TIM9->CCR1=0;
        HAL_GPIO_WritePin(BREAK_DIRECTION_L_GPIO_Port,BREAK_DIRECTION_L_Pin,0);
        HAL_GPIO_WritePin(BREAK_DIRECTION_R_GPIO_Port,BREAK_DIRECTION_R_Pin,0);
        breakFlag=NO_BREAK;
        if(refSpeed>=0)
            HAL_GPIO_WritePin(DRIVE_REVERSE_GPIO_Port,DRIVE_REVERSE_Pin,0);
        else
            HAL_GPIO_WritePin(DRIVE_REVERSE_GPIO_Port,DRIVE_REVERSE_Pin,1);
    }
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
    /*if(fabs(PID->integralTerm)>PID->integralSaturation)
        PID->integralTerm=sign(PID->integralTerm)*PID->integralSaturation;*/
    controllerOut=error*PID->kp+PID->integralTerm*PID->ki+deltaError*PID->kd;
    /*if(fabs(controllerOut)>PID->controllerSaturation)
        controllerOut=sign(controllerOut)*PID->controllerSaturation;*/
    return controllerOut;
}

/**
 * @brief   Reference speed set function.
 * @param   speed - reference speed.
 */
void setReferenceSpeed(float speed)
{
    if(getSpeed()!=0)
    {
    if(sign(speed)!=getSpeed() || speed==0)
        breakFlag=BREAK;
        DRIVE_STOP;
        breakRefCurrent=BREAK_REF_CURRENT;
        refSpeed=speed;
    }
    else
    {
        refSpeed=speed;
        if(refSpeed>=0)
            HAL_GPIO_WritePin(DRIVE_REVERSE_GPIO_Port,DRIVE_REVERSE_Pin,0);
        else
            HAL_GPIO_WritePin(DRIVE_REVERSE_GPIO_Port,DRIVE_REVERSE_Pin,1);
    }
}

/**
 * @brief   Break status set function.
 * @param   status - set break status.
 */
void setBreakStatus(brakeStatus_t status)
{
    breakFlag=status;
}

/**
 * @brief   Break status get function.
 */
brakeStatus_t getBreakStatus(void)
{
    return breakFlag;
}

/**
 * @brief   Return the measurment current in Amp.
 */
float getBrakeCurrent(void)
{
    uint16_t current;
    uint8_t i;
    float currentAmp=0;   
    for(i=0;i<CURRENT_MEASURMENT_COUNT;i++)
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1,100);
        current=HAL_ADC_GetValue(&hadc1);
        currentAmp+=(float)((float)(current-CURRENS_SENSOR_OFFSET)*0.00081)/CURRENT_SENSOR_SENSITIVITY;
    }
    currentAmp/=CURRENT_MEASURMENT_COUNT;
    if(currentAmp<0.1 && currentAmp>-0.1)
        currentAmp=0;
    return -1*currentAmp;
}
