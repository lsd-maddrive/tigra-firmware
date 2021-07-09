#include "driveControl.h"

static brakeStatus_t breakFlag=NO_BREAK;
static float refSpeed=0;
static int8_t currentAngle;
static float breakRefCurrent=BREAK_REF_CURRENT;
static uint8_t reverse=1;
uint8_t uartByte;

extern DAC_HandleTypeDef hdac;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim9;
extern UART_HandleTypeDef huart1;

PIDHandle_t SpeedPID=
  {
    .kp=1,
    .ki=0.1,
    .kd=0,
    .prevError=0,
    .integralTerm=0
  };

  PIDHandle_t breakCurrentPID=
  {
    .kp=10000,
    .ki=7000,
    .kd=800,
    .prevError=0,
    .integralTerm=0
  };

float sign(float a){
	if (a>0) return 1;
	else if (a<0) return -1;
	else return 1;
}

/**
 * @brief   Carries out speed regulation.
 */
void speedControlProcess(void)
{
    float controlImpact;
    if(breakFlag!=EMERGANSY_BRAKE)
    {
        if(refSpeed==0)
        {
            HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0); 
            HAL_GPIO_WritePin(DRIVE_REVERSE_GPIO_Port,DRIVE_REVERSE_Pin,1);
            SpeedPID.integralTerm=0;
            SpeedPID.prevError=0;
            osDelay(50);
        }
        if(breakFlag==NO_BREAK)
        {
            if(reverse==1)
            {
                if(refSpeed>=0)
                    HAL_GPIO_WritePin(DRIVE_REVERSE_GPIO_Port,DRIVE_REVERSE_Pin,1);
                else
                    HAL_GPIO_WritePin(DRIVE_REVERSE_GPIO_Port,DRIVE_REVERSE_Pin,0);
                SpeedPID.integralTerm=0;
                SpeedPID.prevError=0;
                reverse=0;
                osDelay(50);
            }
            controlImpact=PIDController(&SpeedPID,refSpeed-getSpeed());
            if(controlImpact<0 && refSpeed<0)
                controlImpact*=-1;   
            if(controlImpact<0)
                controlImpact=0;
            if(controlImpact!=0)
                controlImpact+=1500;
            HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)controlImpact);  
        }
        else
        {
            breakControl();
        }   
    }
    else
    {
        HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0); 
        HAL_GPIO_WritePin(EMERGANSY_BREAK_INDICATOR_GPIO_Port,EMERGANSY_BREAK_INDICATOR_Pin,0);
        HAL_GPIO_WritePin(ENABLE_INDICATOR_GPIO_Port,ENABLE_INDICATOR_Pin,1);
    }   
}

/**
 * @brief   Break control system.
 */
void breakControl(void)
{
    if(breakFlag==BREAK || breakFlag==BREAK_DROP)
    {
        if(breakFlag==BREAK)
            currentControl(breakRefCurrent);
        else if(breakFlag==BREAK_DROP)
            currentControl(-1*breakRefCurrent);
        if((getSpeed()==0) && breakFlag==BREAK)
        {
            if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0)==0)
            {
                breakFlag=NO_BREAK;  
                TIM9->CCR1=0;
                breakCurrentPID.integralTerm=0;
                breakCurrentPID.prevError=0;
                HAL_GPIO_WritePin(BREAK_DIRECTION_L_GPIO_Port,BREAK_DIRECTION_L_Pin,0);
                HAL_GPIO_WritePin(BREAK_DIRECTION_R_GPIO_Port,BREAK_DIRECTION_R_Pin,0);
            }
            else 
            {
                breakFlag=BREAK_DROP;
                breakCurrentPID.integralTerm=0;
                breakCurrentPID.prevError=0;
            }
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
    controllerOut=error*PID->kp+PID->integralTerm*PID->ki+deltaError*PID->kd;
    return controllerOut;
}

/**
 * @brief   Reference speed set function.
 * @param   speed - reference speed.
 */
void setReferenceSpeed(float speed)
{
    if(breakFlag==NO_BREAK && refSpeed!=speed)
    {
        if(getSpeed()!=0)
        {
            if(sign(speed)!=sign(getSpeed()) || speed==0)
            {
                breakFlag=BREAK;
                HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);
                SpeedPID.integralTerm=0;
                breakRefCurrent=BREAK_REF_CURRENT;
                if(sign(speed)!=sign(refSpeed))
                    reverse=1;
                refSpeed=speed;
            }
            else if(sign(speed)==sign(getSpeed()) && sign(speed)==sign(refSpeed))
            {
                refSpeed=speed;
            }
        }
        else
        {
            refSpeed=speed;
            if(refSpeed<0)
                reverse=1;
        }
    }
    else if ((breakFlag==BREAK || breakFlag==BREAK_DROP) && refSpeed!=speed)
    {
        if(sign(speed)!=sign(refSpeed))
            reverse=1;
        refSpeed=speed;
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

/**
 * @brief   Recive wheels angle from UART1.
 */
void reciveAngle(uint8_t byte)
{
    static uint8_t state=0;
    if(byte==0x1 && state==0)
    {
        state++;
    }
    else if(state==1)
    {
        currentAngle=(int8_t)byte;
        state=0;
    }
}

/**
 * @brief   get current wheels angle.
 */
int8_t getAngle(void)
{
    return currentAngle;
}

/**
 * @brief   send reference angle to rudder control system;
 */
void sendReferenceAngle(float refAngle)
{
    int8_t angle = (int8_t)refAngle;
    HAL_UART_Transmit(&huart1,&angle,sizeof(int8_t),100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart==&huart1)
    {
        reciveAngle(uartByte);
    }
}