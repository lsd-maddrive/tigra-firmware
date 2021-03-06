#include "driveControl.h"

Drive_state_t driveState = STOP;

static brakeStatus_t breakFlag=NO_BREAK;
static float refSpeed=0;
static int8_t currentAngle;
static float breakRefCurrent=BREAK_REF_CURRENT;
static uint8_t reverse=1;
static uint8_t reverseState=0;
uint8_t uartByte;

extern DAC_HandleTypeDef hdac;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim9;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

PIDHandle_t SpeedPID=
  {
    .kp=30,//1.5
    .ki=0.15,//0.03
    .kd=0,
    .prevError=0,
    .integralTerm=0
  };


float sign(float a){
	if (a>0) return 1;
	else if (a<0) return -1;
	else return 1;
}

#define EMERGENCY_CHECK_MAX_COUNT 3
int emergencyBreakCheckCounter = 0;
int previousBreakFlag;

void startEmergencyCheck() 
{
    ledsSet(LEDS_EMERGENSY,LEDS_ON);
}

bool isEmergencyPressed() {
    return (GPIOF->IDR & 0x01) != 1;
}

/**
 * @brief   Carries out speed regulation.
 */
void speedControlProcess(void)
{
    float controlImpact;
    float currentSpeed = getSpeed();
    switch(driveState)
    {
        case STOP:
            brakeSetState(BRAKE_FORWARD,BRAKE_POWER);
            setMotorDirection(MOTOR_DIRECTION_FORWARD);
            setMotorPower(0);
            if(currentSpeed>=0 && refSpeed>0)
            {
                osDelay(50);
                driveChangeState(RUN);
            }
            else if(currentSpeed<=0 && refSpeed<0)
            {
                osDelay(500);
                driveChangeState(REVERS);
            }
            break;
        case RUN:
            if(refSpeed<=0)
            {
                driveChangeState(STOP);
            }
            setMotorDirection(MOTOR_DIRECTION_FORWARD);
            brakeSetState(BRAKE_REALISE,BRAKE_POWER);
            controlImpact=PIDController(&SpeedPID,refSpeed-getFilteredSpeed());
            if(controlImpact<0)
                controlImpact=0;
            else
                controlImpact+=MOTOR_CONSTANT_OFFSET;
            setMotorPower((uint16_t)controlImpact);
            break;
        case REVERS:
            if(refSpeed>=0)
            {
                driveChangeState(STOP);
            }
            setMotorDirection(MOTOR_DIRECTION_BACKWARD);
            brakeSetState(BRAKE_REALISE,BRAKE_POWER);
            controlImpact=PIDController(&SpeedPID,refSpeed-currentSpeed);
            if(controlImpact>0)
                controlImpact=0;
            if(controlImpact<0)
                controlImpact*=-1;
            controlImpact+=MOTOR_CONSTANT_OFFSET;
            setMotorPower((uint16_t)controlImpact);
            break;
        case FAIL:
            break;
    }
}

void driveChangeState(Drive_state_t state)
{
    uint8_t string[100];
    if(driveState==STOP && state==RUN)
    {
        driveState=RUN;
        sprintf(string,"STOP->RUN\n\r");      
    }
    else if(driveState==RUN && state==STOP)
    {
        PIDClear(&SpeedPID);
        driveState=STOP;
        sprintf(string,"RUN->STOP\n\r");  
    }
    else if(driveState==STOP && state==REVERS)
    {
        driveState=REVERS;
        sprintf(string,"STOP->REVERS\n\r");  
    }
    else if(driveState==REVERS && state==STOP)
    {
        PIDClear(&SpeedPID);
        driveState=STOP;
        sprintf(string,"REVERS->STOP\n\r");  
    }
    else
    {
        // sprintf(string,"FAIL CHANGE\n\r");  
    }
    HAL_UART_Transmit(&huart3,&string,strlen(string),100);
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

void PIDClear(PIDHandle_t * PID)
{
    PID->integralTerm=0;
    PID->prevError=0;
}

/**
 * @brief   Reference speed set function.
 * @param   speed - reference speed.
 */
void setReferenceSpeed(float speed)
{
    refSpeed=speed;
}

inline void setMotorPower(uint16_t power)
{
    uint8_t string[30];
    // float speed = getSpeed();
    // sprintf(string,"DAC:%d, Speed:%d\n\r",power,(uint16_t)speed);  
    // HAL_UART_Transmit(&huart3,&string,strlen(string),100);
    HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,power); 
}

void setMotorDirection(Motor_direction_t direction)
{
    if(direction==MOTOR_DIRECTION_FORWARD)
    {
        HAL_GPIO_WritePin(DRIVE_REVERSE_GPIO_Port,DRIVE_REVERSE_Pin,1);
    }
    else if(direction==MOTOR_DIRECTION_BACKWARD)
    {
        HAL_GPIO_WritePin(DRIVE_REVERSE_GPIO_Port,DRIVE_REVERSE_Pin,0);
    }
}

//TODO ?????????????????? ?? ?????????????????? ????????

/**
 * @brief   Recive wheels angle from UART1.
 */
void reciveAngle(uint8_t byte)
{
    currentAngle=(int8_t)byte;
    //static uint8_t state=0;
    /*if(byte==0x1 && state==0)
    {
        state++;
    }
    else if(state==1)
    {
        currentAngle=(int8_t)byte;
        state=0;
    }*/
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