#include "tests.h"

extern UART_HandleTypeDef huart3;
extern DAC_HandleTypeDef hdac;
extern ADC_HandleTypeDef hadc1;

extern PIDHandle_t breakCurrentPID;
extern UART_HandleTypeDef huart1;

/**
 * @brief   Print debug message to UART3
 */
uint16_t powINT(uint16_t num,uint16_t pow)
{
  uint16_t i,rez=1;
  for(i=1;i<=pow;i++)
  {
    rez*=num;
  }
  return rez;
}

/**
 * @brief   Print debug message to UART3.
 * @param   str - pointer to message.
 */
void printDebugMessage(uint8_t *str)
{
    HAL_UART_Transmit(&huart3,str,strlen(str),100);
}

/**
 * @brief   Main test funcrion.
 */
void testProcess(void)
{
    static uint16_t counter1;
    static uint16_t counter2;
    uint8_t string[20];
#if ENCODER_TEST
    if(counter1>50)
    {
        counter1=0;
        encoderTest(); 
    } 
    else
        counter1++;
#endif
#if DRIVE_TEST && !BREAK_TEST && !RUDDER_COMMUNICATION_TEST
    driveTest();
#endif
#if BREAK_TEST && !DRIVE_TEST && !RUDDER_COMMUNICATION_TEST
    breakTest(); 
#endif
#if RUDDER_COMMUNICATION_TEST && !DRIVE_TEST && !BREAK_TEST
    rudderCommunicationTest(); 
    if(counter2>50)
    {
        counter2=0;
        sprintf(string,"Current angle:%d\n\r",getAngle());
        HAL_UART_Transmit(&huart3,&string,strlen(string),100);
    } 
    else
        counter2++;
#endif
    osDelay(10);
}

/**
 * @brief   Encoder test function.
 */
extern uint16_t countInt;

void encoderTest(void)
{
    uint8_t string[100];
    float speed=getSpeed();
    /*if(speed<0)
    {
        speed*=-1;
        sprintf(string,"Speed:-%d.%03d\n\r",(uint32_t)speed, (uint16_t)((speed - (uint32_t)speed)*1000.) );
    }
    else
        sprintf(string,"Speed:%d.%03d\n\r",(uint32_t)speed, (uint16_t)((speed - (uint32_t)speed)*1000.) );*/
    sprintf(string,"count:%d\n\r",countInt);
    HAL_UART_Transmit(&huart3,&string,strlen(string),100);
}

/**
 * @brief   Main drive test function.
 */

void driveTest(void)
{
    uint8_t i;
    uint16_t j;
    static uint8_t state=0;
    static uint8_t str[10];
    uint32_t DACValue=0;
    static uint8_t dir;
    HAL_StatusTypeDef reciveStatus;
    reciveStatus=HAL_UART_Receive(&huart3,&i,1,1);
    str[state]=i;
    if((str[state]=='F' || str[state]=='B') && state==0)
    {
        HAL_UART_Transmit(&huart3,"Print DAC value\n\r",17,100); 
        if(str[state]=='B')
        {
            dir=1;
        }
        if(str[state]=='F')
        {
            dir=0;
        }
        state++;
    }
    else
    {
        if(state>0 && reciveStatus==HAL_OK)
        {
            state++;
            if((str[state-1]>=0x30 && str[state-1]<=0x39) || str[state-1]=='\r' || str[state-1]=='\n')
            {
                if(str[state-1]=='\r')
                {
                    for(j=state-1;j>1;j--)
                    {
                        DACValue+=(str[j-1]-48)*powINT(10,state-1-j);
                    }
                    state=0;
#if TEST_SPEED_CONRTOL_SYSTEM
                    if(DACValue>SPEED_MAX_VALUE) 
                    { 
                        HAL_UART_Transmit(&huart3,"Incorrect value\n\r",17,100);   
                    }
                    else
                    {   
                        if(dir==1)
                            setReferenceSpeed((float)-1*DACValue);
                        else   
                            setReferenceSpeed(DACValue);
                        HAL_UART_Transmit(&huart3,"Value set\n\r",15,100);  
                    }
#else
                    if(dir==1)
                        HAL_GPIO_WritePin(DRIVE_REVERSE_GPIO_Port,DRIVE_REVERSE_Pin,0);
                    else
                        HAL_GPIO_WritePin(DRIVE_REVERSE_GPIO_Port,DRIVE_REVERSE_Pin,1);
                    if(DACValue>4095)
                        HAL_UART_Transmit(&huart3,"Incorrect value\n\r",17,100); 
                    else
                    {
                        HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)DACValue);
                        HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
                        HAL_UART_Transmit(&huart3,"Value set\n\r",15,100); 
                    }
#endif
                }
            }
            else
            {
                state=0;
                HAL_UART_Transmit(&huart3,"Incorrect symbol\n\r",18,100); 
            }
        }
    }

}

void breakTest()
{
    uint8_t symb;
    static uint16_t counter=0;
    float currentAmp;
    uint8_t string[100];
    HAL_UART_Receive(&huart3,&symb,1,1);
    if(symb=='B')
    {
        HAL_UART_Transmit(&huart3,"Break\n\r",7,100);
        setBreakStatus(BREAK);
    }
    if(symb=='R')
    {
        if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0)==1)
        {
            HAL_UART_Transmit(&huart3,"Break realise\n\r",15,100);
            setBreakStatus(BREAK_DROP);
        }
    }
    if(symb=='S')
    {
        HAL_UART_Transmit(&huart3,"Break stop\n\r",12,100);
        TIM9->CCR1=0;
        setBreakStatus(NO_BREAK);
    } 
    else
        counter++;
    if(getBreakStatus()==BREAK)
        currentControl(BREAK_REF_CURRENT);
    else if (getBreakStatus()==BREAK_DROP)
        currentControl(-1*BREAK_REF_CURRENT);
}

void rudderCommunicationTest(void)
{
    uint8_t i;
    uint16_t j;
    static uint8_t state=0;
    static uint8_t str[10];
    uint8_t rotateAngle=0;
    int8_t sendAngle;
    static uint8_t dir;
    HAL_StatusTypeDef reciveStatus;
    reciveStatus=HAL_UART_Receive(&huart3,&i,1,1);
    str[state]=i;  
    if((str[state]=='+' || str[state]=='-') && state==0)
    {
        HAL_UART_Transmit(&huart3,"Print rotate angle\n\r",17,100); 
        if(str[state]=='-')
        {
            dir=1;
        }
        if(str[state]=='+')
        {
            dir=0;
        }
        state++;
    }  
    else
    {
    if(state>0 && reciveStatus==HAL_OK)
    {
        state++;
        if((str[state-1]>=0x30 && str[state-1]<=0x39) || str[state-1]=='\r' || str[state-1]=='\n')
        {
            if(str[state-1]=='\r')
            {
                for(j=state-1;j>1;j--)
                {
                    rotateAngle+=(str[j-1]-48)*powINT(10,state-1-j);
                }
                state=0;
                if(rotateAngle>127)
                    HAL_UART_Transmit(&huart3,"Incorrect value\n\r",17,100); 
                else
                {
                    sendAngle=(int8_t)rotateAngle;
                    if(dir==1)
                        sendAngle*=-1;
                    HAL_UART_Transmit(&huart1,&sendAngle,sizeof(int8_t),100);
                    HAL_UART_Transmit(&huart3,"Value set\n\r",15,100); 
                }
            }
        }
        else
        {
            state=0;
            HAL_UART_Transmit(&huart3,"Incorrect symbol\n\r",18,100); 
        }
    }
}
}