#include "tests.h"

extern UART_HandleTypeDef huart3;
extern DAC_HandleTypeDef hdac;

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
 * @brief   Main test funcrion.
 */
void testProcess(void)
{
    static uint16_t counter;
#if ENCODER_TEST
    if(counter>50)
    {
        counter=0;
        encoderTest(); 
    } 
    else
        counter++;
#endif
#if DRIVE_TEST
    driveTest();
#endif
    osDelay(10);
}

/**
 * @brief   Encoder test function.
 */
void encoderTest(void)
{
    uint8_t string[100];
    float speed=getSpeed();
    if(speed<0)
    {
        speed*=-1;
        sprintf(string,"Speed:-%d.%03d\n\r",(uint32_t)speed, (uint16_t)((speed - (uint32_t)speed)*1000.) );
    }
    else
        sprintf(string,"Speed:%d.%03d\n\r",(uint32_t)speed, (uint16_t)((speed - (uint32_t)speed)*1000.) );
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
    uint8_t direction;
    HAL_StatusTypeDef reciveStatus;
    reciveStatus=HAL_UART_Receive(&huart3,&i,1,1);
    str[state]=i;
    if((str[state]=='F' || str[state]=='B') && state==0)
    {
        HAL_UART_Transmit(&huart3,"Print DAC value\n\r",17,100); 
        state++;
        if(str[state]=='B')
            direction=1;
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
                        setReferenceSpeed(DACValue);
                        HAL_UART_Transmit(&huart3,"Value set\n\r",15,100);  
                    }
#else
                    if(direction==1)
                        HAL_GPIO_WritePin(DRIVE_REVERSE_GPIO_Port,DRIVE_REVERSE_Pin,1);
                    else
                        HAL_GPIO_WritePin(DRIVE_REVERSE_GPIO_Port,DRIVE_REVERSE_Pin,0);
                    if(DACValue>4095)
                        HAL_UART_Transmit(&huart3,"Incorrect value\n\r",17,100); 
                    else
                    {
                        HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)DACValue);
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