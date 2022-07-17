#include "brakeControl.h"

static Brake_state_t brakeState = BRAKE_STOP;

void brakeSetState(Brake_state_t direction,uint16_t power)
{
    if(direction==BRAKE_FORWARD)
    {
        brakeState=BRAKE_FORWARD;
        TIM9->CR1|=TIM_CR1_CEN;
        TIM9->CCR1=power;
        HAL_GPIO_WritePin(BREAK_DIRECTION_L_GPIO_Port,BREAK_DIRECTION_L_Pin,0);
        HAL_GPIO_WritePin(BREAK_DIRECTION_R_GPIO_Port,BREAK_DIRECTION_R_Pin,1);
    }
    else if(direction==BRAKE_REALISE)
    {
        brakeState=BRAKE_REALISE;
        // if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0)!=0)
        // {
            TIM9->CCR1=power;
            TIM9->CR1|=TIM_CR1_CEN;
            HAL_GPIO_WritePin(BREAK_DIRECTION_L_GPIO_Port,BREAK_DIRECTION_L_Pin,1);
            HAL_GPIO_WritePin(BREAK_DIRECTION_R_GPIO_Port,BREAK_DIRECTION_R_Pin,0);
        // }
    }
    else if(direction==BRAKE_STOP)
    {
        brakeState=BRAKE_STOP;
        TIM9->CR1&=~TIM_CR1_CEN;
        TIM9->CCR1=0;
        HAL_GPIO_WritePin(BREAK_DIRECTION_L_GPIO_Port,BREAK_DIRECTION_L_Pin,0);
        HAL_GPIO_WritePin(BREAK_DIRECTION_R_GPIO_Port,BREAK_DIRECTION_R_Pin,0);   
    }
}

void brakeRealise(void)
{
    if(brakeState==BRAKE_REALISE)
        brakeSetState(BRAKE_STOP,0);
}