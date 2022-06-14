#include "leds.h"
#include "main.h"

void ledsSet(Leds_t led,Leds_state_t state)
{
    switch (led)
    {
    case LEDS_EMERGENSY:
        HAL_GPIO_WritePin(EMERGANSY_BREAK_INDICATOR_GPIO_Port,EMERGANSY_BREAK_INDICATOR_Pin,state);
        break;
    case LEDS_ROS:
        HAL_GPIO_WritePin(ROS_CONNECT_INDICATOR_GPIO_Port,ROS_CONNECT_INDICATOR_Pin,state);
        break;
    case LEDS_ENABLE:
        HAL_GPIO_WritePin(ENABLE_INDICATOR_GPIO_Port,ENABLE_INDICATOR_Pin,state);
        break;
    case LEDS_RIGHT:
        HAL_GPIO_WritePin(TURN_SIGNAL_RIGHT_GPIO_Port,TURN_SIGNAL_RIGHT_Pin,state);
        break;
    case LEDS_LEFT:
        HAL_GPIO_WritePin(TURN_SIGNAL_LEFT_GPIO_Port,TURN_SIGNAL_LEFT_Pin,state);
        break;
    default:
        break;
    }
}

void ledsToggle(Leds_t led)
{
   switch (led)
    {
    case LEDS_EMERGENSY:
        HAL_GPIO_TogglePin(EMERGANSY_BREAK_INDICATOR_GPIO_Port,EMERGANSY_BREAK_INDICATOR_Pin);
        break;
    case LEDS_ROS:
        HAL_GPIO_TogglePin(ROS_CONNECT_INDICATOR_GPIO_Port,ROS_CONNECT_INDICATOR_Pin);
        break;
    case LEDS_ENABLE:
        HAL_GPIO_TogglePin(ENABLE_INDICATOR_GPIO_Port,ENABLE_INDICATOR_Pin);
        break;
    case LEDS_RIGHT:
        HAL_GPIO_TogglePin(TURN_SIGNAL_RIGHT_GPIO_Port,TURN_SIGNAL_RIGHT_Pin);
        break;
    case LEDS_LEFT:
        HAL_GPIO_TogglePin(TURN_SIGNAL_LEFT_GPIO_Port,TURN_SIGNAL_LEFT_Pin);
        break;
    default:
        break;
    } 
}