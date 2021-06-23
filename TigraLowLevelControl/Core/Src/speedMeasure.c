#include "speedMeasure.h"


static uint16_t encoderCount;
static uint8_t statusFlag=0;
static uint16_t encoderPeriod;
static uint8_t direction;
extern TIM_HandleTypeDef htim4;

/**
 * @brief   Starts the process of measuring the speed from the encoder
 * @param   diveder - Sets the part of the period over which the speed is averaged
 */
void encoderStart(uint8_t diveder)
{
	TIM4->ARR=(ENCODER_STEP_COUNT/diveder)-1;
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
}

/**
 * @brief   Called from interrupt. Measuring the time of a part of an encoder revolution.
 */
void encoderMeasureDate(void)
{
	if(statusFlag==0)
	{
		TIM13->CNT=0;		
		TIM13->DIER|=TIM_DIER_UIE;
		TIM13->CR1|=TIM_CR1_CEN;
		statusFlag=1;
	}
	else
	{
		encoderPeriod=TIM13->CNT;
		if(TIM4->CR1 & TIM_CR1_DIR)
			direction=1;
		else 
			direction=0;
		statusFlag=0;
	}
}

/**
 * @brief   Called from interrupt. Sets zero speed and stops the measurement process.
 */
void encoderSetZeroSpeed(void)
{
	encoderPeriod=0;
	statusFlag=0;
	TIM13->CR1&=~TIM_CR1_CEN;
	TIM13->CNT=0;
}

/**
 * @brief	Returns the current speed.
 */
float getSpeed(void)
{
	float speed;
	if(encoderPeriod!=0)
	{
		speed=(float)encoderPeriod/1000000;
		speed=3.33/speed;
		if(direction==1)
			speed=-1*speed;
		return speed/SPEED_DIVIDER;
	}
	return 0;
}