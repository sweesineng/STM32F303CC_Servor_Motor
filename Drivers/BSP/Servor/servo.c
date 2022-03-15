/**
  ******************************************************************************
  * @file    servor.c
  * @brief   This file includes the HAL/LL driver for control Servo Motor
  * @note	 Servo motor usually operate in 5V or higher mode. STM32 ADC
  * 		 operate in 3.3V. This library possible to use cascade timer to
  * 		 increase the precision.
  ******************************************************************************
  */
#include "servo.h"

#ifdef LL_Driver
void Start_Timer(TIM_TypeDef *TIMx, uint32_t Channels)
{
	if (!LL_TIM_IsEnabledCounter(TIMx))
	{
		  /* Start Update Timer */
		LL_TIM_CC_EnableChannel(TIMx, Channels);
		LL_TIM_EnableCounter(TIMx);
		LL_TIM_EnableAllOutputs(TIMx);
	}
}
#endif

void ServoMotor_Init(servo_t *s)
{
	s->ARng = (s->ARng) ? s->ARng : 180;

#ifdef LL_Driver
	uint16_t ARR = LL_TIM_GetAutoReload(s->SlaveTim);
#else
	uint16_t ARR = __HAL_TIM_GET_AUTORELOAD(s->SlaveTim) + 1;
#endif

	if (s->MasterTim)
	{
		s->AMin = (s->AMin != ' ') ? s->AMin : (ARR / 2);
		s->AMax = (s->AMax != ' ') ? s->AMax : ARR;
#ifdef LL_Driver
		Start_Timer(s->MasterTim, s->Channel);
#else
		HAL_TIM_Base_Start(s->MasterTim);
#endif
	}else{
		s->AMin = (s->AMin != ' ') ? s->AMin : (ARR / 20);
		s->AMax = (s->AMax != ' ') ? s->AMax : (ARR / 10);
	}
#ifdef LL_Driver
	Start_Timer(s->SlaveTim, s->Channel);
#else
	HAL_TIM_PWM_Start(s->SlaveTim, s->Channel);
#endif
}

void Servo_SetAngle(servo_t *s, uint8_t deg)
{
	deg = (deg > s->ARng) ? s->ARng : deg;
	deg = (deg < 0) ? 0 : deg;
	uint16_t diff = s->AMax - s->AMin;
	uint16_t Duty = ((deg * diff) / s->ARng) + s->AMin;
	Duty = (Duty < 1) ? 1 : Duty;

#ifdef LL_Driver

	if (!LL_TIM_CC_IsEnabledChannel(s->SlaveTim, s->Channel))
	{
		LL_TIM_CC_EnableChannel(s->SlaveTim, s->Channel);
	}
	LL_TIM_OC_SetCompareCH1(s->SlaveTim, s->Duty);	// Replace if using other channel

#else

	__HAL_TIM_SET_COMPARE(s->SlaveTim, s->Channel, Duty);

#endif

}

void Servo_Calibrate(servo_t *s, uint16_t ccr)
{
#ifdef LL_Driver
	if (!LL_TIM_CC_IsEnabledChannel(s->SlaveTim, s->Channel))
	{
		LL_TIM_CC_EnableChannel(s->SlaveTim, s->Channel);
	}
	LL_TIM_OC_SetCompareCH1(s->SlaveTim, s->Duty);	// Replace if using other channel
#else
	__HAL_TIM_SET_COMPARE(s->SlaveTim, s->Channel, ccr);
#endif
}
