/**
  ******************************************************************************
  * @file    servo.h
  * @brief   This file contains constants parameters for control servo motor
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Driver Selection ----------------------------------------------------------*/
//#define LL_Driver

typedef struct {
#ifdef LL_Driver
	TIM_TypeDef* MasterTim;
	TIM_TypeDef* SlaveTim;
#else
	TIM_HandleTypeDef* MasterTim;
	TIM_HandleTypeDef* SlaveTim;
#endif
	uint32_t Channel;
	uint16_t AMin;
	uint16_t AMax;
	uint8_t ARng;
} servo_t;


/* Servo Motor External Function --------------------------------------------------*/
void ServoMotor_Init(servo_t *s);
void Servo_SetAngle(servo_t *s, uint8_t deg);
void Servo_Calibrate(servo_t *s, uint16_t ccr);

#ifdef __cplusplus
}
#endif

#endif
