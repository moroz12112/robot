/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MY_DEFINE_H
#define __MY_DEFINE_H

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_A_HTIM  (( TIM_HandleTypeDef *) &htim1)
#define SERVO_A_CHANNEL TIM_CHANNEL_1

typedef struct __FIFO_Buf{
    uint8_t len;
	uint8_t bufData[100];
	uint8_t posRead;
	uint8_t posWrite;
	uint8_t Chek;
}FIFO_Buf;


typedef struct __FIFO_One_str{

	uint8_t Buf[38];
	uint8_t Chek;

}FIFO_One_str;

typedef struct __Pos_Servo{

	uint8_t letter_Servo;

	uint8_t Min;
	uint8_t Max;

    TIM_HandleTypeDef *htim;
    uint32_t pinEn;

	uint16_t current;
	uint16_t new_angle;

	uint8_t defta;

}Pos_Servo;



typedef struct __FIFO_Comand{

	uint8_t Buf_Angle[38];
	uint8_t Buf_Servo[38];
	uint8_t Chek;

}FIFO_Comand;

extern void Buf_Write(FIFO_Buf *buf, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __MY_DEFINE_H */
