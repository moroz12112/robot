/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */


FIFO_One_str One_str;


Pos_Servo ServoA={
		.letter_Servo='A',
		.htim=SERVO_A_HTIM1,
		.pinEn = SERVO_A_CHANNEL1,
		.Min=0,
		.Max=105,
		.current=700,
		.new_angle=1000,
		.defta=1,
};

Pos_Servo ServoB={
		.letter_Servo='B',
		.htim=SERVO_A_HTIM1,
		.pinEn = SERVO_A_CHANNEL2,
		.Min=30,
		.Max=95,
		.current=700,
		.new_angle=1000,
		.defta=1,
};

Pos_Servo ServoC={
		.letter_Servo='C',
		.htim=SERVO_A_HTIM1,
		.pinEn = SERVO_A_CHANNEL3,
		.Min=20,
		.Max=80,
		.current=700,
		.new_angle=1000,
		.defta=1,
};

Pos_Servo ServoD={
		.letter_Servo='D',
		.htim=SERVO_A_HTIM4,
		.pinEn = SERVO_A_CHANNEL1,
		.Min=0,
		.Max=110,
		.current=520,
		.new_angle=1000,
		.defta=1,
};

Pos_Servo ServoE={

		.letter_Servo='E',
		.htim=SERVO_A_HTIM4,
		.pinEn = SERVO_A_CHANNEL2,
		.Min=10,
		.Max=140,
		.current=520,
		.new_angle=1000,
		.defta=1,
};

Pos_Servo ServoF={
		.letter_Servo='F',
		.htim=SERVO_A_HTIM4,
		.pinEn = SERVO_A_CHANNEL3,
		.Min=0,
		.Max=145,
		.current=520,
		.new_angle=1000,
		.defta=1,
};



FIFO_Buf buf = {
	.len = 100,
	.posRead = 0,
	.posWrite = 0,
	.Chek = 0,
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

uint8_t Buf_Read(FIFO_Buf *buf);
void Buf_Write(FIFO_Buf *buf, uint8_t data);
uint8_t Check_Buf(FIFO_Buf *buf);
uint8_t Parser_Buf(FIFO_Buf *buf);
uint8_t Servo_Control(Pos_Servo *Pos_Servo);
void setPWM(uint16_t value, Pos_Servo *Servo);
uint16_t Transfer_To_The_Range(uint8_t angel , uint8_t Min, uint8_t Max);
void Servo_Processing(Pos_Servo *Servo, uint8_t *v);


void Read_Buf_Str(FIFO_Buf *buf, FIFO_One_str *str);
//void Read_Comand(FIFO_Buf *buf);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // ServoA.htim, ServoA.pinEn);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	USART3 -> CR1 |= USART_CR1_RXNEIE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {


   //получение строки из кольцевого буфера

	if(buf.Chek>0){
		Read_Buf_Str(&buf,&One_str);
	}

	//парсер команды, запись команд в определенные структуры


	if(buf.Chek>0){
	  Parser_Buf(&buf);
	}

	//обработка сервисных команд


	//проверка наличия новых положения сервоприводов

	Servo_Control(&ServoA);
	Servo_Control(&ServoB);
	Servo_Control(&ServoC);
	Servo_Control(&ServoD);
	Servo_Control(&ServoE);
	Servo_Control(&ServoF);

	//проверка для верхнего уровня


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Servo_Processing(Pos_Servo *Servo, uint8_t *angle)
{
	 if(Servo->Max <= *angle){
		*angle=Servo->Max;
	}
	else if(Servo->Min >= *angle){
		*angle=Servo->Min;
	}
	Servo->new_angle=Transfer_To_The_Range(*angle , 0, 180);
	*angle=0;
}
uint8_t Parser_Buf(FIFO_Buf *buf){

	buf->Chek=0;
	uint8_t i = 0;
	uint8_t v = 0;
	while((buf->bufData[i-1] != '\r') && (buf->bufData[i] != '\n') && (i < buf->len)){

		switch (buf->bufData[i]) {

			 case '0':
			 case '1':
			 case '2':
			 case '3':
			 case '4':
			 case '5':
			 case '6':
			 case '7':
			 case '8':
			 case '9':
					 v = v * 10 + (buf->bufData[i] - '0');
				 break;
			 case 'A':
			 case 'a':
				 Servo_Processing(&ServoA, &v);
				 break;
			 case 'B':
			 case 'b':
				 Servo_Processing(&ServoB, &v);
				 break;
			 case 'C':
			 case 'c':
				 Servo_Processing(&ServoC, &v);
				break;
			 case 'D':
			 case 'd':
				 Servo_Processing(&ServoD, &v);
				break;
			 case 'E':
			 case 'e':
				 Servo_Processing(&ServoE, &v);
				break;
			 case 'F':
			 case 'f':
				 Servo_Processing(&ServoF, &v);
			 	break;
			 default:
				// HAL_UART_Transmit_IT(&huart3, &p, 1);
				 break;
		}

		i++;
	}

	//ReadBufStr(buf);

	return 0;
}

uint8_t Buf_Read(FIFO_Buf *buf){


	uint8_t type = buf->bufData[buf->posRead];
	buf->posRead += 1;

	if( buf->posRead >= buf->len ){
		buf->posRead=0;
	}

	return type;

}

void Buf_Write(FIFO_Buf *buf, uint8_t data){

	//uint8_t sss = '8';


	//buf->bufData[buf->posWrite] = data;
	//HAL_UART_Transmit_IT(&huart3, &sss, 1);

	buf->bufData[buf->posWrite] = data;
	if((buf->bufData[buf->posWrite] == '\n') && (buf->bufData[buf->posWrite-1] == '\r')){
		//HAL_UART_Transmit_IT(&huart3, &sss, 1);
		buf->Chek += 1;
	}


	buf->posWrite += 1;

	if( buf->posWrite >= buf->len ){
		buf->posWrite = 0;
	}


}

uint8_t Check_Buf(FIFO_Buf *buf){

	if(buf->posRead != buf->posWrite){
		return 1;
	}
	return 0;
}

void Read_Buf_Str(FIFO_Buf *buf,FIFO_One_str *str){

    uint8_t pos = 0;
	while( Check_Buf(buf) == 1)
		{
			//X = Buf_Read(buf);
			str->Buf[pos] = Buf_Read(buf);//while( HAL_UART_Transmit_IT(&huart3, &X, 1) == HAL_BUSY );
			pos++;
			// TODO обработка длины строки больше чем в str
		}

}

uint8_t Servo_Control(Pos_Servo *Servo){


	if(Servo->current != Servo->new_angle ){

	    		Servo->current+=Servo->defta;
	    		setPWM(Servo->current, Servo);
	    	}
	    	if(Servo->current != Servo->new_angle){

	    	  if(Servo->current < Servo->new_angle){

	    		Servo->current+=Servo->defta;
	    		if(Servo->current>Servo->new_angle){
	    			Servo->current=Servo->new_angle;
	    		}
	    		setPWM(Servo->current, Servo);
	    		HAL_Delay(10);
	    	    }

	    	   else{

	    		 Servo->current-=Servo->defta;
	    		 if(Servo->current<Servo->new_angle){
	    		     Servo->current=Servo->new_angle;
	    		 }
	    		 setPWM(Servo->current, Servo);
	    		 HAL_Delay(10);
	    	   }

	    	}

	  return 0;
}

void setPWM(uint16_t value, Pos_Servo *Servo)
{
	switch (Servo->pinEn) {

				 case TIM_CHANNEL_1:
					 Servo->htim->Instance->CCR1 = value;
					 break;
				 case TIM_CHANNEL_2:
					 Servo->htim->Instance->CCR2 = value;
				 	 break;
				 case TIM_CHANNEL_3:
					 Servo->htim->Instance->CCR3 = value;
				 	 break;
				 case TIM_CHANNEL_4:
					 Servo->htim->Instance->CCR4 = value;
				 	 break;

				 default:
					break;
	}

	////TIM1->CCR1 = value;
	//TIM1->CCR2 = value;
	//TIM1->CCR3 = value;
	//TIM1->CCR4 = value;

	//TIM4->CCR1 = value;
	//TIM4->CCR2 = value;
	//TIM4->CCR3 = value;
	//TIM4->CCR4 = value;
}

uint16_t Transfer_To_The_Range(uint8_t angle , uint8_t Min, uint8_t Max){

	 return 520 + ((2150 - 520) * (100 * (angle - Min) / (Max - Min))) / 100;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

