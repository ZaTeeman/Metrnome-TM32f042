/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#define PAGE_ADRESS 0x08007C00
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{	
	uint16_t Flag; 		// first start flag
	char number[2];		// = {"01"};
	uint16_t size;		// = 2;
	uint16_t speed;		// = 200;
	char Name[16];// = {"NAME !"}			
	} N;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint16_t SpeedBPM = 200;
int8_t	NumberPage = 0;
volatile uint8_t FlagButt2 = 0, FlagButt1 = 0;
							// 2 == right button	1 == left button
uint8_t FlagButton2 = 0, FlagButton1 = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
union FlashMem
	{
		N page[20];
		uint32_t FlashMass[120];  
	}ForFlash;
	
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
void BPMmaster (int Bpm) 
{
	uint32_t TimeBTic = (1000*60)/Bpm;
	static uint32_t speed =0;
	static uint8_t BeebOn = 0, SetSettings = 0;
	static uint32_t pic, SizeCounter = 0;
	const uint32_t PicTime = 50;
	static uint16_t LedPin = GPIO_PIN_1; 
	if (SizeCounter > ForFlash.page[NumberPage].size)SizeCounter = 0;
	
	if (SizeCounter != ForFlash.page[NumberPage].size  && BeebOn == 0 && SetSettings == 0) 
		{
			//SizeCounter = 0;
			TIM2->CCR4 = 2000;	// x2  = 8192;
			htim2.Init.Period = 4000;
			LedPin = GPIO_PIN_1;
			SetSettings = 1;
			
		}
		if(SizeCounter == ForFlash.page[NumberPage].size  && BeebOn == 0 && SetSettings == 0)
		{
			TIM2->CCR4 = 8000;	// x2  = 8192;
			htim2.Init.Period = 16000;
			SizeCounter = 0;
			LedPin = GPIO_PIN_0;
			SetSettings = 1;
		}
		
	if (speed < HAL_GetTick())
		{
			speed = TimeBTic + HAL_GetTick();
			pic = HAL_GetTick() + PicTime;
			HAL_GPIO_WritePin (GPIOA, LedPin, GPIO_PIN_SET);   //	beeb
			HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_4);
			BeebOn = 1;
			SizeCounter++;
			SetSettings = 0;
		}

	if (HAL_GetTick() > pic)
		{
			//HAL_GPIO_WritePin (GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (GPIOA, LedPin, GPIO_PIN_RESET);
			HAL_TIM_PWM_Stop (&htim2, TIM_CHANNEL_4);
			
			BeebOn = 0;
		}
}


////////////////////////////////////
  // functions Button Handling
uint8_t ButtonHendler22 (uint8_t  FlagButton22)
{
	static uint16_t FirstCounter22 = 0, SecondCounter22 = 0; //65535
	uint8_t ButtonSatus22 = 0;
	static uint8_t NotUnpush22;

	// Noize Protections section
	if (FlagButton22 == 1) FirstCounter22++;
	if (FlagButton22 == 0) FirstCounter22--;

	if (FirstCounter22 > 20)
		{
			ButtonSatus22 = 1;
			FirstCounter22 = 20;
		}
	if (FirstCounter22 < 1)
		{
			ButtonSatus22 = 0;
			FirstCounter22 = 1;
		}		

	if (NotUnpush22 == 1 && ButtonSatus22 == 0) NotUnpush22 = 0;
	
	// Handling delay presure 	
	if (NotUnpush22 == 0)
	{
		if (ButtonSatus22 == 1) SecondCounter22++;

		if (SecondCounter22 > 300 && ButtonSatus22 == 1)
			{ 
				SecondCounter22 = 0;
				NotUnpush22 = 1;
				return 2; // long push
			}
		else if (SecondCounter22 > 0 && ButtonSatus22 != 1)
			{
				SecondCounter22 = 0;
				NotUnpush22 = 1;
				return 1; // short push
			} 
	}
	return 0;
}

////////////////////////////////////
uint8_t ButtonHendler11 (uint8_t  FlagButton11)
{
	static uint16_t FirstCounter11 = 0, SecondCounter11 = 0; //65535
	uint8_t ButtonSatus11 = 0;
	static uint8_t NotUnpush11;

	// Noize Protections section
	if (FlagButton11 == 1) FirstCounter11++;
	if (FlagButton11 == 0) FirstCounter11--;

	if (FirstCounter11 > 20)
		{
			ButtonSatus11 = 1;
			FirstCounter11 = 20;
		}
	if (FirstCounter11 < 1)
		{
			ButtonSatus11 = 0;
			FirstCounter11 = 1;
		}		

	if (NotUnpush11 == 1 && ButtonSatus11 == 0) NotUnpush11 = 0;
	
	// Handling delay presure 	
	if (NotUnpush11 == 0)
	{
		if (ButtonSatus11 == 1) SecondCounter11++;

		if (SecondCounter11 > 300 && ButtonSatus11 == 1)
			{ 
				SecondCounter11 = 0;
				NotUnpush11 = 1;
				return 2; // long push
			}
		else if (SecondCounter11 > 0 && ButtonSatus11 != 1)
			{
				SecondCounter11 = 0;
				NotUnpush11 = 1;
				return 1; // short push
			} 
	}
	return 0;
}
	
////////////////////////////////////	
uint8_t ButtonHendler1 (uint8_t  FlagButton)
{
	static uint16_t FirstCounter1 = 0, SecondCounter1 = 0; //65535
	uint8_t ButtonSatus1 = 0;
	static uint8_t NotUnpush1;

	// Noize Protections section
	if (FlagButton == 1) FirstCounter1++;
	if (FlagButton == 0) FirstCounter1--;

	if (FirstCounter1 > 5)
		{
			ButtonSatus1 = 1;
			FirstCounter1 = 5;
		}
	if (FirstCounter1 < 1)
		{
			ButtonSatus1 = 0;
			FirstCounter1 = 1;
		}		

	if (NotUnpush1 == 1 && ButtonSatus1 == 0) NotUnpush1 = 0;
	
	// Handling delay presure 	
	if (NotUnpush1 == 0)
	{
		if (ButtonSatus1 == 1) SecondCounter1++;

		if (SecondCounter1 > 30 && ButtonSatus1 == 1)
			{ 
				SecondCounter1 = 0;
				NotUnpush1 = 1;
				return 2; // long push
			}
		else if (SecondCounter1 > 0 && ButtonSatus1 != 1)
			{
				SecondCounter1 = 0;
				NotUnpush1 = 1;
				return 1; // short push
			} 
	}
	return 0;
}
////////////////////////////////////

uint8_t ButtonHendler2 (uint8_t  FlagButton)
{
	static uint16_t FirstCounter = 0, SecondCounter = 0; //65535
	uint8_t ButtonSatus = 0;
	static uint8_t NotUnpush;

	// Noize Protections section
	if (FlagButton == 1) FirstCounter++;
	if (FlagButton == 0) FirstCounter--;

	if (FirstCounter > 5)
		{
			ButtonSatus = 1;
			FirstCounter = 5;
		}
	if (FirstCounter < 1)
		{
			ButtonSatus = 0;
			FirstCounter = 1;
		}		

	if (NotUnpush == 1 && ButtonSatus == 0) NotUnpush = 0;
	
	// Handling delay presure 	
	if (NotUnpush == 0)
	{
		if (ButtonSatus == 1) SecondCounter++;

		if (SecondCounter > 30 && ButtonSatus == 1)
			{ 
				SecondCounter = 0;
				NotUnpush = 1;
				return 2; // long push
			}
		else if (SecondCounter > 0 && ButtonSatus != 1)
			{
				SecondCounter = 0;
				NotUnpush = 1;
				return 1; // short push
			} 
	}
	return 0;
}
////////////////////////////////////



void ProgramMode (uint8_t Flagbutton1,uint8_t Flagbutton2)
{
	uint8_t MassPosition[17][2] ={{5,0},{0,1},{1,1},{2,1},{3,1},{4,1},{5,1},{6,1},{7,1},
																{8,1},{9,1},{10,1},{11,1},{12,1},{13,1},{14,1},{15,1}};
	static int8_t CursorPosition = 0, OldCursorPosition = 0;
																
	// Set cursor location
	if (Flagbutton2 == 2) CursorPosition++; 
	if (Flagbutton1 == 2) CursorPosition--;
	//set rnge
	if (CursorPosition > 17) CursorPosition = 0;
  if (CursorPosition < 0) CursorPosition = 17;														 

  // Set settings Size
	if (CursorPosition == 0)
	{
		if (Flagbutton2 == 1) ForFlash.page[NumberPage].size = ForFlash.page[NumberPage].size+1;
		if (Flagbutton1 == 1) ForFlash.page[NumberPage].size = ForFlash.page[NumberPage].size-1;
		//set rnge
		if (ForFlash.page[NumberPage].size < 2) ForFlash.page[NumberPage].size = 4;
		if (ForFlash.page[NumberPage].size > 4) ForFlash.page[NumberPage].size = 2;
	}
	
	// Set settings Name
	if (CursorPosition > 0)
	{
		char CharOnDisplay = ForFlash.page[NumberPage].Name[CursorPosition - 1];
		if (FlagButton2 == 1)CharOnDisplay++;
		if (FlagButton1 == 1)CharOnDisplay--;
		if (CharOnDisplay > 95) CharOnDisplay = 65;
		if (CharOnDisplay < 65) CharOnDisplay = 95;
		ForFlash.page[NumberPage].Name[CursorPosition - 1] = CharOnDisplay;
	}	
			//function show position curosr
	LCD_SetPos (MassPosition[CursorPosition][0], MassPosition[CursorPosition][1]);
	static uint32_t pic;
	const uint32_t PicTime = 500;
	
	if (pic < HAL_GetTick())
		{
			pic = HAL_GetTick() + PicTime;
			static uint8_t toggle = 0;
			if (toggle == 0) 
			{
				OldCursorPosition = CursorPosition;
				LCD_SendChar(32);
				toggle = 1;
				pic -= 300;
			}
			else
			{
				if (CursorPosition == 0)
				{
					char st1[1] ; 
					sprintf(st1, "%d", ForFlash.page[NumberPage].size);
					LCD_String(st1);
				}
				if (CursorPosition > 0)
				{
				LCD_SendChar(ForFlash.page[NumberPage].Name[CursorPosition - 1]);
				}
				toggle = 0;
			}
			
		}
		if (OldCursorPosition != CursorPosition)
		{
			LCD_SetPos (5, 0);
						char st11[1] ; 
						sprintf(st11, "%d", ForFlash.page[NumberPage].size);
						LCD_String(st11);
			LCD_SetPos (0, 1);
			LCD_String(ForFlash.page[NumberPage].Name);
			LCD_SetPos (MassPosition[CursorPosition][0], MassPosition[CursorPosition][1]);
		}
	
}


/////////////////////////////////////////////////////
uint8_t ControlToggle (GPIO_PinState status)
{
	static uint8_t LastStatusToggle = 0;
	// Noize Protections section
	uint8_t ToggleStatus = 0;
	static uint16_t FirstCounter = 0;
	
	if (status == GPIO_PIN_SET) FirstCounter++;
	if (status == GPIO_PIN_RESET) FirstCounter--;

	if (FirstCounter > 5)
		{ // filter noize 
			ToggleStatus = 1;
			FirstCounter = 5;
			LastStatusToggle = 1;
		}
		if (FirstCounter < 1 && LastStatusToggle == 1)
		{	//rewrite flash
						FLASH_EraseInitTypeDef EraseInitStruct;
						EraseInitStruct.TypeErase=FLASH_TYPEERASE_PAGES;
						EraseInitStruct.NbPages=0x01;
						EraseInitStruct.PageAddress=PAGE_ADRESS;
								
						uint32_t PAGEError = 1;
					
						HAL_FLASH_Unlock();
						HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
						
							uint32_t adress = PAGE_ADRESS;
							uint8_t index = 0;
							while(index <= 120)
							{
								if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, adress, ForFlash.FlashMass[index])== HAL_OK)
								{
									adress += 4;
									index++;
								}
							}
						HAL_FLASH_Lock();	
					
		}
		
	if (FirstCounter < 1)
		{
			ToggleStatus = 0;
			FirstCounter = 1;
			LastStatusToggle = 0;
		}		
return ToggleStatus;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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
	//fillet stucture for Erase function
		 FLASH_EraseInitTypeDef EraseInitStruct;
	   EraseInitStruct.TypeErase=FLASH_TYPEERASE_PAGES;
     EraseInitStruct.NbPages=0x01;
     EraseInitStruct.PageAddress=PAGE_ADRESS;
	
		uint32_t PAGEError = 1;
	
	// read data from flash
		 uint32_t adress = PAGE_ADRESS;
		 char index = 0;
			while(index <= 120)
			{
					ForFlash.FlashMass[index]	= *(__IO uint32_t*) adress;
					adress += 4;
					index++;
			}	
			
// checking for matching arrays 	
	if (ForFlash.page[0].Flag != 18)
{	
		for (char i = 0; i<=20; i++)
		{
			// plased structures
	    char *localpointer = "________________";
			char ar[2] = {'0','0'};
			sprintf(ar, "%d", i+1);
				ForFlash.page[i].number[0] = ar[0];
				ForFlash.page[i].number[1] = ar[1];
				ForFlash.page[i].size = 2;
				ForFlash.page[i].speed = 200;
				ForFlash.page[i].Flag = 18;
			
			for (char j=0; j<16; j++)
			ForFlash.page[i].Name[j] = *localpointer++; 
			
			// write structure in flash
			
		HAL_FLASH_Unlock();
		HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
		
			 adress = PAGE_ADRESS;
			 index = 0;
			while(index <= 120)
			{
				if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, adress, ForFlash.FlashMass[index])== HAL_OK)
				{
					adress += 4;
					index++;
				}
			}
		HAL_FLASH_Lock();	
		}
}
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	LCD_ini();
	HAL_Delay(50);
	
	
	 // make mask for  LCD
	LCD_SetPos(0, 0);
	char ch = 'N';
	LCD_SendChar(ch);
	LCD_String(ForFlash.page[0].number);
	LCD_SetPos(5, 0);
	char ar1[1] ; 
	sprintf(ar1, "%d", ForFlash.page[0].size);
	LCD_String(ar1);
	LCD_String("/4  ");
	char ar3[3];
	sprintf(ar3, "%d", SpeedBPM);
	LCD_String(ar3);
	LCD_String("bpm");
	LCD_SetPos(0 ,1 );
	LCD_String(ForFlash.page[0].Name);

  /* USER CODE END 2 */
// впарлощдзхжэзлорпа
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		SpeedBPM = ForFlash.page[NumberPage].speed;
		BPMmaster(SpeedBPM);
		uint8_t Toggle = ControlToggle(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7));
		//program mode	
			//program mode	
		if (Toggle == 1)
		{
				FlagButton1 = ButtonHendler11(FlagButt1);
				FlagButton2 = ButtonHendler22(FlagButt2);
	
			
			ProgramMode(FlagButton1, FlagButton2 );
			
		}
	 // Standart mode
		if (Toggle == 0)
		{
				FlagButton1 = ButtonHendler1(FlagButt1);
				FlagButton2 = ButtonHendler2(FlagButt2);
	

			if ( FlagButton2 == 1) ForFlash.page[NumberPage].speed++;
			if ( FlagButton2 == 2) 
				{
					NumberPage++;
					LCD_SetPos(0 ,1 );
					LCD_String(ForFlash.page[NumberPage].Name);
					
				}
			if ( FlagButton1 == 1) ForFlash.page[NumberPage].speed--;
			if ( FlagButton1 == 2) 
				{
					NumberPage--;
					LCD_SetPos(0 ,1 );
					LCD_String(ForFlash.page[NumberPage].Name);
					if (NumberPage < 10) LCD_SendChar(32);
				}
			if ( NumberPage > 19) NumberPage = 0;
			if ( NumberPage < 0) NumberPage = 19;
			
			LCD_SetPos(5 ,0 );
			char ar1[1];
			sprintf(ar1, "%d", ForFlash.page[NumberPage].size);
			LCD_String(ar1);	
			LCD_SetPos(10, 0);
				if (ForFlash.page[NumberPage].speed < 100) LCD_SendChar(32);
			char ar3[3];
			sprintf(ar3, "%d", ForFlash.page[NumberPage].speed);
			LCD_String(ar3);
			
			LCD_SetPos(1, 0);
			LCD_String(ForFlash.page[NumberPage].number);
			if (NumberPage < 10) LCD_SendChar(32);
		}
 
		
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
