/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
//#include "Weighing_scale.h"
#include "stdio.h"
#include "string.h"

#define DEV_ADD ((0x2A)<<1)
#define REG_ADD 0
#define WRT_DAT 1
#define ADC_CTRL_REG  0x00
#define ADC_CTRL_REG1  0x01
#define ADC_RESULT_ADDRESS  0x12
#define ADC_RESET  0x01
#define ADC_START  0x02
#define ADC_INITIALIZE  0x3E

#define ADC_GAIN_CALIB  0x06
#define ADC_OFFSET_CALIB  0x03

//----------Multiplexer------------------------//
#define I2C_TCA9548A_ADDR			((0x70)<<1)	// write = 0xE0, read = 0xE1

#define CMD_TCA9548A_OFF			0x00
#define CMD_TCA9548A_CH0			0x01
#define CMD_TCA9548A_CH1			0x02
#define CMD_TCA9548A_CH2			0x04
#define CMD_TCA9548A_CH3			0x08
#define CMD_TCA9548A_CH4			0x10
#define CMD_TCA9548A_CH5			0x20
#define CMD_TCA9548A_CH6			0x40
#define CMD_TCA9548A_CH7			0x80


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint8_t i2cDataBuff_W[10];
uint8_t i2cDataBuff_R[10];
uint8_t i2cDataBuff2_R[10];
uint8_t ctrlReg2Val;
uint32_t gainCalib;
uint32_t offsetCalib;
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void I2C_channel_initADC(int channel);
long int read7802(int channel);

extern void overload(void);
extern void calculation(int channel);
extern void calibration(int channel);
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
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	I2C_channel_initADC(CMD_TCA9548A_CH0);

	I2C_channel_initADC(CMD_TCA9548A_CH1);
	//  I2C_channel_initADC(CMD_TCA9548A_CH2);
	//  I2C_channel_initADC(CMD_TCA9548A_CH3);

	 //command from display need to be discuss for channel 1
	if(0x01)
	calibration(CMD_TCA9548A_CH0);
	//command from display need to be discuss for channel 2
	if(0x02)
    calibration(CMD_TCA9548A_CH0);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		   calculation(CMD_TCA9548A_CH0);
		   calculation(CMD_TCA9548A_CH1);
		   overload();
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
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}
long int read7802(int channel)
{
	//	int32_t test_value = 0;
	int32_t test_value[5] = {0};
	//	i2cDataBuff_R[0] = 10;
	HAL_StatusTypeDef ret = 0;

	i2cDataBuff_W[0]=(CMD_TCA9548A_OFF| channel);
	//	  i2cDataBuff_W[WRT_DAT]=ADC_RESET;
	ret = HAL_I2C_Master_Transmit(&hi2c1,I2C_TCA9548A_ADDR,&i2cDataBuff_W[REG_ADD],1,HAL_MAX_DELAY);
	HAL_Delay(100);

	i2cDataBuff_W[REG_ADD]=ADC_CTRL_REG;
	//	  i2cDataBuff_W[WRT_DAT]=0x3E;
	ret = HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,&i2cDataBuff_W[REG_ADD],1,HAL_MAX_DELAY);
	ret = HAL_I2C_Master_Receive(&hi2c1,DEV_ADD,&i2cDataBuff_R[0],1,HAL_MAX_DELAY);
	HAL_Delay(100);

	if((i2cDataBuff_R[0] & 0x20) != 0)
	{
		i2cDataBuff_W[REG_ADD]=ADC_RESULT_ADDRESS;
		ret = HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,&i2cDataBuff_W[REG_ADD],1,HAL_MAX_DELAY);
		ret = HAL_I2C_Master_Receive(&hi2c1,DEV_ADD,&i2cDataBuff2_R[0],3,HAL_MAX_DELAY);
		HAL_Delay(100);

		//  		i2cDataBuff_W[REG_ADD]=0x13;
		//  		ret = HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,&i2cDataBuff_W[REG_ADD],1,HAL_MAX_DELAY);
		//  		ret = HAL_I2C_Master_Receive(&hi2c1,DEV_ADD,&i2cDataBuff2_R[1],1,HAL_MAX_DELAY);
		//  		HAL_Delay(100);
		//
		//  		i2cDataBuff_W[REG_ADD]=0x14;
		//  		ret = HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,&i2cDataBuff_W[REG_ADD],1,HAL_MAX_DELAY);
		//  		ret = HAL_I2C_Master_Receive(&hi2c1,DEV_ADD,&i2cDataBuff2_R[2],1,HAL_MAX_DELAY);
		//  		HAL_Delay(100);

		test_value[channel-1] = (i2cDataBuff2_R[0] * 0x1000) + (i2cDataBuff2_R[1] * 0x100 ) + (i2cDataBuff2_R[2]);
		//if ADC value is negative (bit 24 set)
		if(test_value[channel-1] & 0x00800000)
		{
			//set the high bits to force sign extension
			test_value[channel-1] |= 0xff000000;
		}
		// return(count+2000000);

	}
	 return(test_value[channel-1] );
}

void I2C_channel_initADC(int channel)
{
	int32_t test_value = 0;
	HAL_StatusTypeDef ret = 0;
	i2cDataBuff_W[0]=(CMD_TCA9548A_OFF| channel);
	//	  i2cDataBuff_W[WRT_DAT]=ADC_RESET;
	ret = HAL_I2C_Master_Transmit(&hi2c1,I2C_TCA9548A_ADDR,&i2cDataBuff_W[REG_ADD],1,HAL_MAX_DELAY);
	HAL_Delay(100);

	//	  i2cDataBuff_W[DEV_ADD]=0x29;
	i2cDataBuff_W[REG_ADD]=ADC_CTRL_REG;
	i2cDataBuff_W[WRT_DAT]=ADC_RESET;
	ret = HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,&i2cDataBuff_W[REG_ADD],2,HAL_MAX_DELAY);
	HAL_Delay(100);

	i2cDataBuff_W[REG_ADD]=ADC_CTRL_REG;
	i2cDataBuff_W[WRT_DAT]=ADC_START;
	ret = HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,&i2cDataBuff_W[REG_ADD],2,HAL_MAX_DELAY);
	HAL_Delay(100);

	i2cDataBuff_W[REG_ADD]=ADC_CTRL_REG;
	//	  i2cDataBuff_W[WRT_DAT]=0x3E;
	ret = HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,&i2cDataBuff_W[REG_ADD],1,HAL_MAX_DELAY);
	ret = HAL_I2C_Master_Receive(&hi2c1,DEV_ADD,&i2cDataBuff_R[0],1,HAL_MAX_DELAY);
	HAL_Delay(100);

	i2cDataBuff_W[REG_ADD]=ADC_CTRL_REG;
	i2cDataBuff_W[WRT_DAT]=ADC_INITIALIZE;
	ret = HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,&i2cDataBuff_W[REG_ADD],2,HAL_MAX_DELAY);
	HAL_Delay(100);

	i2cDataBuff_W[REG_ADD]=ADC_CTRL_REG;
	//	  i2cDataBuff_W[WRT_DAT]=0x3E;
	ret = HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,&i2cDataBuff_W[REG_ADD],1,HAL_MAX_DELAY);
	ret = HAL_I2C_Master_Receive(&hi2c1,DEV_ADD,&i2cDataBuff_R[0],1,HAL_MAX_DELAY);
	HAL_Delay(100);

	i2cDataBuff_W[REG_ADD]=ADC_CTRL_REG1;
	//  	  i2cDataBuff_W[WRT_DAT]=ADC_START;
	ret = HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,&i2cDataBuff_W[REG_ADD],1,HAL_MAX_DELAY);
	ret = HAL_I2C_Master_Receive(&hi2c1,DEV_ADD,&i2cDataBuff_R[0],1,HAL_MAX_DELAY);

	ctrlReg2Val = i2cDataBuff_R[0];

	HAL_Delay(100);

	//memset(i2cDataBuff_R,0,10);

	test_value = i2cDataBuff_R[0];
	i2cDataBuff_W[REG_ADD]=ADC_GAIN_CALIB;
	i2cDataBuff_W[WRT_DAT]=0;
	ret = HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,&i2cDataBuff_W[REG_ADD],1,HAL_MAX_DELAY);
	// ret = HAL_I2C_Master_Receive(&hi2c1,DEV_ADD,&i2cDataBuff_R[0],3,HAL_MAX_DELAY);

	HAL_Delay(100);

	memset(i2cDataBuff_R,0,10);

	i2cDataBuff_W[REG_ADD]=ADC_GAIN_CALIB;
	ret = HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,&i2cDataBuff_W[REG_ADD],1,HAL_MAX_DELAY);
	ret = HAL_I2C_Master_Receive(&hi2c1,DEV_ADD,&i2cDataBuff_R[0],4,HAL_MAX_DELAY);

	gainCalib = (i2cDataBuff_R[0] << 16) + (i2cDataBuff_R[1] << 8 ) + (i2cDataBuff_R[2]);
	HAL_Delay(100);

	memset(i2cDataBuff_R,0,10);

	i2cDataBuff_W[REG_ADD]=ADC_OFFSET_CALIB;
	ret = HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,&i2cDataBuff_W[REG_ADD],1,HAL_MAX_DELAY);
	ret = HAL_I2C_Master_Receive(&hi2c1,DEV_ADD,&i2cDataBuff_R[0],4,HAL_MAX_DELAY);

	offsetCalib = (i2cDataBuff_R[0] << 16) + (i2cDataBuff_R[1] << 8 ) + (i2cDataBuff_R[2]);
	HAL_Delay(100);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
