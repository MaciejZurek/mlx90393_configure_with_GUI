/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mlx90393.h"
#include "stdio.h"
#include "string.h"
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Init(&huart2);
  __HAL_RCC_I2C1_CLK_ENABLE();		// enable clock signal for I2C

  HAL_Delay(100);

  char Data[255];
  char inf_temp[121];
  char* information_to_UART = "";
  uint8_t values[8];
  conf_state = WAIT_FOR_GUI;
  bool flag = true;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(conf_state)
	  {
	  	  case WAIT_FOR_GUI: // waiting for data to appear on UART

	  		if(flag)
	  		{
		  		HAL_UART_Receive(&huart2, values, 8, HAL_MAX_DELAY);
		  		store_data_from_gui_to_struct(values, &gui_settings); // send data from array with achieved data from gui to struct
		  		flag = false;
		  		HAL_Delay(100);
		  		continue;
	  		}

	  		HAL_UART_Receive(&huart2, values, 8, HAL_MAX_DELAY);
	  		store_data_from_gui_to_struct(values, &gui_settings); // send data from array with achieved data from gui to struct

	  		switch(values[7])
	  		{
	  			case 0:
	  				conf_state = READ_VM_MEMORY;
	  				information_to_UART = "Reading VM memory...";
	  				HAL_UART_Transmit(&huart2, (uint8_t*)information_to_UART, strlen(information_to_UART), HAL_MAX_DELAY);
	  				HAL_Delay(10);

	  				break;
	  			case 1:
		  			conf_state = WRITE_NVM_MEMORY;
	  				information_to_UART = "Storing volatile memory to non-volatile memory...";
	  				HAL_UART_Transmit(&huart2, (uint8_t*)information_to_UART, strlen(information_to_UART), HAL_MAX_DELAY);
	  				HAL_Delay(100);

		  			break;
	  			case 2:
	  				conf_state = CALIBRATION;
	  				information_to_UART = "Calibration is starting...";
	  				HAL_UART_Transmit(&huart2, (uint8_t*)information_to_UART, strlen(information_to_UART), HAL_MAX_DELAY);
	  				HAL_Delay(100);

	  				break;
	  			default:
	  				information_to_UART = "Wrong mode!";
	  				HAL_UART_Transmit(&huart2, (uint8_t*)information_to_UART, strlen(information_to_UART), HAL_MAX_DELAY);
	  				break;
	  		}

	  		  break;
	  	  case READ_VM_MEMORY:	// get current settings, pack to structure and send over UART // mode 0

	  		slave_settings.gain = get_gain();
	  		slave_settings.osr = get_osr();
	  		slave_settings.dig_filter = get_filter();
	  		slave_settings.res_x = get_resolution_X();
			slave_settings.res_y = get_resolution_Y();
			slave_settings.res_z = get_resolution_Z();
			slave_settings.offset_x = get_offset_X();
			slave_settings.offset_y = get_offset_Y();
			slave_settings.offset_z = get_offset_Z();

			sprintf((char*)inf_temp, 	"Gain: %d\n"\
										"OSR: %d\n"\
										"Digital filter: %d\n"\
										"Resolution X: %d\n"\
										"Resolution Y: %d\n"\
										"Resolution Z: %d\n"\
										"VM memory reading completed successfully.",\
										slave_settings.gain, slave_settings.osr,\
										slave_settings.dig_filter, slave_settings.res_x,\
										slave_settings.res_y, slave_settings.res_z);

			HAL_UART_Transmit(&huart2, (uint8_t*)inf_temp, strlen(inf_temp), HAL_MAX_DELAY);
			conf_state = WAIT_FOR_GUI;

	  		  break;
	  	  case WRITE_NVM_MEMORY:	// write VM to NVM // mode 1

	  		if(OK == write_non_volatile_mem())
	  		{
	  			information_to_UART = "Data storage in NVM completed successfully.";
	  			HAL_UART_Transmit(&huart2, (uint8_t*)information_to_UART, strlen(information_to_UART), HAL_MAX_DELAY);
	  			conf_state = WAIT_FOR_GUI;
	  		}

	  		  break;
	  	  case CALIBRATION:	// calibration

	  		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	  		// set parameters according to data from GUI
//	  		set_offsets(gui_settings.field_intensity);
	  		set_gain(gui_settings.gain);
	  		set_osr(gui_settings.osr);
	  		set_filter(gui_settings.dig_filter);
	  		set_resolution_X(gui_settings.res_x);
	  		set_resolution_Y(gui_settings.res_y);
	  		set_resolution_Z(gui_settings.res_z);


	  		// store them in struct
	  		slave_settings.gain = get_gain();
	  		slave_settings.osr = get_osr();
	  		slave_settings.dig_filter = get_filter();
	  		slave_settings.res_x = get_resolution_X();
	  		slave_settings.res_y = get_resolution_Y();
	  		slave_settings.res_z = get_resolution_Z();
	  		slave_settings.offset_x = get_offset_X();
	  		slave_settings.offset_y = get_offset_Y();
	  		slave_settings.offset_z = get_offset_Z();
	  		set_multipliers(&multiplier_x, &multiplier_y, &multiplier_z);

	  		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  		information_to_UART = "Calibration completed successfully.";
	  		HAL_UART_Transmit(&huart2, (uint8_t*)information_to_UART, strlen(information_to_UART), HAL_MAX_DELAY);
			conf_state = WAIT_FOR_GUI;

	  		  break;
	  	  default:
	  		  conf_state = WAIT_FOR_GUI;
	  		  break;
	  }

//	read_measure_X(&mlx_data);
//	calc_data.x = (mlx_data.X_axis + slave_settings.offset_x) * multiplier_x;
//	read_measure_Y(&mlx_data);
//	calc_data.y = (mlx_data.Y_axis + slave_settings.offset_y) * multiplier_y;
//	read_measure_Z(&mlx_data);
//	calc_data.z = (mlx_data.Z_axis + slave_settings.offset_z) * multiplier_z;


//	sprintf((char*) Data, "%4.2f\t%4.2f\t%4.2f\r\n", calc_data.x, calc_data.y, calc_data.z);
//	HAL_UART_Transmit(&huart2, Data, strlen(Data), HAL_MAX_DELAY);

//	HAL_Delay(100);
//	HAL_UART_Transmit(&huart2, "\n", 1, HAL_MAX_DELAY);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
