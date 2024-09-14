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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "robotArm.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define HIGH_STEP_HTIM htim1
#define LOW_STEP_HTIM htim4
#define ROT_STEP_HTIM htim3
#define GRIPPER_HTIM htim10
#define HIGH_STEP_TIM_CHANNEL TIM_CHANNEL_3
#define LOW_STEP_TIM_CHANNEL TIM_CHANNEL_2
#define ROT_STEP_TIM_CHANNEL TIM_CHANNEL_2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct robotArm_s robotArm;
uint8_t ReceiveBuffer[20];
uint8_t RxEventFlag=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Command_OK(void);
void Command_Error(void);
void Command_ClearBuffer(void);
void Command_InitRobot(void);
void Command_Control_Program(void);
void RobotArm_UART_loop(void);
void RobotArm_Control_Program(void);
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  Command_InitRobot();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 RobotArm_UART_loop();

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*
 * Obsługa przerwania od licznika taktującego silnik chwytaka
 * Zmienia takty na zasadzie maszyny stanów
 * Inkrementuje licznik kroków
 * Jeżeli licznik kroków obliczy wszystkie kroki do wykonania, to wyłączy chwytak
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==robotArm.gripper_Stepper.timer.htim->Instance){
		//Full step drive gripper
		switch (robotArm.gripper_Stepper.tick) {
			case 0:
				HAL_GPIO_WritePin(GRIPPER_IN1_GPIO_Port, GRIPPER_IN1_Pin, SET);
				HAL_GPIO_WritePin(GRIPPER_IN2_GPIO_Port, GRIPPER_IN2_Pin, SET);
				HAL_GPIO_WritePin(GRIPPER_IN3_GPIO_Port, GRIPPER_IN3_Pin, RESET);
				HAL_GPIO_WritePin(GRIPPER_IN4_GPIO_Port, GRIPPER_IN4_Pin, RESET);

				robotArm.gripper_Stepper.step_counter++;
				if(robotArm.gripper_Stepper.step_counter >= robotArm.gripper_Stepper.steps_to_count)
				{
					gripper_stop(&robotArm.gripper_Stepper);
				}

				if(robotArm.gripper_Stepper.state==open)robotArm.gripper_Stepper.tick++;
				else robotArm.gripper_Stepper.tick=3;
				break;
			case 1:
				HAL_GPIO_WritePin(GRIPPER_IN1_GPIO_Port, GRIPPER_IN1_Pin, RESET);
				HAL_GPIO_WritePin(GRIPPER_IN2_GPIO_Port, GRIPPER_IN2_Pin, SET);
				HAL_GPIO_WritePin(GRIPPER_IN3_GPIO_Port, GRIPPER_IN3_Pin, SET);
				HAL_GPIO_WritePin(GRIPPER_IN4_GPIO_Port, GRIPPER_IN4_Pin, RESET);

				robotArm.gripper_Stepper.step_counter++;
				if(robotArm.gripper_Stepper.step_counter >= robotArm.gripper_Stepper.steps_to_count)
				{
					gripper_stop(&robotArm.gripper_Stepper);
				}

				if(robotArm.gripper_Stepper.state==open)robotArm.gripper_Stepper.tick++;
				else robotArm.gripper_Stepper.tick--;
				break;
			case 2:
				HAL_GPIO_WritePin(GRIPPER_IN1_GPIO_Port, GRIPPER_IN1_Pin, RESET);
				HAL_GPIO_WritePin(GRIPPER_IN2_GPIO_Port, GRIPPER_IN2_Pin, RESET);
				HAL_GPIO_WritePin(GRIPPER_IN3_GPIO_Port, GRIPPER_IN3_Pin, SET);
				HAL_GPIO_WritePin(GRIPPER_IN4_GPIO_Port, GRIPPER_IN4_Pin, SET);

				robotArm.gripper_Stepper.step_counter++;
				if(robotArm.gripper_Stepper.step_counter >= robotArm.gripper_Stepper.steps_to_count)
				{
					gripper_stop(&robotArm.gripper_Stepper);
				}

				if(robotArm.gripper_Stepper.state==open)robotArm.gripper_Stepper.tick++;
				else robotArm.gripper_Stepper.tick--;
				break;
			case 3:
				HAL_GPIO_WritePin(GRIPPER_IN1_GPIO_Port, GRIPPER_IN1_Pin, SET);
				HAL_GPIO_WritePin(GRIPPER_IN2_GPIO_Port, GRIPPER_IN2_Pin, RESET);
				HAL_GPIO_WritePin(GRIPPER_IN3_GPIO_Port, GRIPPER_IN3_Pin, RESET);
				HAL_GPIO_WritePin(GRIPPER_IN4_GPIO_Port, GRIPPER_IN4_Pin, SET);

				robotArm.gripper_Stepper.step_counter++;
				if(robotArm.gripper_Stepper.step_counter >= robotArm.gripper_Stepper.steps_to_count)
				{
					gripper_stop(&robotArm.gripper_Stepper);
				}

				if(robotArm.gripper_Stepper.state==open)robotArm.gripper_Stepper.tick=0;
				else robotArm.gripper_Stepper.tick--;
				break;
		}
	}
}

/*
 * Obsługa przerwania od zakończenia impulsu PWM:
 * Sprawdza z którego timera doszło przerwanie
 * Inkrementuje licznik kroków
 * Jeżeli licznik kroków obliczy wszystkie kroki do wykonania, to wylaczy silnik
 *
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == robotArm.high_Stepper.timer.htim->Instance)
	{
		robotArm.high_Stepper.step_counter++;

		if(robotArm.high_Stepper.step_counter >= robotArm.high_Stepper.steps_to_count)
		{
			stepper_stop(&robotArm.high_Stepper);
		}
	}
	if(htim->Instance == robotArm.low_Stepper.timer.htim->Instance)
	{
		robotArm.low_Stepper.step_counter++;

		if(robotArm.low_Stepper.step_counter >= robotArm.low_Stepper.steps_to_count)
		{
			stepper_stop(&robotArm.low_Stepper);
		}
	}
	if(htim->Instance == robotArm.rot_Stepper.timer.htim->Instance)
	{
		robotArm.rot_Stepper.step_counter++;

		if(robotArm.rot_Stepper.step_counter >= robotArm.rot_Stepper.steps_to_count)
		{
			stepper_stop(&robotArm.rot_Stepper);
		}
	}
}
/*
 * Obsługa przerwania od odbierania UART
 * * Wystawia flagę inforumującą o tym, że dane zostały odebrane i umieszczone w buforze odczytu
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance == USART2){
		RxEventFlag=1;
	}
}

/*
 * Inicjalizacja robota:
 * Wyświetla informację o poprawnym połączeniu
 * Wysyła informację o konieczności inicjalizacji
 * Czeka, aż użytkownik ustawi ramiona robota w pozycje krańcowe wyślę komendę 'INIT'
 * Inicjalizuje robota zgodnie z pozycjami krańcowymi
 * Wyświetla informację o zakończeniu inicjalizacji
 */

void Command_InitRobot(void){
	HAL_UART_Transmit(&huart2, "\nConnected!\nMove robot to endstop position and send 'INIT'\n", strlen("\nConnected!\nMove robot to endstop position and send 'INIT'\n"), 100);
	HAL_UARTEx_ReceiveToIdle_IT(&huart2, ReceiveBuffer, 20);
	while(strcmp(ReceiveBuffer,"INIT")!=0);
	Command_ClearBuffer();
	robotArm_init(&robotArm);
	HAL_UART_Transmit(&huart2, "Robot init completed\n", strlen("Robot init completed\n"), 100);

}
/*
 * Obsługa nieprawidłowo wpisanej komendy:
 * Wyświetla menu z dostępnymi komendami
 * Czyści bufor odczytu
 */
void Command_Error(void){
	uint8_t errortext[]="\n\nAvaliable commands:\n\nHOME\t\t\t-SET HOME POSITION X:0 Y:120 Z:120\nX:__ Y:__ Z:__\t\t-Set position XYZ\nG O/C\t\t\t-Gripper Open/Close\nSTART\t\t\t-Start program\nSTOP\t\t\t-Stop program\n";
	HAL_UART_Transmit(&huart2, errortext, strlen(errortext), 100);
	Command_ClearBuffer();

}
/*
 * Czyszczenie bufora:
 * Wypełnienie wszystkich elementów bufora odczytu pustymi znakami '\0'
 * Wyzerowanie flagi odczytu po UART
 * Zlecenie odczytu UART w trybie przerwaniowym
 */
void Command_ClearBuffer(void){
	for ( uint8_t i = 0; i <= (20-1); ++i) {
		ReceiveBuffer[i]='\0';
	}
	RxEventFlag=0;
	HAL_UARTEx_ReceiveToIdle_IT(&huart2, ReceiveBuffer, 20);
}
/*
 * Wyświetlenie komunikatu o starcie programu użytkownika
 */
void Command_Control_Program(void){
	uint8_t text[]="\nStarted control program. To stop send 'STOP'";
	HAL_UART_Transmit(&huart2, text, strlen(text), 100);

}

/*
 * Główna pętla obsługi menu użytkownika
 * Oczekiwanie na flagę odczytu UART
 * Gdy flaga nadejdzie, obsługuje komendę
 * Gdy komenda jest nieprawidłowa wyświetla dostępne komendy
 */

void RobotArm_UART_loop(void){
	if(RxEventFlag){
			if(ReceiveBuffer[0]=='X'){
				float ReceiveX,ReceiveY,ReceiveZ;
				if(sscanf(ReceiveBuffer,"X:%f Y:%f Z:%f",&ReceiveX,&ReceiveY,&ReceiveZ)==3){
					Command_ClearBuffer();
					robotArm_moveToXYZ(&robotArm, ReceiveX, ReceiveY, ReceiveZ);
				}
				else Command_Error();
			}
			else if(ReceiveBuffer[0]=='G'){

				if(ReceiveBuffer[2]=='O'){
					Command_ClearBuffer();
					robotArm_Gripper(&robotArm, open);
				}
				else if (ReceiveBuffer[2]=='C'){
					Command_ClearBuffer();
					robotArm_Gripper(&robotArm, close);
				}
				else Command_Error();
			}
			else if(strcmp(ReceiveBuffer,"HOME")==0){
				Command_ClearBuffer();
				robotArm_moveToXYZ(&robotArm,X_HOME,Y_HOME,Z_HOME);
			}
			else if(strcmp(ReceiveBuffer,"START")==0){
				Command_Control_Program();
				Command_ClearBuffer();
				do {
					Command_ClearBuffer();
					RobotArm_Control_Program();
				} while (strcmp(ReceiveBuffer,"STOP")!=0);
				Command_ClearBuffer();
			}
			else Command_Error();
		  }
}
/*
 * Program sterujący robotem do zaimplementowania przez użytkownika
 */
void RobotArm_Control_Program(void){

	  robotArm_Gripper(&robotArm, open);
	  robotArm_moveToXYZ(&robotArm, 0, 120, -80);
	  robotArm_moveToXYZ(&robotArm, 0, 120, -110);
	  robotArm_Gripper(&robotArm, close);
	  robotArm_moveToXYZ(&robotArm, 120, 0, -80);
	  robotArm_moveToXYZ(&robotArm, 120, 0, -110);
	  robotArm_Gripper(&robotArm, open);

	  robotArm_moveToXYZ(&robotArm, 0, 120, 120);

	  robotArm_moveToXYZ(&robotArm, 120, 0, -80);
	  robotArm_moveToXYZ(&robotArm, 120, 0, -110);
	  robotArm_Gripper(&robotArm, close);
	  robotArm_moveToXYZ(&robotArm, 120, 0, 0);

	  robotArm_moveToXYZ(&robotArm, 0, 120, 0);
	  robotArm_moveToXYZ(&robotArm, 0, 120, -110);
	  robotArm_Gripper(&robotArm, open);
	  robotArm_moveToXYZ(&robotArm, 0, 120, 120);
}
/*
 * Obsługa błędu komunikacji UART:
 * Gdy wystąpi błąd w komunikacji UART, wysyła informację i programowo wykonuje reset procesora
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	HAL_UART_Transmit(&huart2, "\nUART error! Reset system...\n", strlen("\nUART error! Reset system...\n"), 100);
	HAL_NVIC_SystemReset();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
