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
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN_Operation.h"
#include "A1_Drive.h"
#include "Inverse_Mechanical_Calulation.h"
#include "PID_Controlor.h"
#include "M6020_PID_Controller.h"
#include "servo.h"

#define PI 3.1415926535f
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
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int Stable = 0;
int Received = 0;

double Target_X,Target_Y;

void CAN_Recieved_Data_Dealer()
{
	char tag = CAN_Recieved_Message[0];
	if (tag == 'M'){
		int xx = (int16_t)(CAN_Recieved_Message[1]<<8 | CAN_Recieved_Message[2]);
		int yy = (int16_t)(CAN_Recieved_Message[3]<<8 | CAN_Recieved_Message[4]);
		int aa = (int16_t)(CAN_Recieved_Message[5]<<8 | CAN_Recieved_Message[6]);
		Angle_R = (double)aa;
		Switch_State = CAN_Recieved_Message[7];
		Start_Turn = 1;
		double x = (double)xx/1000.0f;
		double y = (double)yy/1000.0f;
		Target_X = x;
		Target_Y = y;
	}else if (tag == 'S'){
		Switch_State = (int16_t)(CAN_Recieved_Message[1]<<8 | CAN_Recieved_Message[2]);
	}else if (tag == 'T'){
		int Speed = (int16_t)(CAN_Recieved_Message[1]<<8 | CAN_Recieved_Message[2]);
		Base_Speed_Calculation(1,Speed);
		CAN_CMD_Current(Current[1],0,0,0);
	}else if (tag == 'C'){
		Stable = 1;
	}else if (tag == 'A'){
		int aa1 = (int16_t)(CAN_Recieved_Message[1]<<8 | CAN_Recieved_Message[2]);
		int aa2 = (int16_t)(CAN_Recieved_Message[3]<<8 | CAN_Recieved_Message[4]);
		int aa3 = (int16_t)(CAN_Recieved_Message[5]<<8 | CAN_Recieved_Message[6]);
		Theta1 = ((float)aa1/180.0f)*PI;
		Theta2 = ((float)aa2/180.0f)*PI;
		A1_Start_Move_A = 1;
		A1_Start_Move_B = 1;
		Servo_Goal_Position(1,aa3);
	}else if (tag == 'R') Received = 1;
}

void Speed_Position_Control(int ID, float Speed,float Angle)
{
	int start_count = 1;
	int now_angle = Now_Angle_Normal[ID];
	float round = (19.0f*Angle_R)/360.0f;
	for (int i=1;i<=(int)round;){
		if (Now_Angle_Normal[ID] > 8000 && start_count){
			i++;
			start_count = 0;
		}else if (!start_count && Now_Angle_Normal[ID] < 8000){
			start_count = 1;
		}
		Base_Speed_Calculation(ID,100);
		CAN_CMD_Current(Current[1],Current[2],Current[3],Current[4]);
		HAL_Delay(10);
	}
	for (int i=1;i<=50;i++){
		Base_Speed_Calculation(ID,100);
		CAN_CMD_Current(Current[1],Current[2],Current[3],Current[4]);
		HAL_Delay(10);
	}

	int goal_angle = (int)((float)(round-(float)((int)round))*8191.0f);
//	if (goal_angle > 8000){
//		while(Now_Angle_Normal[ID] < 8000)
//		{
//			Base_Speed_Calculation(ID,200);
//			CAN_CMD_Current(Current[1],Current[2],Current[3],Current[4]);
//			HAL_Delay(10);
//		}
//		Base_Speed_Calculation(ID,100);
//		CAN_CMD_Current(Current[1],Current[2],Current[3],Current[4]);
//		HAL_Delay(10);
//		while (Now_Angle_Normal[ID] < goal_angle - 8000){
//			Base_Speed_Calculation(ID,100);
//			CAN_CMD_Current(Current[1],Current[2],Current[3],Current[4]);
//			HAL_Delay(10);
//		}
//		Start_Turn = 0;
//	}else{
//		while(abs(Now_Angle_Normal[ID]-goal_angle)>100)
//		{
//			Base_Speed_Calculation(ID,100);
//			CAN_CMD_Current(Current[1],Current[2],Current[3],Current[4]);
//			HAL_Delay(10);
//		}
//	}
//	goal_angle = goal_angle + now_angle;
//	if (goal_angle > 8000) goal_angle -= 8000;
//	while(abs(Now_Angle_Normal[ID]-goal_angle)>100)
//	{
//		Current_Position_Control(1,goal_angle);
//		HAL_Delay(10);
//	}
	for (int i=1;i<=5;i++){
		Base_Speed_Calculation(ID,100);
		CAN_CMD_Current(0,Current[2],Current[3],Current[4]);
		HAL_Delay(10);
	}
	Start_Turn = 0;
}
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART6_UART_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */
	Usart6_TX_DMA_Init();
	USART6_Rx_Init();
	CAN2_Filter_Init();
	Tuner_Control_Init();

	M6020_Control_Init();

	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
	//Servo_Led(1,1);
//	Servo_Torque(1,1);
//	int pos = (4095.0f/360.0f)*90.0f;
//	Servo_Goal_Position(1,pos);

	Received = 0;
	while (!Received){
		CAN_Transmit_Message('P',0,0,0,0,0x301);
		HAL_Delay(10);
	}

	Servo_Torque(1,1);
	//Servo_Goal_Position(1,0);

	while(!Stable)
	{
		HAL_Delay(10);
	}

	HAL_Delay(100);
	for (int i=1;i<=10;i++){
		A1_Motor_Multiple_Control(0,0,0,0,0);
		HAL_Delay(10);
		A1_Motor_Multiple_Control(1,0,0,0,0);
		HAL_Delay(10);
	}

	Set_Zero[0] = A1_State[0].Position;
	Set_Zero[1] = A1_State[1].Position;

	for (int i=1;i<=10;i++){
		A1_Motor_Position_Control(0,Set_Zero[0]);
		HAL_Delay(10);
		A1_Motor_Position_Control(1,Set_Zero[1]);
		HAL_Delay(10);
	}



	HAL_Delay(1000);
//	float temp = -(PI/180.0f)*80.0f;
//	A1_Motor_Position_Control(0,temp);
//	HAL_Delay(5);
//	temp = (PI/180.0f)*90.0f;
//	A1_Motor_Position_Control(1,temp);
//	HAL_Delay(5);
//	int pos = -(4095.0f/360.0f)*90.0f;
//	Servo_Goal_Position(1,pos);

	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
//  MX_FREERTOS_Init();
//  /* Start scheduler */
//  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//
		if (Start_Turn){
//			A1_Motor_Position_Control(0,Theta1);
//			HAL_Delay(10);
//			A1_Motor_Position_Control(1,Theta2);
//			HAL_Delay(10);
//			int angle = (int)((180.0f/PI)*Theta3);
//			Servo_Goal_Position(1,angle);
//			Speed_Position_Control(1,200,90.0f);
//			CAN_Transmit_Message('C',1,1,1,1,0x301);
//			HAL_Delay(10);
			Arm_Move(Target_X,Target_Y);
			Start_Turn = 0;
			CAN_Transmit_Message('C',1,1,1,1,0x301);
		}else{
			//A1_Motor_Multiple_Control(0xBB,0,0,0,0);
//			A1_Motor_Position_Control(0,Theta1);
//			HAL_Delay(10);
//			A1_Motor_Position_Control(1,Theta2);
//			HAL_Delay(100);
//			Base_Speed_Calculation(1,0);
//			CAN_CMD_Current(Current[1],0,0,0);
//			HAL_Delay(10);
		}

		if (Switch_State){
			HAL_GPIO_WritePin(Switcher_GPIO_Port,Switcher_Pin,GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(Switcher_GPIO_Port,Switcher_Pin,GPIO_PIN_RESET);
		}

		Voltage_Position_Control(3,804);
		HAL_Delay(10);
		//CAN_CMD_Current(1000,0,0,0);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 144;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

