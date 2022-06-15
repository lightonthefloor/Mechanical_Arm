/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "servo.h"
#include "A1_Drive.h"
#include "PID_Controlor.h"
#include "Inverse_Mechanical_Calulation.h"
#include "CAN_Operation.h"

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Motor_Move_A1_AHandle;
osThreadId Motor_Move_A1_BHandle;
osThreadId Motor_Move_SevoHandle;
osThreadId Motor_Move_RHandle;
osThreadId Switcher_OpenHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Motor_Task_A1_A(void const * argument);
void Motor_Task_A1_B(void const * argument);
void Motor_Task_Sevo(void const * argument);
void Motor_Task_R(void const * argument);
void Switcher_Open_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Motor_Move_A1_A */
  osThreadDef(Motor_Move_A1_A, Motor_Task_A1_A, osPriorityHigh, 0, 128);
  Motor_Move_A1_AHandle = osThreadCreate(osThread(Motor_Move_A1_A), NULL);

//  /* definition and creation of Motor_Move_A1_B */
  osThreadDef(Motor_Move_A1_B, Motor_Task_A1_B, osPriorityHigh, 0, 128);
  Motor_Move_A1_BHandle = osThreadCreate(osThread(Motor_Move_A1_B), NULL);

//  /* definition and creation of Motor_Move_Sevo */
//  osThreadDef(Motor_Move_Sevo, Motor_Task_Sevo, osPriorityHigh, 0, 128);
//  Motor_Move_SevoHandle = osThreadCreate(osThread(Motor_Move_Sevo), NULL);

//  /* definition and creation of Motor_Move_R */
//  osThreadDef(Motor_Move_R, Motor_Task_R, osPriorityIdle, 0, 128);
//  Motor_Move_RHandle = osThreadCreate(osThread(Motor_Move_R), NULL);
//
//  /* definition and creation of Switcher_Open */
//  osThreadDef(Switcher_Open, Switcher_Open_Task, osPriorityHigh, 0, 128);
//  Switcher_OpenHandle = osThreadCreate(osThread(Switcher_Open), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		Base_Speed_Calculation(1,1000);
		CAN_CMD_Current(Current[1],0,0,0);
		osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Motor_Task_A1_A */
/**
* @brief Function implementing the Motor_Move_A1_A thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Task_A1_A */
void Motor_Task_A1_A(void const * argument)
{
  /* USER CODE BEGIN Motor_Task_A1_A */
  /* Infinite loop */
  while(1)
	{
		A1_Motor_Position_Control(0,Theta1);
		osDelay(100);
	}
  /* USER CODE END Motor_Task_A1_A */
}

/* USER CODE BEGIN Header_Motor_Task_A1_B */
/**
* @brief Function implementing the Motor_Move_A1_A thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Task_A1_B */
void Motor_Task_A1_B(void const * argument)
{
  /* USER CODE BEGIN Motor_Task_A1_B */
  /* Infinite loop */
  while(1)
  {
		A1_Motor_Position_Control(0,Theta2);
		osDelay(100);
  }
  /* USER CODE END Motor_Task_A1_B */
}

/* USER CODE BEGIN Header_Motor_Task_Sevo */
/**
* @brief Function implementing the Motor_Move_Sevo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Task_Sevo */
void Motor_Task_Sevo(void const * argument)
{
  /* USER CODE BEGIN Motor_Task_Sevo */
  /* Infinite loop */
  while(1){
		int angle = (int)((180.0f/PI)*Theta3);
		Servo_Goal_Position(1,angle);
		osDelay(100);
	}
  /* USER CODE END Motor_Task_Sevo */
}

/* USER CODE BEGIN Header_Motor_Task_R */
/**
* @brief Function implementing the Motor_Move_R thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Task_R */
void Motor_Task_R(void const * argument)
{
  /* USER CODE BEGIN Motor_Task_R */
  /* Infinite loop */
	int ID = 1;
  while(1)
	{
		if (Start_Turn){
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
				Base_Speed_Calculation(ID,200);
				CAN_CMD_Current(Current[1],Current[2],Current[3],Current[4]);
				osDelay(10);
			}
			int goal_angle = (int)((float)(round-(float)((int)round))*8191.0f)+now_angle;
			if (goal_angle > 8000){
				while(Now_Angle_Normal[ID] < 8000)
				{
					Base_Speed_Calculation(ID,200);
					CAN_CMD_Current(Current[1],Current[2],Current[3],Current[4]);
					osDelay(10);
				}
				Base_Speed_Calculation(ID,200);
				CAN_CMD_Current(Current[1],Current[2],Current[3],Current[4]);
				osDelay(10);
				while (Now_Angle_Normal[ID] < goal_angle - 8000){
					Base_Speed_Calculation(ID,200);
					CAN_CMD_Current(Current[1],Current[2],Current[3],Current[4]);
					osDelay(10);
				}
				Start_Turn = 0;
			}else{
				while(abs(Now_Angle_Normal[ID]-goal_angle)<100)
				{
					Base_Speed_Calculation(ID,100);
					CAN_CMD_Current(Current[1],Current[2],Current[3],Current[4]);
					osDelay(10);
				}
			}
			Start_Turn = 0;
		}else{
			Base_Speed_Calculation(ID,0);
			CAN_CMD_Current(Current[1],Current[2],Current[3],Current[4]);
			osDelay(10);
		}
	}
  /* USER CODE END Motor_Task_R */
}

/* USER CODE BEGIN Header_Switcher_Open_Task */
/**
* @brief Function implementing the Switcher_Open thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Switcher_Open_Task */
void Switcher_Open_Task(void const * argument)
{
  /* USER CODE BEGIN Switcher_Open_Task */
  /* Infinite loop */
  while(1)
	{
		if (Switch_State){
			HAL_GPIO_WritePin(Switcher_GPIO_Port,Switcher_Pin,GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(Switcher_GPIO_Port,Switcher_Pin,GPIO_PIN_RESET);
		}
	}
  /* USER CODE END Switcher_Open_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
