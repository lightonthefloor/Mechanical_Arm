//
// Created by Trisoil on 2021/12/14.
//


#include <stdlib.h>
#include "PID_Controlor.h"
#include "CAN_Operation.h"
#include "gpio.h"
#include "main.h"
#include "can.h"
#include "math.h"


PID_Number PID_Speed_Control_Num[8];
PID_Number PID_Position_Control_Num[8];
int16_t Current[5];

// 3.5 0.05 0.3
void M3508_Speed_Control_Init(uint8_t ID_Num){
	PID_Speed_Control_Num[ID_Num].kp = 1.6f;
	PID_Speed_Control_Num[ID_Num].ki = 0.52f;
	PID_Speed_Control_Num[ID_Num].kd = 0.6f;
	PID_Speed_Control_Num[ID_Num].max_output  = 16384;
	PID_Speed_Control_Num[ID_Num].integral_limit = 10000;
	PID_Speed_Control_Num[ID_Num].deadband = 10;
	PID_Speed_Control_Num[ID_Num].max_err = 8000;
}

void M3508_Position_Control_Init(uint8_t ID_Num){
	PID_Position_Control_Num[ID_Num].kp = 0.1f;
	PID_Position_Control_Num[ID_Num].ki = 0;
	PID_Position_Control_Num[ID_Num].kd = 0.8f;
	PID_Position_Control_Num[ID_Num].max_output  = 1000;
	PID_Position_Control_Num[ID_Num].integral_limit = 10000;
	PID_Position_Control_Num[ID_Num].deadband = 200;
	PID_Position_Control_Num[ID_Num].max_err = 8000;
}

int16_t Speed_PID_Calculate(float Set_Speed, uint8_t ID_Num){
	PID_Speed_Control_Num[ID_Num].ki_output += PID_Speed_Control_Num[ID_Num].ki * PID_Speed_Control_Num[ID_Num].now_err;
    if (PID_Speed_Control_Num[ID_Num].ki_output > PID_Speed_Control_Num[ID_Num].integral_limit) {
			PID_Speed_Control_Num[ID_Num].ki_output = PID_Speed_Control_Num[ID_Num].integral_limit;
    }else if (PID_Speed_Control_Num[ID_Num].ki_output < -PID_Speed_Control_Num[ID_Num].integral_limit){
			PID_Speed_Control_Num[ID_Num].ki_output = -PID_Speed_Control_Num[ID_Num].integral_limit;
    }
	PID_Speed_Control_Num[ID_Num].kp_output = PID_Speed_Control_Num[ID_Num].kp * PID_Speed_Control_Num[ID_Num].now_err;
	PID_Speed_Control_Num[ID_Num].kd_output = PID_Speed_Control_Num[ID_Num].kd * (PID_Speed_Control_Num[ID_Num].now_err - PID_Speed_Control_Num[ID_Num].last_err);
	PID_Speed_Control_Num[ID_Num].output = PID_Speed_Control_Num[ID_Num].ki_output + PID_Speed_Control_Num[ID_Num].kp_output + PID_Speed_Control_Num[ID_Num].kd_output;
    if (PID_Speed_Control_Num[ID_Num].output > PID_Speed_Control_Num[ID_Num].max_output){
			PID_Speed_Control_Num[ID_Num].output = PID_Speed_Control_Num[ID_Num].max_output;
    }else if (PID_Speed_Control_Num[ID_Num].output < -PID_Speed_Control_Num[ID_Num].max_output){
			PID_Speed_Control_Num[ID_Num].output = -PID_Speed_Control_Num[ID_Num].max_output;
    }
    return PID_Speed_Control_Num[ID_Num].output;
}

int16_t Position_PID_Calculate(float Set_Position, uint8_t ID_Num){
	PID_Position_Control_Num[ID_Num].ki_output += PID_Position_Control_Num[ID_Num].ki * PID_Position_Control_Num[ID_Num].now_err;
	if (PID_Position_Control_Num[ID_Num].ki_output > PID_Position_Control_Num[ID_Num].integral_limit) {
		PID_Position_Control_Num[ID_Num].ki_output = PID_Position_Control_Num[ID_Num].integral_limit;
	}else if (PID_Position_Control_Num[ID_Num].ki_output < -PID_Position_Control_Num[ID_Num].integral_limit){
		PID_Position_Control_Num[ID_Num].ki_output = -PID_Position_Control_Num[ID_Num].integral_limit;
	}
	PID_Position_Control_Num[ID_Num].kp_output = PID_Position_Control_Num[ID_Num].kp * PID_Position_Control_Num[ID_Num].now_err;
	PID_Position_Control_Num[ID_Num].kd_output = PID_Position_Control_Num[ID_Num].kd * (PID_Position_Control_Num[ID_Num].now_err - PID_Position_Control_Num[ID_Num].last_err);
	PID_Position_Control_Num[ID_Num].output = PID_Position_Control_Num[ID_Num].ki_output + PID_Position_Control_Num[ID_Num].kp_output + PID_Position_Control_Num[ID_Num].kd_output;
	if (PID_Position_Control_Num[ID_Num].output > PID_Position_Control_Num[ID_Num].max_output){
		PID_Position_Control_Num[ID_Num].output = PID_Position_Control_Num[ID_Num].max_output;
	}else if (PID_Position_Control_Num[ID_Num].output < -PID_Position_Control_Num[ID_Num].max_output){
		PID_Position_Control_Num[ID_Num].output = -PID_Position_Control_Num[ID_Num].max_output;
	}
	return PID_Position_Control_Num[ID_Num].output;
}

void Base_Speed_Calculation(uint8_t ID_Num, float Set_Speed)
{
	PID_Speed_Control_Num[ID_Num].last_err = PID_Speed_Control_Num[ID_Num].now_err;
	PID_Speed_Control_Num[ID_Num].now_err = Set_Speed - Now_Speed_Normal[ID_Num];
    if (abs(PID_Speed_Control_Num[ID_Num].now_err) < PID_Speed_Control_Num[ID_Num].deadband)
    {
      Current[ID_Num] = PID_Speed_Control_Num[ID_Num].output;
    }else{
      Current[ID_Num] = Speed_PID_Calculate(Set_Speed, ID_Num);
    }
}

void Current_Position_Control(uint8_t ID_Num, float Set_Angle)
{
	float Temp = Set_Angle;
	PID_Position_Control_Num[ID_Num].last_err = PID_Position_Control_Num[ID_Num].now_err;
	PID_Position_Control_Num[ID_Num].now_err = Temp - Now_Angle_Normal[ID_Num];
	uint16_t Speed = Position_PID_Calculate(Temp,ID_Num);
	if (abs(PID_Position_Control_Num[ID_Num].now_err) < PID_Position_Control_Num[ID_Num].deadband){
		//CAN_CMD_Voltage(ID_Num, PID_Position_Control_Num[ID_Num].output);
		Speed = 0;
	}
	Base_Speed_Calculation(ID_Num, Speed);
	CAN_CMD_Current(Current[1],Current[2],Current[3],Current[4]);
}

void Base_Control_Init()
{
    for (int i=1;i<=4;i++) M3508_Speed_Control_Init(i);
		CAN1_Filter_Init();
}

void Tuner_Control_Init()
{
	M3508_Speed_Control_Init(1);
	M3508_Position_Control_Init(1);
	CAN1_Filter_Init();
}