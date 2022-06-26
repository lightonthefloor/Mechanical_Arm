//
// Created by Trisoil on 2021/12/14.
//


#include <stdlib.h>
#include "M6020_PID_Controller.h"
#include "CAN_Operation.h"
#include "gpio.h"
#include "main.h"
#include "can.h"
#include "math.h"

struct PID_Number PID_Speed_Num_M6020[5];
struct PID_Number PID_Position_Num_M6020[5];

void M6020_Speed_Control_Init(uint8_t ID_Num){
	PID_Speed_Num_M6020[ID_Num].kp = 18.0f;
	PID_Speed_Num_M6020[ID_Num].ki = 30.0f;
	PID_Speed_Num_M6020[ID_Num].kd = 0.5f;
	PID_Speed_Num_M6020[ID_Num].max_output  = 30000;
	PID_Speed_Num_M6020[ID_Num].integral_limit = 10000;
	PID_Speed_Num_M6020[ID_Num].deadband = 10;
	PID_Speed_Num_M6020[ID_Num].max_err = 8000;
}

void M6020_Position_Control_Init(uint8_t ID_Num){
	PID_Position_Num_M6020[ID_Num].kp = 0.7f;
	PID_Position_Num_M6020[ID_Num].ki = 0;
	PID_Position_Num_M6020[ID_Num].kd = 0.6f;
	PID_Position_Num_M6020[ID_Num].max_output  = 30000;
	PID_Position_Num_M6020[ID_Num].integral_limit = 10000;
	PID_Position_Num_M6020[ID_Num].deadband = 20;
	PID_Position_Num_M6020[ID_Num].max_err = 8000;
}

int16_t PID_Speed_Calculate(uint8_t ID_Num){
	PID_Speed_Num_M6020[ID_Num].ki_output += PID_Speed_Num_M6020[ID_Num].ki * PID_Speed_Num_M6020[ID_Num].now_err;
	if (PID_Speed_Num_M6020[ID_Num].ki_output > PID_Speed_Num_M6020[ID_Num].integral_limit) {
		PID_Speed_Num_M6020[ID_Num].ki_output = PID_Speed_Num_M6020[ID_Num].integral_limit;
	}else if (PID_Speed_Num_M6020[ID_Num].ki_output < -PID_Speed_Num_M6020[ID_Num].integral_limit){
		PID_Speed_Num_M6020[ID_Num].ki_output = -PID_Speed_Num_M6020[ID_Num].integral_limit;
	}
	PID_Speed_Num_M6020[ID_Num].kp_output = PID_Speed_Num_M6020[ID_Num].kp * PID_Speed_Num_M6020[ID_Num].now_err;
	PID_Speed_Num_M6020[ID_Num].kd_output = PID_Speed_Num_M6020[ID_Num].kd * (PID_Speed_Num_M6020[ID_Num].now_err - PID_Speed_Num_M6020[ID_Num].last_err);
	PID_Speed_Num_M6020[ID_Num].output = PID_Speed_Num_M6020[ID_Num].ki_output + PID_Speed_Num_M6020[ID_Num].kp_output + PID_Speed_Num_M6020[ID_Num].kd_output;
	if (PID_Speed_Num_M6020[ID_Num].output > PID_Speed_Num_M6020[ID_Num].max_output){
		PID_Speed_Num_M6020[ID_Num].output = PID_Speed_Num_M6020[ID_Num].max_output;
	}else if (PID_Speed_Num_M6020[ID_Num].output < -PID_Speed_Num_M6020[ID_Num].max_output){
		PID_Speed_Num_M6020[ID_Num].output = -PID_Speed_Num_M6020[ID_Num].max_output;
	}
	return PID_Speed_Num_M6020[ID_Num].output;
}

void Voltage_Speed_Control(uint8_t ID_Num, float Set_Speed)
{
	PID_Speed_Num_M6020[ID_Num].last_err = PID_Speed_Num_M6020[ID_Num].now_err;
	PID_Speed_Num_M6020[ID_Num].now_err = Set_Speed - Now_Speed_M6020[ID_Num];
	if (PID_Speed_Num_M6020[ID_Num].now_err < PID_Speed_Num_M6020[ID_Num].deadband){
		CAN_CMD_Voltage(ID_Num, PID_Speed_Num_M6020[ID_Num].output);
	}
	int16_t Voltage = PID_Speed_Calculate(ID_Num);
	CAN_CMD_Voltage(ID_Num, Voltage);
}

void M6020_Control_Init()
{
	for (int i=1;i<=4;i++){
		M6020_Speed_Control_Init(i);
		M6020_Position_Control_Init(i);
	}
}

int16_t PID_Position_Calculate(uint8_t ID_Num){
	PID_Position_Num_M6020[ID_Num].ki_output += PID_Position_Num_M6020[ID_Num].ki * PID_Position_Num_M6020[ID_Num].now_err;
	if (PID_Position_Num_M6020[ID_Num].ki_output > PID_Position_Num_M6020[ID_Num].integral_limit) {
		PID_Position_Num_M6020[ID_Num].ki_output = PID_Position_Num_M6020[ID_Num].integral_limit;
	}else if (PID_Position_Num_M6020[ID_Num].ki_output < -PID_Position_Num_M6020[ID_Num].integral_limit){
		PID_Position_Num_M6020[ID_Num].ki_output = -PID_Position_Num_M6020[ID_Num].integral_limit;
	}
	PID_Position_Num_M6020[ID_Num].kp_output = PID_Position_Num_M6020[ID_Num].kp * PID_Position_Num_M6020[ID_Num].now_err;
	PID_Position_Num_M6020[ID_Num].kd_output = PID_Position_Num_M6020[ID_Num].kd * (PID_Position_Num_M6020[ID_Num].now_err - PID_Position_Num_M6020[ID_Num].last_err);
	PID_Position_Num_M6020[ID_Num].output = PID_Position_Num_M6020[ID_Num].ki_output + PID_Position_Num_M6020[ID_Num].kp_output + PID_Position_Num_M6020[ID_Num].kd_output;
	if (PID_Position_Num_M6020[ID_Num].output > PID_Position_Num_M6020[ID_Num].max_output){
		PID_Position_Num_M6020[ID_Num].output = PID_Position_Num_M6020[ID_Num].max_output;
	}else if (PID_Position_Num_M6020[ID_Num].output < -PID_Position_Num_M6020[ID_Num].max_output){
		PID_Position_Num_M6020[ID_Num].output = -PID_Position_Num_M6020[ID_Num].max_output;
	}
	return PID_Position_Num_M6020[ID_Num].output;
}

void Voltage_Position_Control(uint8_t ID_Num, float Set_Angle)
{
	PID_Position_Num_M6020[ID_Num].last_err = PID_Position_Num_M6020[ID_Num].now_err;
	PID_Position_Num_M6020[ID_Num].now_err = Set_Angle - Now_Angle_M6020[ID_Num];
	int16_t Speed = PID_Position_Calculate(ID_Num);
	if (abs(PID_Position_Num_M6020[ID_Num].now_err) < PID_Position_Num_M6020[ID_Num].deadband){
		//CAN_CMD_Voltage(ID_Num, PID_Position_Num_M6020[ID_Num].output);
		Speed = 0;
	}
	Voltage_Speed_Control(ID_Num, Speed);
}