//
// Created by Trisoil on 2022/6/15.
//

#ifndef M6020_DRIVE_M6020_PID_CONTROLLER_H
#define M6020_DRIVE_M6020_PID_CONTROLLER_H

#include "main.h"

struct PID_Number{
		float kp;
		float ki;
		float kd;
		float kp_output;
		float ki_output;
		float kd_output;
		float output;
		int err_sum;
		int now_err;
		int last_err;
		int deadband;
		int max_output;
		int integral_limit;
		int max_err;
};

extern void M6020_Speed_Control_Init(uint8_t ID_Num);
extern void Voltage_Speed_Control(uint8_t ID_Num, float Set_Speed);
extern void Voltage_Position_Control(uint8_t ID_Num, float Set_Angle);
extern void M6020_Control_Init();

#endif //M6020_DRIVE_M6020_PID_CONTROLLER_H
