//
// Created by Trisoil on 2022/6/14.
//

#include "Inverse_Mechanical_Calulation.h"
#include "A1_Drive.h"
#include "servo.h"
#include <math.h>

#define PI 3.1415926535f

double A,B;
double L1,L2,L3,L4;
double Theta1,Theta2,Theta3;
double Angle_R;
int Switch_State;
int Start_Turn = 0;
float Now_X,Now_Y;

void Inverse_Theta_Calculation(double X,double Y)
{
	L1 = 504.11f;
	L2 = 262.0f;
	L3 = 248.75f;
	L4 = 207.77f;
	A = X - L4;
	B = Y - L1;
	Theta3 = asin(((A*A + B*B)+(L3*L3 - L2*L2))/(2.0f*L3*sqrt(A*A + B*B))) - asin(A/sqrt(A*A + B*B));
	Theta1 = asin(((A*A + B*B)+(L2*L2 - L3*L3))/(2.0f*L2*sqrt(A*A + B*B))) - asin(B/sqrt(A*A + B*B));
	Theta2 = (PI/2.0f) - (Theta1 + Theta3);
}

void Arm_Move(float Aim_X,float Aim_Y){
	float Delta_X = (Aim_X-Now_X)/100.0f;
	float Delta_Y = (Aim_Y-Now_Y)/100.0f;
	double x = Now_X;
	double y = Now_Y;
	for (int i=1;i<=100;i++)
	{
		x += Delta_X;
		y += Delta_Y;
		Inverse_Theta_Calculation(x,y);
		A1_Motor_Position_Control(0,Theta1+Set_Zero[0]);
		HAL_Delay(5);
		A1_Motor_Position_Control(1,Theta2+Set_Zero[1]);

		Servo_Goal_Position(1,(uint32_t)((180.0f/PI)*Theta3));
	}
}



