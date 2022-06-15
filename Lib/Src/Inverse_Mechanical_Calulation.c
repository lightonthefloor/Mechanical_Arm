//
// Created by Trisoil on 2022/6/14.
//

#include "Inverse_Mechanical_Calulation.h"
#include <math.h>

#define PI 3.1415926535f

double A,B;
double L1,L2,L3,L4;
double Theta1,Theta2,Theta3;
double Angle_R;
int Switch_State;
int Start_Turn = 0;

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





