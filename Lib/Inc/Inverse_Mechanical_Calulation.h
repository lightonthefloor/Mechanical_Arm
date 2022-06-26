//
// Created by Trisoil on 2022/6/14.
//

#ifndef A1_DRIVE_A_INVERSE_MECHANICAL_CALULATION_H
#define A1_DRIVE_A_INVERSE_MECHANICAL_CALULATION_H

extern void Inverse_Theta_Calculation(double X,double Y);
extern double Theta1,Theta2,Theta3;
extern int Switch_State;
extern double Angle_R;
extern int Start_Turn;
extern float Now_X,Now_Y;

extern void Arm_Move(float Aim_X,float Aim_Y);

#endif //A1_DRIVE_A_INVERSE_MECHANICAL_CALULATION_H
