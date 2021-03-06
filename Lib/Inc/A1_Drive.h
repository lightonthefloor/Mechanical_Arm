//
// Created by Trisoil on 2022/5/3.
//

#include "main.h"

#ifndef A1_DRIVE_A1_DRIVE_H
#define A1_DRIVE_A1_DRIVE_H

typedef struct{
		uint8_t start[2];
		uint8_t Motor_ID;
		uint8_t reserved_a;
		uint8_t mode;
		uint8_t ModifyBit;
		uint8_t ReadBit;
		uint8_t reserved_b;
		uint32_t Modify;
		uint16_t T;
		int16_t W;
		int32_t Pos;
		uint16_t kp;
		uint16_t kw;
}Send_Data;

typedef struct{
		uint8_t mode;
		uint8_t Temp;
		uint16_t T;
		uint16_t W;
		uint16_t Acc;
		int32_t Pos;
		float Pos2;
}Rx_Data;

typedef struct{
		uint8_t Mode;
		uint8_t Temp;
		float Torque;
		float Omega;
		uint16_t Acc;
		float Position;
		float Position2;
}Motor_State;

extern Motor_State A1_State[3];
extern float Set_Zero[3];
extern int A1_Start_Move_A;
extern int A1_Start_Move_B;

extern void Mode_Control(int ID,int Mode);
extern void Usart6_TX_DMA_Init(void);
extern void A1_Motor_Multiple_Control(int ID,int mode,float Torque, float W,float Position);
extern void A1_Motor_Speed_Control(int ID,float W);
extern void A1_Motor_Position_Control(int ID,float Position);
extern void A1_Motor_Speed_Position_Control(int ID,float W,float Position);
extern void USART6_Rx_Init();
extern void Usart_Printf(const char *fmt,...);

#endif //A1_DRIVE_A1_DRIVE_H
