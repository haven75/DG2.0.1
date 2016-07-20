/*
 * functions.h
 *
 *  Created on: Feb 27, 2016
 *      Author: Administrator
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

void frequency_measure();
void InitspPID();
void InitsePID();
signed int LocPIDCal();
void sensor_display();
void Get_speed();

void SAIC1_inter(void);
//void SAIC2_inter(void);
void Set_Middlepoint();
void SendHex(unsigned char hex);
void Senddata();
void SpeedSet();
void speed_control();
void Key_Detect_Compensator();
signed int Steer();
void delay();
void Ramp_Detect();
void StopLineDetect();
extern unsigned char Left_Compensator, Right_Compensator;
extern float Middle_Compensator;
extern unsigned int Uphill,Downhill,Up_Flag,Down_Flag,Ramp_Flag,Ramp_Time,
speed1,speed2,speed3,speed4,speed5;
extern unsigned char StartFlag,StopFlag,RunFlag,Stop;
extern float kp1,kp2,kp3,kp4,kd1,kd2,kd3,kd4;
#endif /* FUNCTIONS_H_ */
