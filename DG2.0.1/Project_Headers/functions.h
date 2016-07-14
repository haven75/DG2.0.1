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
extern unsigned char Left_Compensator, Right_Compensator;
extern float Middle_Compensator;
#endif /* FUNCTIONS_H_ */
