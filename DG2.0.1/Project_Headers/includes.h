/*
 * includes.h
 *
 *  Created on: Feb 27, 2016
 *      Author: Administrator
 */

#include "MPC5604B.h"
#include "init.h" 
#include "IntcInterrupts.h"
//#include "lcddriver.h"
#include "define.h"
#include "functions.h"
#include "oleddriver.h"
#include "ReedSwitch.h"




#define byte unsigned char
#define word unsigned int


/***************foreward*****************/
#define forward SIU.GPDI[44].R	//光编检测车实际方向:1前进 0倒退


/***************switchs*****************/
#define switch1 SIU.GPDI[56].R
#define switch2 SIU.GPDI[54].R
#define switch3 SIU.GPDI[52].R
#define switch4 SIU.GPDI[50].R
#define switch5 SIU.GPDI[48].R
#define switch6 SIU.GPDI[24].R

#define Key2 SIU.GPDI[57].R
#define Key1 SIU.GPDI[22].R



/***************变量定义*****************/
#define STEER_HELM_CENTER 1264//1706//855//862//853/861.5/

