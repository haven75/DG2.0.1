/*
 * ReedSwitch.c
 *
 *  Created on: Jun 4, 2016
 *      Author: Quan
 */


#include "includes.h"
unsigned char StopFlag=0;

void StartDetect(void)
{
	if(ReedSwitch1==0||ReedSwitch2==0)
		StopFlag=1;
}
