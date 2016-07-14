/*
 * ReedSwitch.h
 *
 *  Created on: Jun 4, 2016
 *      Author: Quan
 */

#ifndef REEDSWITCH_H_
#define REEDSWITCH_H_


#define ReedSwitch1 SIU.GPDI[63].R
#define ReedSwitch2 SIU.GPDI[15].R


void StartDetect(void);
extern unsigned char StartFlag,StopFlag;

#endif /* REEDSWITCH_H_ */
