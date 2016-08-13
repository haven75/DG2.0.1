#include "includes.h"
unsigned int Flag=0,wait=9,INTC_Time=0;
signed int steer=0,delay_count=0,StartDelay=0;
void FastSpeedMode();
void MiddleSpeedMode();
void SlowSpeedMode();

void main(void)
 {
	initALL();
	while(wait>0);
	Set_Middlepoint();
	while(switch6==1&&switch5==1)
		FastSpeedMode();
	while(switch6==0&&switch5==1)
		MiddleSpeedMode();
	while(switch6==1&&switch5==0)
		SlowSpeedMode();
}


void Pit0ISR()     
{
	INTC_Time++;
	if(INTC_Time==5)
	{
		Flag=1;
		frequency_measure();
		
		if(wait>0)
			wait--;
		if(1)
		{
			if(delay_count<400)
				delay_count++;
			else Ramp_Detect();
			if(Ramp_Flag==1)
				Ramp_Time++;
			if(Ramp_Time>85)
				Up_Flag=2;
			if(Ramp_Flag>100)
				Ramp_Flag=0;
		}
		if(switch2==0)
			StartDelay++;
		if(StartDelay>301)
			StartDelay=301;
		INTC_Time=0;
	}
	Get_speed();
	speed_control();
	PIT.CH[0].TFLG.B.TIF = 1;
}

void FastSpeedMode()
{
	speed1=70;
	speed5=39;
	for (;;) 
	{
		Key_Detect_Compensator();
		if(Flag==1)
		{
			sensor_display();
			steer=STEER_HELM_CENTER+LocPIDCal();
			if(steer<=STEER_HELM_CENTER-230)
				steer=STEER_HELM_CENTER-235;
			if(steer>=STEER_HELM_CENTER+230)
				steer=STEER_HELM_CENTER+213;
			Dis_Num(64,3,(WORD)steer,4);
			if(Up_Flag==1)
				steer=STEER_HELM_CENTER+2;
			SET_steer(steer);
			StopLineDetect();
			if(StartDelay>300)
				SpeedSet();
		}
		Flag=0;
		Senddata();
	}
}
                   
void MiddleSpeedMode()
{
	speed1=68;
	speed5=38;
	for (;;) 
	{
		Key_Detect_Compensator();
		if(Flag==1)
		{
			sensor_display();
			steer=STEER_HELM_CENTER+LocPIDCal();
			if(steer<=STEER_HELM_CENTER-210)
				steer=STEER_HELM_CENTER-235;
			if(steer>=STEER_HELM_CENTER+210)
				steer=STEER_HELM_CENTER+213;
			Dis_Num(64,3,(WORD)steer,4);
			if(Up_Flag==1)
				steer=STEER_HELM_CENTER+2;
			SET_steer(steer);
			StopLineDetect();
			if(StartDelay>300)
				SpeedSet();
		}
		Flag=0;
		Senddata();
	}
}

void SlowSpeedMode()
{
	speed1=66;
	speed5=37;
	for (;;) 
	{
		Key_Detect_Compensator();
		if(Flag==1)
		{
			sensor_display();
			steer=STEER_HELM_CENTER+LocPIDCal();
			if(steer<=STEER_HELM_CENTER-210)
				steer=STEER_HELM_CENTER-235;
			if(steer>=STEER_HELM_CENTER+210)
				steer=STEER_HELM_CENTER+213;
			Dis_Num(64,3,(WORD)steer,4);
			if(Up_Flag==1)
				steer=STEER_HELM_CENTER+2;
			SET_steer(steer);
			StopLineDetect();
			if(StartDelay>300)
				SpeedSet();
		}
		Flag=0;
		Senddata();
	}
}
