#include "includes.h"
unsigned int Flag=0,wait=9;
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
	Flag=1;
	frequency_measure();
	Get_speed();
	if(wait>0)
		wait--;
	if(switch4==0)
	{
		if(delay_count<400)
			delay_count++;
		else Ramp_Detect();
		if(Ramp_Flag==1)
			Ramp_Time++;
		if(Ramp_Time>60)
		{
			Up_Flag=2;
			Ramp_Flag=0;
		}
	}
	if(switch2==0)
		StartDelay++;
	if(StartDelay>301)
		StartDelay=301;

	PIT.CH[0].TFLG.B.TIF = 1;
}

void FastSpeedMode()
{
	/*speed1=430;
	speed5=200;*/
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
				steer=STEER_HELM_CENTER;
			SET_steer(steer);
			StopLineDetect();
			if(StartDelay>300)
				SpeedSet();
			speed_control();
		}
		Flag=0;
		Senddata();
	}
}
                   
void MiddleSpeedMode()
{
	speed1=410;
	speed5=200;
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
				steer=STEER_HELM_CENTER;
			SET_steer(steer);
			StopLineDetect();
			if(StartDelay>300)
				SpeedSet();
			speed_control();
		}
		Flag=0;
		Senddata();
	}
}

void SlowSpeedMode()
{
	speed1=400;
	speed5=195;
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
				steer=STEER_HELM_CENTER;
			SET_steer(steer);
			StopLineDetect();
			if(StartDelay>300)
				SpeedSet();
			speed_control();
		}
		Flag=0;
		Senddata();
	}
}
