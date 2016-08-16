#include "includes.h"
unsigned int Flag=0,wait=9,INTC_Time=0,mode_flag=0,Limit_Flag=0;
signed int steer=0,delay_count=0,StartDelay=0;
void FastSpeedMode();
void MiddleSpeedMode();
void SlowSpeedMode();
void openloopMode();

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
	while(switch6==0&&switch5==0)
		openloopMode();
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
			if(Ramp_Time>70&&Ramp_Time<120)
			{
				if(steer<=STEER_HELM_CENTER-30)
					steer=STEER_HELM_CENTER-30;
				if(steer>=STEER_HELM_CENTER+30)
					steer=STEER_HELM_CENTER+30;
				SET_steer(steer);
				Limit_Flag=1;
			}
			if(Ramp_Time>83)
				Up_Flag=2;
			if(Ramp_Flag>100)
				Ramp_Flag=0;
			if(Ramp_Time>=120)
				Limit_Flag=0;
			if(Ramp_Time>200)
			{
				Up_Flag=0;
				Ramp_Flag=0;
				Uphill=0;
				Down_Flag=0;
				Ramp_Time=0;
			}
		}
		if(switch3==0)
			StartDelay++;
		if(StartDelay>301)
			StartDelay=301;
		INTC_Time=0;
	}
	Get_speed();
	if(mode_flag==0)
		speed_control();
	PIT.CH[0].TFLG.B.TIF = 1;
}
void openloopMode()
{
#define D1 38
#define D2 35
	kp1=9.3,kd1=D1,
	kp2=5.3,kd2=D1,
	kp3=2.4,kd3=D2,
	kp4=1.5,kd4=D2;
	mode_flag=1;
	for (;;) 
		{
			Key_Detect_Compensator();
			if(Flag==1)
			{
				sensor_display();				
				steer=STEER_HELM_CENTER+LocPIDCal();
				if(steer<=STEER_HELM_CENTER-230)
					steer=STEER_HELM_CENTER-233;
				if(steer>=STEER_HELM_CENTER+230)
					steer=STEER_HELM_CENTER+233;				
				if(Up_Flag==1)
					steer=STEER_HELM_CENTER+2;
				if(Limit_Flag==0)
					SET_steer(steer);			
				Dis_Num(64,3,(WORD)steer,4);
				StopLineDetect();
				if(StopFlag)
					EMIOS_0.CH[9].CBDR.R=0;
				if(StartDelay>300)
				{
					if(Up_Flag==1)
						EMIOS_0.CH[9].CBDR.R = Openloop_Speed-6;
					else
						EMIOS_0.CH[9].CBDR.R = Openloop_Speed;
						
				}
	//			SpeedSet();
			}
			Flag=0;
			Senddata();
		}
	
}
void FastSpeedMode()
{
float	kp1=8.5,kd1=30,
		kp2=5.05,kd2=30,
		kp3=2.3,kd3=35,
		kp4=1.2,kd4=35;
	speed1=76;
	speed5=38;
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
				steer=STEER_HELM_CENTER+233;
			Dis_Num(64,3,(WORD)steer,4);
			if(Up_Flag==1)
				steer=STEER_HELM_CENTER;
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
float	kp1=8.5,kd1=30,
		kp2=5,kd2=30,
		kp3=2.3,kd3=35,
		kp4=1.2,kd4=35;
	speed1=72;
	speed5=38;
	for (;;) 
	{
		Key_Detect_Compensator();
		if(Flag==1)
		{
			sensor_display();
			steer=STEER_HELM_CENTER+LocPIDCal();
			if(steer<=STEER_HELM_CENTER-230)
				steer=STEER_HELM_CENTER-233;
			if(steer>=STEER_HELM_CENTER+230)
				steer=STEER_HELM_CENTER+233;
			Dis_Num(64,3,(WORD)steer,4);
			if(Up_Flag==1)
				steer=STEER_HELM_CENTER;
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
	float	kp1=8.2,kd1=30,
			kp2=5,kd2=30,
			kp3=2.3,kd3=35,
			kp4=1.2,kd4=35;
	speed1=69;
	speed5=38;
	for (;;) 
	{
		Key_Detect_Compensator();
		if(Flag==1)
		{
			sensor_display();
			steer=STEER_HELM_CENTER+LocPIDCal();
			if(steer<=STEER_HELM_CENTER-230)
				steer=STEER_HELM_CENTER-233;
			if(steer>=STEER_HELM_CENTER+230)
				steer=STEER_HELM_CENTER+233;
			Dis_Num(64,3,(WORD)steer,4);
			if(Up_Flag==1)
				steer=STEER_HELM_CENTER;
			SET_steer(steer);
			StopLineDetect();
			if(StartDelay>300)
				SpeedSet();
		}
		Flag=0;
		Senddata();
	}
}
