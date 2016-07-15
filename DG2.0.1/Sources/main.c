#include "includes.h"
int Flag=0,wait=9;
signed int steer=0;




void main(void)
 {
	initALL();
	
	while(wait>0);
	Set_Middlepoint();
	for (;;) 
	{
		Key_Detect_Compensator();
		if(Flag==1)
		{
			sensor_display();
			steer=STEER_HELM_CENTER+LocPIDCal();
			if(steer<=1516)
				steer=1470;
			if(steer>=1896)
				steer=1919;
		    SET_steer(steer);
			Dis_Num(64,3,(WORD)steer,5);
			SpeedSet();
			speed_control();
		}
		Flag=0;
		Senddata();
	}
}


void Pit0ISR()     
{
	Flag=1;
	frequency_measure();
	Get_speed();
	if(wait>0)
		wait--;
	PIT.CH[0].TFLG.B.TIF = 1;
}


