/*
 /*
 * functions.c
 *
 *  Created on: Feb 27, 2016
 *      Author: Administrator
 */

/*
 *  functions.c
 *
 *  Created on: Feb 20, 2016
 *      Author: Administrator
 */
#include"includes.h"
#define Hillcont 0
#define Frequency_Over 220
unsigned int chuwan,Hill_count;
float fre_diff,dis,LEFT_old,LEFT_new=0,RIGHT_old,RIGHT_new=0,MIDDLE_old,MIDDLE_new=0,temp_steer,temp_steer_old;
float LEFT_Temp,RIGHT_Temp,MIDDLE_Temp,Lsum,Rsum,Msum;
float sensor[3][10]={0},avr[10]={0.005,0.01,0.01,0.0125,0.0125,0.025,0.025,0.05,0.15,0.7};
unsigned int left,right,middle,flag=0,zd_flag=0,slow,pause=0; //车子在赛道的位置标志
unsigned int count1,count2,currentspeed,speed_target; 
unsigned int presteer,currentsteer,dsteer,Angle;
unsigned char Left_Compensator=35, Right_Compensator=36;
float Middle_Compensator=26;
unsigned int Uphill=0,Downhill=0,Up_Flag=0,Down_Flag=0,Straight,Ramp_Flag,Ramp_Time=0;
unsigned int 
             speed1=345,	
			 speed2=295,
			 speed3=255,
			 speed4=225,
			 speed5=205;

float
#define D1 38
#define D2 32
		kp1=9.3,ki2=0,kd1=D1,  
		kp2=5.25,ki3=0,kd2=D1,
		kp3=1.9,ki4=0,kd3=D2,
		kp4=0.9,ki=0,kd4=D2;


float kp,ki,kd;
int RIGHT,LEFT,MIDDLE,temp_fre[2];
unsigned char Outdata[8];
float sumerror,lasterror,Msetpoint=0,temp_middle=0,sensor_compensator=0,middleflag=0,dleft=0,dmiddle=0,dright=0,start_middle=0,start_left=0,start_right=0;
int Set_speed,temp_speed,pwm;
int speed_iError,speed_lastError,speed_prevError,Error[3];
float 
      speed_kp=0.6,
	  speed_ki=0.2,
	  speed_kd=0.1;


/****************************************************************************************************************
* 函数名称：frequency_measure()	
* 函数功能：获取电感频率
* 入口参数：无
* 出口参数：无
* 修改人  ：温泉
* 修改时间：2016/03/6
*****************************************************************************************************************/
void frequency_measure(void)
{
	unsigned int i,j;
	
	LEFT_new=(WORD)EMIOS_0.CH[24].CCNTR.R;
	if (LEFT_new >= LEFT_old)
		{
			LEFT_Temp = LEFT_new - LEFT_old;
		}
	else
		{
			LEFT_Temp = 0xffff - (-LEFT_new + LEFT_old);
		}
	LEFT_old=LEFT_new;
	
	
	RIGHT_new=(WORD)EMIOS_0.CH[23].CCNTR.R;
	if (RIGHT_new >= RIGHT_old)
		{
			RIGHT_Temp = RIGHT_new - RIGHT_old;
		}
	else
		{
			RIGHT_Temp = 0xffff - (-RIGHT_new + RIGHT_old);
		}
	RIGHT_old=RIGHT_new;
	

	MIDDLE_new=(WORD)EMIOS_0.CH[16].CCNTR.R;
	if (MIDDLE_new >= MIDDLE_old)
		{
			MIDDLE_Temp = MIDDLE_new - MIDDLE_old;
		}
		else
		{
			MIDDLE_Temp = 0xffff - (-MIDDLE_new + MIDDLE_old);
		}
	MIDDLE_old=MIDDLE_new;
	

	for(i=0;i<9;i++)
	{
		sensor[0][i]=sensor[0][i+1];
		sensor[1][i]=sensor[1][i+1];
		sensor[2][i]=sensor[2][i+1];
	}
	sensor[0][9]=LEFT_Temp;
	sensor[1][9]=MIDDLE_Temp;
	sensor[2][9]=RIGHT_Temp;
	Lsum=0;
	Msum=0;
	Rsum=0;
	for( j=0;j<10;j++)
	{
		Lsum+=sensor[0][j]*avr[j];
		Msum+=sensor[1][j]*avr[j];
		Rsum+=sensor[2][j]*avr[j];
	}
	LEFT=Lsum;
	MIDDLE=Msum;
	RIGHT=Rsum;
}

/****************************************************************************************************************
* 函数名称：InitsePID()	
* 函数功能：初始化舵机的PID参数
* 入口参数：无
* 出口参数：无
* 修改人  ：温泉
* 修改时间：2016/02/18
*****************************************************************************************************************/
unsigned int abs(signed int x)
{
	if(x>=0)
		return x;
	else 
		return -x;
}
/****************************************************************************************************************
* 函数名称：LocPIDCal()	
* 函数功能：计算舵机的PWM变化值
* 入口参数：无
* 出口参数：无
* 修改人  ：温泉
* 修改时间：2016/02/18
*****************************************************************************************************************/
signed int LocPIDCal(void)
{
	register float iError,dError;
	
	
		
	dleft=LEFT-start_left;
	if(dleft>33)
		dleft=33;
	if(MIDDLE>start_middle)
		dmiddle=0;
	else
		dmiddle=MIDDLE-start_middle;
	dright=RIGHT-start_right;
	if(dright>36)
		dright=36;
	
	if(flag==1)
	{
		if(dleft<8&&dmiddle<-22&&dright<8)
			return(190);
//		else if(dleft<6&&dmiddle<-25&&dright<6&&Up_Flag==1)
//			return(0);		
		else
		{
			flag=0;
			if(dleft>=dright)
			{
				if(dmiddle>=-Middle_Compensator)
					fre_diff=-dmiddle;
				else
					fre_diff=-dmiddle+Left_Compensator-dleft;
					
			}
			else
			{
				if(dmiddle>=-Middle_Compensator)
					fre_diff=dmiddle;
				else
					fre_diff=dmiddle-Right_Compensator+dright;
			}
		}
	}	
	else if(flag==2)
	{
		if(dright<8&&dmiddle<-22&&dleft<8)
			return(-190);
//	else if(dleft<6&&dmiddle<-25&&dright<6&&Up_Flag==1)
//		return(0);
		else
		{
			flag=0;
			if(dleft>=dright)
			{
				if(dmiddle>=-Middle_Compensator)
					fre_diff=-dmiddle;
				else
					fre_diff=-dmiddle+Left_Compensator-dleft;
					
			}
			else
			{
				if(dmiddle>=-Middle_Compensator)
					fre_diff=dmiddle;
				else
					fre_diff=dmiddle-Right_Compensator+dright;
			}
		}
	}
	else
	{
		flag=0;
		if(dleft>=dright)
		{
			if(dmiddle>=-Middle_Compensator)
				fre_diff=-dmiddle;
			else
				fre_diff=-dmiddle+Left_Compensator-dleft;
				
		}
		else
		{
			if(dmiddle>=-Middle_Compensator)
				fre_diff=dmiddle;
			else
				fre_diff=dmiddle-Right_Compensator+dright;
		}
		
	}
/*	if(dleft<4&&dmiddle<-33&&dright<4)
		return(temp_steer_old);*/
	if(fre_diff<0)
		fre_diff*=1;
	iError=fre_diff; 
	sumerror+=iError;
	dError=iError-lasterror;
	lasterror=iError;		
	if(fre_diff>=-10&&fre_diff<=10)      //直道
	{
		kp=kp4/10*abs(fre_diff);
		kd=kd4;
	}
	else if(fre_diff>=-20&&fre_diff<=20)      //直道
	{

		kp=kp4+(kp3-kp4)/10*(abs(fre_diff)-10);
		kd=kd3;
	}
	else if(fre_diff>=-35&&fre_diff<=35)                                //小弯
	{
		kp=kp3+(kp2-kp3)/15*(abs(fre_diff)-20);
		kd=kd2;
	}
		else                              //小弯
		{

			kp=kp2+(kp1-kp2)/25*(abs(fre_diff)-35);
			kd=kd1;
				
		}
		temp_steer=kp*iError+kd*dError;
		if(temp_steer>=190)
			flag=1;               //左打死
		else if(temp_steer<=-190)
			flag=2;
		else 
			flag=0;
		temp_steer_old=temp_steer;
		return(temp_steer);
	

}
/****************************************************************************************************************
* 函数名称：speed_control( )	
* 函数功能：速度控制
* 出口参数：无
* 修改人  ：温泉
* 修改时间：2016/03/16
*****************************************************************************************************************/
void speed_control()
{
	speed_iError=speed_target-currentspeed;
	Error[2]=Error[1];
	Error[1]=Error[0];
	Error[0]=speed_iError;
	
	
	
	temp_speed+=speed_kp*(Error[0]-Error[1])+speed_ki*Error[0]+speed_kd*(Error[0]-Error[1]-(Error[1]-Error[2]));
	if(temp_speed>135)
		temp_speed=135;
	if(temp_speed<-150)
			temp_speed=-150;
	SET_motor(temp_speed);
	if(forward)
		SET_motor(0);
}
/****************************************************************************************************************
* 函数名称：speed_set( )	
* 函数功能：速度设定
* 出口参数：无
* 修改人  ：温泉
* 修改时间：2016/03/16
*****************************************************************************************************************/

void SpeedSet(void)
{
	 if((temp_steer>=181||temp_steer<=-186))
	{	
			speed_target=speed5;
	}
	else if(temp_steer<30&&temp_steer>-30)  
	{
	    	zd_flag++;
	    	slow++;
	    	if(zd_flag>100)
	    	{
	    		speed_target = speed1;
	    		chuwan=0;
	    	}
	    	pause=0;
	    } 
    else if(temp_steer>-60 && temp_steer<60)
    {
        	if(zd_flag>300)
        	{
        		speed_target=speed4;
        		pause=1;
        	}
        	else if(zd_flag>200)
        	{
        		speed_target=speed5;
        		pause=1;
        	}
        	else
        		speed_target = speed2-(abs(temp_steer)-30)/30*(speed2-speed1);
        	zd_flag=0;
        	pause=0;
        } 
    else if(temp_steer>-100 && temp_steer<100)
    {
        	zd_flag=0; 
        	if(chuwan)
        		slow=0;
            speed_target = speed3-(abs(temp_steer)-60)/40*(speed3-speed2);
            pause=0;
     } 
    else if(temp_steer>=-140 && temp_steer<140)
    {
        	zd_flag=0;
        	if(chuwan)
        		slow=0;
            speed_target = speed4-(abs(temp_steer)-100)/40*(speed4-speed3);
            pause=0;
     }  
    else 
    {
        	zd_flag=0;
        	if(chuwan)
        		slow=0;
            speed_target = speed5-(abs(temp_steer)-140)/40*(speed5-speed4);
            pause=0;
     }  
    if(StopFlag==1)
    	speed_target=0;
}
/*******************************************************ADC*************************************************************/
unsigned int ADC_GetValueChannel()
{
   unsigned int Value=0;
   while(ADC.CDR[1].B.VALID != 1) {}         //等待ADC1转换结果有效
   Value = (unsigned int)ADC.CDR[1].B.CDATA;    //读取ADC1的转换结果
   return Value;
}

/*****************************************************采集陀螺仪值*********************************************************/
unsigned int Get_Angle()
{
	unsigned int Angle_t;
	Angle=0;
	for(Angle_t=0;Angle_t<10;Angle_t++)
	{
		Angle+=ADC_GetValueChannel()*avr[Angle_t];
		
	}
}

/****************************************************************************************************************
* 函数名称：sensor_display()	
* 函数功能：显示传感器值
* 入口参数：无
* 出口参数：无
* 修改人  ：温泉
* 修改时间：2016/02/18
*****************************************************************************************************************/
void sensor_display(void)
{
	Dis_Num(64,0,(WORD)LEFT,5);
	Dis_Num(64,1,(WORD)MIDDLE,5);
	Dis_Num(64,2,(WORD)RIGHT,5);
	Dis_Num(64,4,(WORD)currentspeed,5);
	Dis_Num(64,5,(WORD)fre_diff,5);
	Dis_Num(64,6,(WORD)-fre_diff,5);
	//Dis_Num(0,0,(WORD)Left_Compensator,5);
	//Dis_Num(0,2,(WORD)Right_Compensator,5);
	//Dis_Num(0,1,(WORD)Middle_Compensator,5);
	Dis_Num(0,0,(WORD)start_left,5);
	Dis_Num(0,2,(WORD)start_right,5);
	Dis_Num(0,1,(WORD)start_middle,5);
	Dis_Num(0,3,(WORD)Openloop_Speed,5);
	Dis_Num(0,4,(WORD)flag,5);
	Dis_Num(44,0,(WORD)Uphill,2);
	Dis_Num(44,1,(WORD)Up_Flag,2);
	Dis_Num(44,2,(WORD)Down_Flag,2);
}

/****************************************************************************************************************
* 函数名称：SAIC1_inter()	
* 函数功能：光编输入脉冲捕捉
* 入口参数：无
* 出口参数：无
* 修改人  ：温泉
* 修改时间：2016/02/23
*****************************************************************************************************************/
/*void SAIC1_inter(void) 
{

    if(  EMIOS_0.CH[3].CSR.B.FLAG  == 1)
	{
		count1++;
		EMIOS_0.CH[3].CSR.B.FLAG=1;    //清除标志位
	}
}*/
/****************************************************************************************************************
* 函数名称：SAIC2_inter()	
* 函数功能：光编输入脉冲捕捉
* 入口参数：无
* 出口参数：无
* 修改人  ：温泉
* 修改时间：2016/02/23
*****************************************************************************************************************/
/*void SAIC2_inter(void) 
{

    if(  EMIOS_0.CH[7].CSR.B.FLAG  == 1)
	{
		count2++;
		EMIOS_0.CH[7].CSR.B.FLAG=1;    //清除标志位
	}
}
*/

/****************************************************************************************************************
* 函数名称：Get_speed()	
* 函数功能：10msec中断获取速度
* 入口参数：无
* 出口参数：无
* 修改人  ：温泉
* 修改时间：2016/02/23
*****************************************************************************************************************/
void Get_speed()  //定时2mse采速度
{
	count1=(WORD)EMIOS_0.CH[3].CCNTR.R;
	if (count1 >= count2)
		{
			currentspeed = count1 - count2;
		}
	else
		{
			currentspeed = 0xffff - (-count1 + count2);
		}
	if(forward)
		currentspeed=-currentspeed;
	count2=count1;
	//PIT.CH[1].TFLG.B.TIF=1;*/
}

/****************************************************************************************************************
* 函数名称：Set_Middlepoint()	
* 函数功能：中间线圈频率标定
* 入口参数：无
* 出口参数：无
* 修改人  ：温泉
* 修改时间：2016/02/23
*****************************************************************************************************************/
void Set_Middlepoint()
{
	start_middle=MIDDLE+23;
	start_left=LEFT+19;
	start_right=RIGHT+17;
	sensor_compensator=RIGHT-LEFT;
//	Msetpoint=temp_middle;
//	Dis_Num(64,6,(WORD)Msetpoint,5);
}



void SendHex(unsigned char hex) 
{
    unsigned char temp;
    temp = hex & 0x0F;
    if(temp < 10) 
    {
    	LINFlex_TX(temp + '0');
     }   
    else 
    {
    	LINFlex_TX(temp - 10 + 'A');
     }
     temp = hex >> 4;
     if(temp < 10) 
     {
    	 LINFlex_TX(temp + '0');
      } 
     else 
     {
    	 LINFlex_TX(temp - 10 + 'A');
      }
}

void Senddata()
{	unsigned int i;
	Outdata[0]=(unsigned char)LEFT;
	Outdata[1]=(unsigned char)RIGHT;
	Outdata[2]=(unsigned char)speed_target ;
	Outdata[3]=(unsigned char)currentspeed;
	Outdata[4]=(unsigned char)(LEFT>>8);
	Outdata[5]=(unsigned char)(RIGHT>>8);
	Outdata[6]=(unsigned char)(speed_target >>8);
	Outdata[7]=(unsigned char)(currentspeed>>8);
	LINFlex_TX('=');
	LINFlex_TX('=');
	for(i=0;i<8;i++)
	{
		SendHex(Outdata[i]);
		if(i==3)
		{
			LINFlex_TX('f');
			LINFlex_TX('f');
		}
	}
}


/***********************************************按键补偿*******************************************************/
void Key_Detect_Compensator()
{
	if(switch1==0&&switch2==0)
	{
		if(Key1==0)
		{
			delay();
		if(Key1==0)
			Middle_Compensator--;
		while(Key1==0)
			sensor_display();
		}
		if(Key2==0)
		{
			delay();
			if(Key2==0)
				Middle_Compensator++;
			while(Key2==0)
				sensor_display();
		}
	}
	if(switch1==0&&switch2)
	{
		if(Key1==0)
		{
			delay();
		if(Key1==0)
			Left_Compensator--;
		while(Key1==0)
			sensor_display();
		}
		if(Key2==0)
		{
			delay();
			if(Key2==0)
				Left_Compensator++;
			while(Key2==0)
				sensor_display();
		}
	}
	if(switch1&&switch2==0)
	{
		if(Key1==0)
		{
			delay();
		if(Key1==0)
			Right_Compensator--;
		while(Key1==0)
			sensor_display();
		}
		if(Key2==0)
		{
			delay();
			if(Key2==0)
				Right_Compensator++;
			while(Key2==0)
				sensor_display();
		}
	}
	if(switch3==0)
	{
		if(Key1==0)
		{
			delay();
		if(Key1==0)
			Openloop_Speed--;
		while(Key1==0)
			sensor_display();
		}
		if(Key2==0)
		{
			delay();
			if(Key2==0)
				Openloop_Speed++;
			while(Key2==0)
				sensor_display();
		}
	}
}


/*************************************************void delay***************************************/
void delay()
{
	unsigned int delay=1000,t;
	for(t=100;t>0;t--)
		delay--;
}
void Ramp_Detect()
{
	if(Up_Flag==0	&&	Down_Flag==0	&&	RIGHT-start_right+dleft+MIDDLE-start_middle>Frequency_Over)
		{
			Hill_count++;
			if(Hill_count>Hillcont)
			{
				Hill_count=0;
				Up_Flag=1;
			}
		}
		if(Up_Flag==1	&&	Down_Flag==0	&&	RIGHT-start_right+dleft+MIDDLE-start_middle<(Frequency_Over-100))
			{
			Uphill=1;
			Ramp_Flag=1;
			}
		if(Uphill==1	&&	RIGHT-start_right+dleft+MIDDLE-start_middle>Frequency_Over)
		{
			Hill_count++;
			if(Hill_count>Hillcont)
			{
				Hill_count=0; 
				Uphill=2;
				Down_Flag=1;
				Up_Flag=2;
			}
		}
		if(Up_Flag==0&&Down_Flag==1&&RIGHT-start_right+dleft+MIDDLE-start_middle<(Frequency_Over-100))
		{
			Down_Flag=2;
		}
}
