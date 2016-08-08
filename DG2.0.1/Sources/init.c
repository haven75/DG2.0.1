/*
 * init.c
 *
 *  Created on: Feb 27, 2016
 *      Author: Administrator
 */


#include"includes.h"

unsigned int Openloop_Speed=100;
void initALL(void)
{
	disableWatchdog();
	initModesAndClock();
	initEMIOS_0MotorAndSteer();
	initEMIOS_0ModulusCounter();
	initLINFlex_0_UART();
	initOLED();
	initKeys_Switchs();
	initPIT();
	initADC();
	//SAIC_INIT();
	LCD_Init();
	
	enableIrq();
}
//*			 *************************sysclk初始化*******************************************************    	  *
void initModesAndClock(void)
{
    ME.MER.R = 0x0000001D;	/* Enable DRUN, RUN0, SAFE, RESET modes */
	/* 设置sysclk */
    CGM.FMPLL_CR.R = 0x02400100;	/* 8 MHz xtal: Set PLL0 to 64 MHz */
    //CGM.FMPLL_CR.R = 0x01280000;	/* 8 MHz xtal: Set PLL0 to 80 MHz */
    //CGM.FMPLL_CR.R = 0x013C0000;	/* 8 MHz xtal: Set PLL0 to 120 MHz */ 
    ME.RUN[0].R = 0x001F0064;	/* RUN0 cfg: 16MHzIRCON,OSC0ON,PLL0ON,syclk=PLL     sysclk选择锁相环时钟 */
    ME.RUNPC[0].R = 0x00000010;	/* Peri. Cfg. 1 settings: only run in RUN0 mode      选择RUN0模式 */
  
	/* PCTL[?] 选择需要时钟模块(默认即可，不用设置) */
	//ME.PCTL[32].R = 0x00;	/* MPC56xxB/P/S ADC 0: select ME.RUNPC[0] */
    ME.PCTL[32].B.DBG_F = 0;
	/* Mode Transition to enter RUN0 mode */
    ME.MCTL.R = 0x40005AF0;	/* Enter RUN0 Mode & Key */
    ME.MCTL.R = 0x4000A50F;	/* Enter RUN0 Mode & Inverted Key */
    
    while (ME.GS.B.S_MTRANS) {}	/* Wait for mode transition to complete 等待模式转换完成 */
    while(ME.GS.B.S_CURRENTMODE != 4) {} /* Verify RUN0 is the current mode 等待选择RUN0模式 */
  
	/* 开peri0、1、2 */
	/* 外设时钟总线 可用于分频 */
	CGM.SC_DC[0].R = 0x80;	/* LIN */
	CGM.SC_DC[1].R = 0x80;	/* FLEXCAN,DSPI */
    CGM.SC_DC[2].R = 0x80;	/* eMIOS,CTU,ADC */
}

void disableWatchdog(void)
{
	SWT.SR.R = 0x0000c520; /* Write keys to clear soft lock bit */
	SWT.SR.R = 0x0000d928;
	SWT.CR.R = 0x8000010A; /* Clear watchdog enable (WEN) */
}

//*****************************************************************************************************************
//*************************	PWM初始化:电机正反转E5、E6；舵机输出A4   **********************************************
//*****************************************************************************************************************
void initEMIOS_0MotorAndSteer(void)
{
  //eMIOS0初始化80MHz分为1MHz
	EMIOS_0.MCR.B.GPRE= 63;   //GPRE+1=分频系数；/* Divide 80 MHz sysclk by 79+1 = 80 for 1MHz(0.1us) eMIOS clk*/
	EMIOS_0.MCR.B.GPREN = 1;	/* Enable eMIOS clock */
	EMIOS_0.MCR.B.GTBE = 1;   /* Enable global time base */
	EMIOS_0.MCR.B.FRZ = 1;    /* Enable stopping channels when in debug mode */
	
  /**********电机PWM 5kHZ A9 A10口*********************************************************************************/ 
 /* Modulus Up Counter 5kHZ */
	EMIOS_0.CH[8].CCR.B.UCPRE=0;	/* Set channel prescaler to divide by 1+1 */
	EMIOS_0.CH[8].CCR.B.UCPEN = 1;	/* Enable prescaler; uses default divide by 1 */
	EMIOS_0.CH[8].CCR.B.FREN = 1;	/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[8].CADR.R = 200;	/* 设置周期200us 5KHZ */
	EMIOS_0.CH[8].CCR.B.MODE = 0x50;	/* Modulus Counter Buffered (MCB) */
	EMIOS_0.CH[8].CCR.B.BSL = 0x3;	/* Use internal counter */
/* 前进输出 OPWMB PA9 输出0-2000 */
	EMIOS_0.CH[9].CCR.B.BSL = 0x1;	/* Use counter bus D (default) */
	EMIOS_0.CH[9].CCR.B.MODE = 0x60;	/* Mode is OPWM Buffered */
	EMIOS_0.CH[9].CCR.B.EDPOL = 1;	/* Polarity-leading edge sets output/trailing clears*/
	EMIOS_0.CH[9].CADR.R = 1;	/* Leading edge when channel counter bus= */
	EMIOS_0.CH[9].CBDR.R = Openloop_Speed;	/* Trailing edge when channel counter bus= */
	
	SIU.PCR[9].R = 0x0600;	/*[11:10]选择AFx 此处AF1 /* MPC56xxS: Assign EMIOS_0 ch 21 to pad */
	
/* 后退输出 OPWMB PA10 输出0-2000 */
	EMIOS_0.CH[10].CCR.B.BSL = 0x1;
	EMIOS_0.CH[10].CCR.B.MODE = 0x60;
	EMIOS_0.CH[10].CCR.B.EDPOL = 1;
	EMIOS_0.CH[10].CADR.R = 1;
	EMIOS_0.CH[10].CBDR.R = 0;
	
	SIU.PCR[10].R = 0x0600;
	
	
  /**********舵机PWM 50HZ A2口输出         20000*7.5%=     中位**********/
    /* Modulus Up Counter 50HZ */
    EMIOS_0.CH[0].CCR.B.UCPRE=0;	/* Set channel prescaler to divide by 4 */
	EMIOS_0.CH[0].CCR.B.UCPEN = 1;	/* Enable prescaler; uses default divide by 4 */
	EMIOS_0.CH[0].CCR.B.FREN = 1;	/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[0].CADR.R = 10000;	/* 设置周期0.02s  50HZ */
	EMIOS_0.CH[0].CCR.B.MODE = 0x50;	/* Modulus Counter Buffered (MCB) */
	EMIOS_0.CH[0].CCR.B.BSL = 0x3;	/* Use internal counter */
    /* 方向舵机 PWM PA2 输出0-50000 */
	EMIOS_0.CH[2].CCR.B.BSL = 0x1;	/* Use counter bus C (default) */
	EMIOS_0.CH[2].CCR.B.MODE = 0x60;	/* Mode is OPWM Buffered */  
    EMIOS_0.CH[2].CCR.B.EDPOL = 1;	/* Polarity-leading edge sets output/trailing clears*/
	EMIOS_0.CH[2].CADR.R = 1;	/* Leading edge when channel counter bus=250*/
	EMIOS_0.CH[2].CBDR.R =STEER_HELM_CENTER;// STEER_HELM_CENTER;	/* Trailing edge when channel counter bus=500*/
	SIU.PCR[2].R = 0x0600;/* [11:10]选择AFx 此处AF1 /* MPC56xxS: Assign EMIOS_0 ch 2 to pad */
}
/*************************电机接口函数*********************/
void SET_motor(signed int speed)
{
	if(speed>=0) 
	{
		EMIOS_0.CH[9].CBDR.R = speed;
		EMIOS_0.CH[10].CBDR.R = 0;
	}
	else 
	{
		EMIOS_0.CH[9].CBDR.R = 0;
		EMIOS_0.CH[10].CBDR.R =-speed;
	}
}
/*************************舵机接口函数***********************/
void SET_steer(signed int steer)
{
	EMIOS_0.CH[2].CBDR.R = steer;
}



//*****************************************************************************************************************
//****************************************中断初始化******************************************************    	  *
//*****************************************************************************************************************

void enableIrq(void) {
  INTC.CPR.B.PRI = 0;          /* Single Core: Lower INTC's current priority */
  asm(" wrteei 1");	    	   /* Enable external interrupts */
}




//*****************************************************************************************************************
//**********************************************定时初始化*************************************************    	  *
//*****************************************************************************************************************
void initPIT(void) 
{                                 //PIT02msec中断立controlflag  PIT1速度反馈2ms一个控制周期
                           	       // NOTE:  DIVIDER FROM SYSCLK TO PIT ASSUMES DEFAULT DIVIDE BY 1 
  PIT.PITMCR.R = 0x00000001;       // Enable PIT and configure timers to stop in debug mode 
  PIT.CH[0].LDVAL.R = 640000;        //PIT0 timeout=160000 sysclks x 1sec/80M sysclks =2msec
  PIT.CH[0].TCTRL.R = 0X00000003;    //Enable PIT0 interrupt and make PIT active to count 
  
  PIT.CH[1].LDVAL.R = 800000;      // PIT1 timeout = 800000 sysclks x 1sec/80M sysclks = 10msec 
  PIT.CH[1].TCTRL.R = 0x00000003; // Enable PIT1 interrupt and make PIT active to count 
  
 // PIT.CH[2].LDVAL.R =320000000;    //设置计数值为32000000
 // PIT.CH[2].TCTRL.R = 0x000000003; //使能PIT2计数，并使能中断
  
  INTC_InstallINTCInterruptHandler(Pit0ISR,59,1); 
 // INTC_InstallINTCInterruptHandler(Pit1ISR,60,2);
}




void initEMIOS_0ModulusCounter(void) //频率采集初始化
{
	/* 计数部分 PD12 */
	EMIOS_0.CH[24].CCR.B.MODE = 0x51;	/* Mode is MCB */
	EMIOS_0.CH[24].CCR.B.BSL = 0x3;	/* Use internal counter */
	EMIOS_0.CH[24].CCR.B.UCPRE=0;	/* Set channel prescaler to divide by 1 */
	EMIOS_0.CH[24].CCR.B.UCPEN = 1;	/* Enable prescaler; uses default divide by 1 */
	EMIOS_0.CH[24].CCR.B.FREN = 0;	/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[24].CCR.B.EDPOL=1;	/* Edge Select rising edge */
	EMIOS_0.CH[24].CADR.R=0xffff;
		/* (WORD)EMIOS_0.CH[24].CCNTR.R 数据寄存器 */
	SIU.PCR[60].R = 0x0500;	/* Initialize pad for eMIOS channel Initialize pad for input */
	
	/*计数部分 PE0 */
	EMIOS_0.CH[16].CCR.B.MODE = 0x51;	/* Mode is MCB */
	EMIOS_0.CH[16].CCR.B.BSL = 0x3;	/* Use internal counter */
	EMIOS_0.CH[16].CCR.B.UCPRE=0;	/* Set channel prescaler to divide by 1 */
	EMIOS_0.CH[16].CCR.B.UCPEN = 1;	/* Enable prescaler; uses default divide by 1 */
	EMIOS_0.CH[16].CCR.B.FREN = 0;	/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[16].CCR.B.EDPOL=1;	/* Edge Select rising edge */
	EMIOS_0.CH[16].CADR.R=0xffff;
		/* (WORD)EMIOS_0.CH[16].CCNTR.R 数据寄存器 */
	SIU.PCR[64].R = 0x0500;	/* Initialize pad for eMIOS channel Initialize pad for input */
	
	/*计数部分 PE7 */
	EMIOS_0.CH[23].CCR.B.MODE = 0x51;	/* Mode is MCB */
	EMIOS_0.CH[23].CCR.B.BSL = 0x3;	/* Use internal counter */
	EMIOS_0.CH[23].CCR.B.UCPRE=0;	/* Set channel prescaler to divide by 1 */
	EMIOS_0.CH[23].CCR.B.UCPEN = 1;	/* Enable prescaler; uses default divide by 1 */
	EMIOS_0.CH[23].CCR.B.FREN = 0;	/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[23].CCR.B.EDPOL=1;	/* Edge Select rising edge */
	EMIOS_0.CH[23].CADR.R=0xffff;
		/* (WORD)EMIOS_0.CH[23].CCNTR.R 数据寄存器 */
	SIU.PCR[71].R = 0x0500;	/* Initialize pad for eMIOS channel Initialize pad for input */
	
	/*计数部分 PE7 */
	EMIOS_0.CH[3].CCR.B.MODE = 0x51;	/* Mode is MCB */
	EMIOS_0.CH[3].CCR.B.BSL = 0x3;	/* Use internal counter */
	EMIOS_0.CH[3].CCR.B.UCPRE=0;	/* Set channel prescaler to divide by 1 */
	EMIOS_0.CH[3].CCR.B.UCPEN = 1;	/* Enable prescaler; uses default divide by 1 */
	EMIOS_0.CH[3].CCR.B.FREN = 0;	/* Freeze channel counting when in debug mode */
	EMIOS_0.CH[3].CCR.B.EDPOL=1;	/* Edge Select rising edge */
	EMIOS_0.CH[3].CADR.R=0xffff;
		/* (WORD)EMIOS_0.CH[23].CCNTR.R 数据寄存器 */
	SIU.PCR[3].R = 0x0500;	/* Initialize pad for eMIOS channel Initialize pad for input */
	
}


//*****************************************************************************************************************
//************************************************A/D模块************************************************    	  *
//*****************************************************************************************************************
void initADC(void)
{
	//ADC.MCR.R = 0x00000100;          /* Conversion times for 80MHz ADClock */
	//ADC.NCMR[1].R = 0x0000007f;     /* Select ANS0 inputs for conversion */
	//ADC.CTR[1].R = 0x00008606;      /* Conversion times for 32MHz ADClock */
	//ADC.MCR.B.NSTART=1;             /* Trigger normal conversions for ADC0 */
	//SIU.PCR[24].R = 0x2000;         /* MPC56xxB: Initialize PB[8] as ANS0 */
	//SIU.PCR[25].R = 0x2000;         /* MPC56xxB: Initialize PB[9] as ANS1 */
    //ADC.MCR.B.NSTART=1;             /* Trigger normal conversions for ADC0 */
    ADC.MCR.R        = 0x20000000;        //初始化ADC为扫描模式，时钟为32MHz。
    ADC.NCMR[0].R    = 0x0000000F;        //选择转换ADC0,ADC1,ADC2,ADC3
    ADC.CTR[0].R     = 0x00008606;        //设置转换时间
   	SIU.PCR[20].R = 0x2000;               // 设置PB[4]为ADC0模数转换输入
  	SIU.PCR[21].R = 0x2000;               // 设置PB[5]为ADC1模数转换输入
    ADC.MCR.B.NSTART = 1;                 //启动AD转换
}


//*****************************************************************************************************************
//************************************************调试模块************************************************    	  *
//*****************************************************************************************************************

/*************************蓝牙初始化***********************/
void initLINFlex_0_UART (void) 
{
	LINFLEX_0.LINCR1.B.INIT=1;  //进入初始化模式
	LINFLEX_0.LINCR1.R=0x00000015; 
	LINFLEX_0.LINIER.B.DRIE=0; //允许接收中断
	/*波特率算法baud=Fperiph_clk/(16*LFDIV)
	DIV_M=LFDIV整数部分
	DIV_F=LFDIV小数部分*16  */ 	
	LINFLEX_0.LINIBRR.B.DIV_M= 69;  	//波特率设置38400:80M-130+3 57600:80M-86+13 115200:80M-43+6  9600:80M-520+83
    LINFLEX_0.LINFBRR.B.DIV_F = 7;		//38400:64M-104+3
    LINFLEX_0.UARTCR.B.UART=1;
	LINFLEX_0.UARTCR.R=0x00000033;//8-bit data、UART mode
	LINFLEX_0.LINCR1.B.INIT=0; //退出初始化模式
	
	SIU.PCR[18].R = 0x0400;    /* MPC56xxB: Configure port B2 as LIN0TX */
    SIU.PCR[19].R = 0x0103;    /* MPC56xxB: Configure port B3 as LIN0RX */
  	///INTC_InstallINTCInterruptHandler(LINFlex_RX_Interrupt,79,4); 
}
/*************************蓝牙接口函数*********************/
void LINFlex_TX(unsigned char data)
{
	LINFLEX_0.BDRL.B.DATA0=data;       //发送语句
	while(!LINFLEX_0.UARTSR.B.DTF){}
	LINFLEX_0.UARTSR.B.DTF=1;
}

void LINFlex_RX_Interrupt(void)
{
	if(LINFLEX_0.BDRM.B.DATA4==0x55)
	{
		;//for(;;){SET_motor(0);}
	}
	LINFLEX_0.UARTSR.B.DRF=1;      //清空标志位
}

/*********************显示屏初始化************************/
void initOLED(void)
{
	SIU.PCR[27].R = 0x0203;  
	SIU.PCR[28].R = 0x0203;  
	SIU.PCR[29].R = 0x0203;  
	SIU.PCR[30].R = 0x0203;  
	SIU.PCR[31].R = 0x0203;  
}
/*********************拨码开关、按键初始化************************/
void initKeys_Switchs(void)
{
	SIU.PCR[24].R = 0x0100;               // PB[8]: Input  switch6 SIU.GPDO[64].B.PDI 
	SIU.PCR[48].R = 0x0100;               // PD[0]: Input  switch5
	SIU.PCR[50].R = 0x0100;               // PD[2]: Input  switch4
	SIU.PCR[52].R = 0x0100;               // PD[4]: Input  switch3
	SIU.PCR[54].R = 0x0100;               // PD[6]: Input  switch2
	SIU.PCR[56].R = 0x0100;               // PD[8]: Input  switch1
	SIU.PCR[57].R = 0x0100; 
	SIU.PCR[22].R = 0x0100; 
	SIU.PCR[63].R = 0x0100;               //PD[15]干簧管
	SIU.PCR[15].R = 0x0100;
	SIU.PCR[44].R = 0x0100;
}


/*void SAIC_INIT(void)
{
	EMIOS_0.CH[3].CCR.B.MODE  = 0x02;   //设置输入捕捉SAIC模式
	EMIOS_0.CH[3].CCR.B.BSL   = 0x3;    //使用1MHz的内部计数器作为时钟源
	EMIOS_0.CH[3].CCR.B.IF   = 0x2;     //设置输入滤波器
	EMIOS_0.CH[3].CCR.B.FCK  = 1;       //滤波器时钟为主时钟
	EMIOS_0.CH[3].CCR.B.FEN  = 1;       //使能输入捕捉中断
	EMIOS_0.CH[3].CCR.B.DMA  = 0;       //FLAG用作中断请求
	EMIOS_0.CH[3].CCR.B.EDSEL  = 0;       //设置上升沿触发中断
	EMIOS_0.CH[3].CCR.B.EDPOL  = 1;      
	EMIOS_0.CH[3].CCR.B.UCPRE = 0;      //设置分频值为1
	EMIOS_0.CH[3].CCR.B.UCPEN = 1;      //使能分频器
	EMIOS_0.CH[3].CCR.B.FREN  = 1;      //在冻结模式下停止计数
	
	SIU.PCR[3].R = 0x0102; 
	//INTC_InstallINTCInterruptHandler(SAIC1_inter,142,1);

	/*EMIOS_0.CH[7].CCR.B.MODE  = 0x02;   //设置输入捕捉SAIC模式
	EMIOS_0.CH[7].CCR.B.BSL   = 0x3;    //使用1MHz的内部计数器作为时钟源
	EMIOS_0.CH[7].CCR.B.IF   = 0x2;     //设置输入滤波器
	EMIOS_0.CH[7].CCR.B.FCK  = 1;       //滤波器时钟为主时钟
	EMIOS_0.CH[7].CCR.B.FEN  = 1;       //使能输入捕捉中断
	EMIOS_0.CH[7].CCR.B.DMA  = 0;       //FLAG用作中断请求
	EMIOS_0.CH[7].CCR.B.EDSEL  = 0;       //设置下降沿沿触发中断
	EMIOS_0.CH[7].CCR.B.EDPOL  = 1;      
	EMIOS_0.CH[7].CCR.B.UCPRE = 0;      //设置分频值为1
	EMIOS_0.CH[7].CCR.B.UCPEN = 1;      //使能分频器
	EMIOS_0.CH[7].CCR.B.FREN  = 1;      //在冻结模式下停止计数
	
	SIU.PCR[7].R = 0x0102; 
	INTC_InstallINTCInterruptHandler(SAIC2_inter,144,1);
}*/
