#include "stm32f10x.h"
#include "interface.h"
#include "OLED_Driver.h"
#include "IRCtrol.h"
#include "motor.h"
#include "UltrasonicCtrol.h"
#include "SpeedCtrol.h"
#include "sys.h"	
#include "key.h"
#include "delay.h"	
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "malloc.h" 
#include "usart.h"
//#include "usart2.h"
#include "common.h" 
#include "usart3.h"	
//#include "usmart.h" 
#include "ov7670.h"
#include "exti.h"
#include "timer.h"


//全局变量定义
unsigned int speed_count=0;									//占空比计数器 50次一周期
char front_left_speed_duty=SPEED_DUTY;
char front_right_speed_duty=SPEED_DUTY;
char behind_left_speed_duty=SPEED_DUTY;
char behind_right_speed_duty=SPEED_DUTY;

unsigned int dis_left,dis_right;
unsigned int dis_direct;
extern unsigned char avg_speed;								//车速声明
extern volatile float distance_tran; 	//距离声明
volatile float distance_last = 0;			//上次距离记录
volatile float distance_judgement = 0;	//用于判断距离

unsigned char tick_1ms = 0;								//1ms计数器，作为电机的基本计数器
unsigned char tick_5ms = 0;								//5ms计数器，作为主函数的基本周期
unsigned char tick_200ms = 0;							//刷新LED闪烁显示
unsigned int tick_rushtime=0;
int tick_slowtime = 0;
//unsigned char tick_500ms = 0;							//电机驱动延时

unsigned char HalfSpeed = 0;
unsigned char Stopped = 0;

char ctrl_comm = COMM_STOP;								//控制指令
char ctrl_comm_last = COMM_STOP;					//上一次的指令
extern unsigned int oscillation;									//延时控制
char RC_comm = COMM_STOP;									//控制指令
char WF_comm = COMM_STOP;									//控制指令
unsigned char continue_time=0;
unsigned char wf_rec_flag=0;							//WIFI上位机控制标志位

unsigned char duoji_count=0;
unsigned char zhuanjiao = 11;

unsigned char CCD_UD_duoji_count=0;
unsigned char CCD_UD_zhuanjiao = 11;
unsigned char CCD_LR_duoji_count=0;
unsigned char CCD_LR_zhuanjiao = 11;

float pitch,roll,yaw; 					//欧拉角:pitch俯仰、roll:横滚、yaw:偏航
float l_pitch,l_roll,l_yaw;			//上一次的欧拉角记录
float pitch_last = 0;								//记录上一次俯仰角；
short aacx,aacy,aacz;						//加速度传感器原始数据
short acz[5] ;                   //z轴加速度记录
int acz_judge;                 //判断用z轴加速度
short posture = 0;							//姿态，1：坡上；0：坡下
int i = 0;												//序号
int stopkkkk = 1;
short gyrox,gyroy,gyroz;				//陀螺仪原始数据
short tempIMU;									//温度
short temp;
short ramp = 0;                 //坡道检测结果，1：坡上；0：坡下
char Bright_check = 0;					//亮度检测结果，1：明亮；0：黑暗
char Red_light = 0;							//红灯识别结果，1：已识别；0：未识别
char Green_light = 0;						//绿灯识别结果，1：已识别；0：未识别
char Identification = 0;				//图形识别结果，待定
char void_right = 0;						//右侧红外避障结果，1：无障碍；0：有障碍
char void_left = 0;							//左侧红外避障结果，1：无障碍；0：有障碍
char Search_L = 0;							//循迹左侧结果，1：有黑线；0：无黑线
char Search_M = 0;							//循迹中间结果，1：有黑线；0：无黑线
char Search_R = 0;							//循迹右侧结果，1：有黑线；0：无黑线
char Search_L_last = 0;							//循迹左侧上次结果，1：有黑线；0：无黑线
char Search_M_last = 0;							//循迹中间上次结果，1：有黑线；0：无黑线
char Search_R_last = 0;							//循迹右侧上次结果，1：有黑线；0：无黑线
char LF_Light = 0;							//左前灯状态，1：开灯；0：关灯
char RF_Light = 0;							//右前灯状态，1：开灯；0：关灯
char LB_Light = 0;							//左后灯状态，1：开灯；0：关灯
char RB_Light = 0;							//右后灯状态，1：开灯；0：关灯

char Is_Display = 0;						//是否通过串口发送视频
char Is_Movebyline = 0;					//是否循迹
char key = 0;										//按键输入

extern u8 ov_sta;								//在exit.c里面定义



//坡道检测
void RampCheck(void)
{
	if (pitch <= 0)
	{
		ramp = 1;
	}
}

//姿态检测
void PostureCheck(void)
{
	if ((acz_judge<15900)||(pitch<-2))
	{
//		HalfSpeed = 2;
		posture = 1;
//		if (ramp == 0)	ramp = 1;
//		else ramp = 0;
	}
	else
	{
		posture = 0;
	}
}
		

//环境光亮度检测
void BrightCheck(void)
{
	if (BRIGHT_IO == BRIGHT_N)		//环境黑暗
	{
		Bright_check = 0;
		LF_Light_SET;
		RF_Light_SET;
		LB_Light_SET;
		RB_Light_SET;
	}
	else		//环境明亮
	{
		Bright_check = 1;
		LF_Light_RESET;
		RF_Light_RESET;
		LB_Light_RESET;
		RB_Light_RESET;
	}
}

void endddd(void)
{while(1)
{CarStop();}
}
//循迹检测,通过判断三个光电对管的状态来控制小车运动
void SearchLine(void)  //从传感器输出判断各路是否检测到黑线，给Search_L，Search_M，Search_R赋值，在interface.h中找循迹光电对管定义
{
	
if (Search_R == BLACK_AREA)		//中
	{
		ctrl_comm = COMM_RIGHT;
	}
	else if (Search_L == BLACK_AREA)		//左
	{
		ctrl_comm = COMM_LEFT;
	}
	else if (Search_M == BLACK_AREA)		//右
	{
		ctrl_comm = COMM_UP;
	}
	if (Search_M == WHITE_AREA && Search_L == WHITE_AREA && Search_R == WHITE_AREA)
	{
		if (Search_L_last == BLACK_AREA)		//左
		{
			ctrl_comm = COMM_LEFT;
	}
		else if (Search_R_last == BLACK_AREA)		//右
		{
			ctrl_comm = COMM_RIGHT;
	}
		else
		{
			ctrl_comm = COMM_RIGHT;
		}
	
	}
		if (Search_M == BLACK_AREA && Search_L == BLACK_AREA && Search_R == BLACK_AREA)
	{
		
		//CarBack();
		
			ctrl_comm = COMM_STOP;
		//endddd();
	
}
}
//小车运动控制
void Move(void) //
{
	
	
	//按指令行车

		switch(ctrl_comm)
		{
			case COMM_UP:  //小车前进  
			{	
//				if (ctrl_comm != ctrl_comm_last) oscillation = oscillation/2;
//				else oscillation =1000;
				CarGo();
				CarGo();
//				display_list_char(8,0,"^");
				break;
			}
			case COMM_DOWN:  //小车后退
			{
//				if (ctrl_comm != ctrl_comm_last) oscillation = oscillation/2;
//				else oscillation =1000;
				CarBack();
//				display_list_char(8,0,"v");
				break;
			}
			case COMM_LEFT:  //小车左转
			{
//				if (ctrl_comm != ctrl_comm_last) oscillation = oscillation/2;
//				else oscillation =1000;
				CarLeft();//motor.c里编程
//				display_list_char(8,0,"<");
				break;
			}
			case COMM_RIGHT: //小车右转
			{
//				if (ctrl_comm != ctrl_comm_last) oscillation = oscillation/2;
//				else oscillation =1000;
				CarRight();//motor.c里编程
//				display_list_char(8,0,">");
				break;
			}
			case COMM_STOP:  //小车停车
			{
				CarStop();
//				display_list_char(8,0,"X");
				break;
			}
			default : break;
		}
		Delayms(2);//防抖

}

//控制信号接收
void SignalReceive(void)
{
	atk_8266_wifista_Rece();
	
	if(wf_rec_flag == 1)	//接收到WIFI信号
	{
		wf_rec_flag = 0;
		switch(WF_comm)
		{
			case COMM_UP:    	ctrl_comm = COMM_UP;break; //前进指令
			case COMM_DOWN:  	ctrl_comm = COMM_DOWN;break; //后退指令
			case COMM_LEFT:  	ctrl_comm = COMM_LEFT;break; //左转指令
			case COMM_RIGHT: 	ctrl_comm = COMM_RIGHT;break; //右转指令
			case COMM_STOP:  	ctrl_comm = COMM_STOP;break; //停车指令

			default : break;
		}
	}
	if(ir_rec_flag == 1)	//接收到红外信号
	{
		ir_rec_flag = 0;
		switch(RC_comm)
		{
			case COMM_UP:    	ctrl_comm = COMM_UP;break; //同上
			case COMM_DOWN:  	ctrl_comm = COMM_DOWN;break;
			case COMM_LEFT:  	ctrl_comm = COMM_LEFT;break;
			case COMM_RIGHT: 	ctrl_comm = COMM_RIGHT;break;
			case COMM_STOP:  	ctrl_comm = COMM_STOP;break;

			default : break;
		}
	}
}

//红外、超声测距
void BarrierDetect(void)
{
	Distance();
	if ((distance_tran <= distance_last - 50) ||((distance_tran <= distance_last + 2) && (distance_tran >= distance_tran - 2)))
	{
		distance_judgement = distance_last;
	}
	else
	{
		distance_judgement = distance_tran;
	}
	if (VOID_R_IO == BARRIER_Y)		//右侧有障碍
	{
		void_right = 0;
	}
	else                          //右侧无障碍
	{
		void_right = 1;             
	}
	if (VOID_L_IO  == BARRIER_Y)	//左侧有障碍
	{
		void_left = 0;
	}
	else                          //左侧无障碍
	{
		void_left = 1;
	}
	if ((void_right == 0)&&(void_left == 0) || (distance_judgement <= 50))//两侧均有障碍
	{
		HalfSpeed = 1;							//速度减半
	}
	else
	{
		HalfSpeed = HalfSpeed;		//速度保持

	}
//	if ((distance_judgement <= 10.5)&&(distance_judgement >= 9)&&((void_right == 0)||(void_left == 0)))//障碍距离不超过5cm
//	{
//		ctrl_comm = COMM_STOP;//停车
//	}
		if ((distance_judgement <= 10.5)&&(distance_judgement >= 8.5)&&((void_right == 0)||(void_left == 0)))//障碍距离不超过5cm
	{
		ctrl_comm = COMM_STOP;//停车
	}
	else if ((distance_judgement < 8.5)&&((void_right == 0)||(void_left == 0)))
	{
		HalfSpeed = 1;							//速度减半
		ctrl_comm = COMM_DOWN;//倒车
	}
	else
	{
		ctrl_comm = ctrl_comm;
	}
	if (distance_last != distance_tran)
	{
		distance_last = distance_tran;
	}
}
//主函数
int main(void)
{

	delay_init_O(); 			//外部时钟选择
	delay_init();					//初始化延迟函数
	GPIOCLKInit();				//使能所有GPIO时钟
	UserLEDInit();				//初始化用户指示灯(PC13端口)
	//IRCtrolInit();				//初始化红外遥控配置 多车时，前车会受后车的红外避障发射影响
	TIM2_Init();					//初始化定时器2
	MotorInit();					//初始化电机控制接口
	UltraSoundInit();			//初始化超声波测距模块
	RedRayInit();					//初始化循迹模块及避障模块的红外对管
	ServoInit();					//初始化3路舵机控制接口
	SPI1_Init();					//初始化SPI1接口		
//	OLED_Init();					//初始化液晶显示
	BrightInit();					//初始化亮度检测接口
	LightInit();					//初始化车灯接口
	KEY_Init();						//初始化按键接口
	mem_init();						//初始化内存池
	uart_init(115200);  	//初始化串口1，波特率为115200，用于USMART
	USART3_Init(115200);  //初始化串口3，波特率为115200，用于WIFI模块
	//usmart_init(72);		//初始化USMART@72MHz
	MPU_Init();						//初始化惯性测量模块
	mpu_dmp_init();				//初始化惯性测量模块DMP
	
	
	EXTI8_Init();													//使能定时器捕获
	
	//=====以下为OLED液晶屏显示内容======================
//	LCD_All();
//	Delayms(50);
//	LCD_Clear();
//	
//	display_list_char(0,1,"D:000cm");
//	display_list_char(8,1,"S:000c/s");
//	display_list_char(0,2,"T:-00.0");
//	display_list_char(8,2,"R:-000.0");
//	display_list_char(0,3,"P:-00.0");
//	display_list_char(8,3,"Y:-000.0");
//	display_list_char(0,0,"WIFI");	
	//display_list_char(0,0,"0 00000");	
	//====================================================
	
	



	//=========================================================================
	//WIFI模块上电自动进入透传模式，如不需要修改WIFI配置，可将初始化部分注释掉
	//atk_8266_wifista_Init();
	//=========================================================================

//	display_list_char(0,0,"0 00000");
	
 //=======以下为主函数大循环==============================================
 while(1)
 {	
	if (Search_M != WHITE_AREA || Search_L != WHITE_AREA || Search_R != WHITE_AREA)
			{
				if (Search_L_last != Search_L) Search_L_last = Search_L;
				if (Search_R_last != Search_R) Search_R_last = Search_L;
				if (Search_M_last != Search_M) Search_M_last = Search_L;
			} 
			Search_L = SEARCH_L_IO;
			Search_R = SEARCH_R_IO;
			Search_M = SEARCH_M_IO;
	 
			
	  if(tick_5ms >= 5) //5ms定时到
		{			
			if (Search_M != WHITE_AREA || Search_L != WHITE_AREA || Search_R != WHITE_AREA)
			{
				if (Search_L_last != Search_L) Search_L_last = Search_L;
				if (Search_R_last != Search_R) Search_R_last = Search_L;
				if (Search_M_last != Search_M) Search_M_last = Search_L;
			} 
			Search_L = SEARCH_L_IO;
			Search_R = SEARCH_R_IO;
			Search_M = SEARCH_M_IO;
			
			key = KEY_Scan(0);
			if(key == KEY2_PRES) //按键2：是否循迹行车
			{
				if(Is_Movebyline == 0)
				{
					Is_Movebyline = 1;
					display_list_char(15,0,"1");	
				}
				else
				{
					Is_Movebyline = 0;
					display_list_char(15,0,"0");
				}	
			}
			tick_5ms = 0;
			//tick_200ms++;
			
			
			
		//	if(tick_200ms >= 4)	//实测调整
			//{
				//tick_200ms = 0;
//LEDToggle(LED_PIN); //用户指示灯闪烁,表明程序运行正常
	//			atk_8266_wifista_Tran();					//wifi发送数据至上位机 中速
							
		//	}
			
		//	MeasureSpeed();							//测速
				
	//		PostureCheck();				//姿态检测

		
			
		//	BrightCheck();			//执行环境光亮度检测
			
		//	SignalReceive();		//控制信号接收
			
	//		if(posture != 1)
	//		{
	//			BarrierDetect();					//避障测距
	//		}
			SearchLine();
		if (Is_Movebyline != 0 )
		{ 	
         			//执行循迹检测
		Move();	
    		
		}
		
					//小车运动控制

		}
		
		

 }
}

