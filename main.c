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


//ȫ�ֱ�������
unsigned int speed_count=0;									//ռ�ձȼ����� 50��һ����
char front_left_speed_duty=SPEED_DUTY;
char front_right_speed_duty=SPEED_DUTY;
char behind_left_speed_duty=SPEED_DUTY;
char behind_right_speed_duty=SPEED_DUTY;

unsigned int dis_left,dis_right;
unsigned int dis_direct;
extern unsigned char avg_speed;								//��������
extern volatile float distance_tran; 	//��������
volatile float distance_last = 0;			//�ϴξ����¼
volatile float distance_judgement = 0;	//�����жϾ���

unsigned char tick_1ms = 0;								//1ms����������Ϊ����Ļ���������
unsigned char tick_5ms = 0;								//5ms����������Ϊ�������Ļ�������
unsigned char tick_200ms = 0;							//ˢ��LED��˸��ʾ
unsigned int tick_rushtime=0;
int tick_slowtime = 0;
//unsigned char tick_500ms = 0;							//���������ʱ

unsigned char HalfSpeed = 0;
unsigned char Stopped = 0;

char ctrl_comm = COMM_STOP;								//����ָ��
char ctrl_comm_last = COMM_STOP;					//��һ�ε�ָ��
extern unsigned int oscillation;									//��ʱ����
char RC_comm = COMM_STOP;									//����ָ��
char WF_comm = COMM_STOP;									//����ָ��
unsigned char continue_time=0;
unsigned char wf_rec_flag=0;							//WIFI��λ�����Ʊ�־λ

unsigned char duoji_count=0;
unsigned char zhuanjiao = 11;

unsigned char CCD_UD_duoji_count=0;
unsigned char CCD_UD_zhuanjiao = 11;
unsigned char CCD_LR_duoji_count=0;
unsigned char CCD_LR_zhuanjiao = 11;

float pitch,roll,yaw; 					//ŷ����:pitch������roll:�����yaw:ƫ��
float l_pitch,l_roll,l_yaw;			//��һ�ε�ŷ���Ǽ�¼
float pitch_last = 0;								//��¼��һ�θ����ǣ�
short aacx,aacy,aacz;						//���ٶȴ�����ԭʼ����
short acz[5] ;                   //z����ٶȼ�¼
int acz_judge;                 //�ж���z����ٶ�
short posture = 0;							//��̬��1�����ϣ�0������
int i = 0;												//���
int stopkkkk = 1;
short gyrox,gyroy,gyroz;				//������ԭʼ����
short tempIMU;									//�¶�
short temp;
short ramp = 0;                 //�µ��������1�����ϣ�0������
char Bright_check = 0;					//���ȼ������1��������0���ڰ�
char Red_light = 0;							//���ʶ������1����ʶ��0��δʶ��
char Green_light = 0;						//�̵�ʶ������1����ʶ��0��δʶ��
char Identification = 0;				//ͼ��ʶ����������
char void_right = 0;						//�Ҳ������Ͻ����1�����ϰ���0�����ϰ�
char void_left = 0;							//��������Ͻ����1�����ϰ���0�����ϰ�
char Search_L = 0;							//ѭ���������1���к��ߣ�0���޺���
char Search_M = 0;							//ѭ���м�����1���к��ߣ�0���޺���
char Search_R = 0;							//ѭ���Ҳ�����1���к��ߣ�0���޺���
char Search_L_last = 0;							//ѭ������ϴν����1���к��ߣ�0���޺���
char Search_M_last = 0;							//ѭ���м��ϴν����1���к��ߣ�0���޺���
char Search_R_last = 0;							//ѭ���Ҳ��ϴν����1���к��ߣ�0���޺���
char LF_Light = 0;							//��ǰ��״̬��1�����ƣ�0���ص�
char RF_Light = 0;							//��ǰ��״̬��1�����ƣ�0���ص�
char LB_Light = 0;							//����״̬��1�����ƣ�0���ص�
char RB_Light = 0;							//�Һ��״̬��1�����ƣ�0���ص�

char Is_Display = 0;						//�Ƿ�ͨ�����ڷ�����Ƶ
char Is_Movebyline = 0;					//�Ƿ�ѭ��
char key = 0;										//��������

extern u8 ov_sta;								//��exit.c���涨��



//�µ����
void RampCheck(void)
{
	if (pitch <= 0)
	{
		ramp = 1;
	}
}

//��̬���
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
		

//���������ȼ��
void BrightCheck(void)
{
	if (BRIGHT_IO == BRIGHT_N)		//�����ڰ�
	{
		Bright_check = 0;
		LF_Light_SET;
		RF_Light_SET;
		LB_Light_SET;
		RB_Light_SET;
	}
	else		//��������
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
//ѭ�����,ͨ���ж��������Թܵ�״̬������С���˶�
void SearchLine(void)  //�Ӵ���������жϸ�·�Ƿ��⵽���ߣ���Search_L��Search_M��Search_R��ֵ����interface.h����ѭ�����Թܶ���
{
	
if (Search_R == BLACK_AREA)		//��
	{
		ctrl_comm = COMM_RIGHT;
	}
	else if (Search_L == BLACK_AREA)		//��
	{
		ctrl_comm = COMM_LEFT;
	}
	else if (Search_M == BLACK_AREA)		//��
	{
		ctrl_comm = COMM_UP;
	}
	if (Search_M == WHITE_AREA && Search_L == WHITE_AREA && Search_R == WHITE_AREA)
	{
		if (Search_L_last == BLACK_AREA)		//��
		{
			ctrl_comm = COMM_LEFT;
	}
		else if (Search_R_last == BLACK_AREA)		//��
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
//С���˶�����
void Move(void) //
{
	
	
	//��ָ���г�

		switch(ctrl_comm)
		{
			case COMM_UP:  //С��ǰ��  
			{	
//				if (ctrl_comm != ctrl_comm_last) oscillation = oscillation/2;
//				else oscillation =1000;
				CarGo();
				CarGo();
//				display_list_char(8,0,"^");
				break;
			}
			case COMM_DOWN:  //С������
			{
//				if (ctrl_comm != ctrl_comm_last) oscillation = oscillation/2;
//				else oscillation =1000;
				CarBack();
//				display_list_char(8,0,"v");
				break;
			}
			case COMM_LEFT:  //С����ת
			{
//				if (ctrl_comm != ctrl_comm_last) oscillation = oscillation/2;
//				else oscillation =1000;
				CarLeft();//motor.c����
//				display_list_char(8,0,"<");
				break;
			}
			case COMM_RIGHT: //С����ת
			{
//				if (ctrl_comm != ctrl_comm_last) oscillation = oscillation/2;
//				else oscillation =1000;
				CarRight();//motor.c����
//				display_list_char(8,0,">");
				break;
			}
			case COMM_STOP:  //С��ͣ��
			{
				CarStop();
//				display_list_char(8,0,"X");
				break;
			}
			default : break;
		}
		Delayms(2);//����

}

//�����źŽ���
void SignalReceive(void)
{
	atk_8266_wifista_Rece();
	
	if(wf_rec_flag == 1)	//���յ�WIFI�ź�
	{
		wf_rec_flag = 0;
		switch(WF_comm)
		{
			case COMM_UP:    	ctrl_comm = COMM_UP;break; //ǰ��ָ��
			case COMM_DOWN:  	ctrl_comm = COMM_DOWN;break; //����ָ��
			case COMM_LEFT:  	ctrl_comm = COMM_LEFT;break; //��תָ��
			case COMM_RIGHT: 	ctrl_comm = COMM_RIGHT;break; //��תָ��
			case COMM_STOP:  	ctrl_comm = COMM_STOP;break; //ͣ��ָ��

			default : break;
		}
	}
	if(ir_rec_flag == 1)	//���յ������ź�
	{
		ir_rec_flag = 0;
		switch(RC_comm)
		{
			case COMM_UP:    	ctrl_comm = COMM_UP;break; //ͬ��
			case COMM_DOWN:  	ctrl_comm = COMM_DOWN;break;
			case COMM_LEFT:  	ctrl_comm = COMM_LEFT;break;
			case COMM_RIGHT: 	ctrl_comm = COMM_RIGHT;break;
			case COMM_STOP:  	ctrl_comm = COMM_STOP;break;

			default : break;
		}
	}
}

//���⡢�������
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
	if (VOID_R_IO == BARRIER_Y)		//�Ҳ����ϰ�
	{
		void_right = 0;
	}
	else                          //�Ҳ����ϰ�
	{
		void_right = 1;             
	}
	if (VOID_L_IO  == BARRIER_Y)	//������ϰ�
	{
		void_left = 0;
	}
	else                          //������ϰ�
	{
		void_left = 1;
	}
	if ((void_right == 0)&&(void_left == 0) || (distance_judgement <= 50))//��������ϰ�
	{
		HalfSpeed = 1;							//�ٶȼ���
	}
	else
	{
		HalfSpeed = HalfSpeed;		//�ٶȱ���

	}
//	if ((distance_judgement <= 10.5)&&(distance_judgement >= 9)&&((void_right == 0)||(void_left == 0)))//�ϰ����벻����5cm
//	{
//		ctrl_comm = COMM_STOP;//ͣ��
//	}
		if ((distance_judgement <= 10.5)&&(distance_judgement >= 8.5)&&((void_right == 0)||(void_left == 0)))//�ϰ����벻����5cm
	{
		ctrl_comm = COMM_STOP;//ͣ��
	}
	else if ((distance_judgement < 8.5)&&((void_right == 0)||(void_left == 0)))
	{
		HalfSpeed = 1;							//�ٶȼ���
		ctrl_comm = COMM_DOWN;//����
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
//������
int main(void)
{

	delay_init_O(); 			//�ⲿʱ��ѡ��
	delay_init();					//��ʼ���ӳٺ���
	GPIOCLKInit();				//ʹ������GPIOʱ��
	UserLEDInit();				//��ʼ���û�ָʾ��(PC13�˿�)
	//IRCtrolInit();				//��ʼ������ң������ �೵ʱ��ǰ�����ܺ󳵵ĺ�����Ϸ���Ӱ��
	TIM2_Init();					//��ʼ����ʱ��2
	MotorInit();					//��ʼ��������ƽӿ�
	UltraSoundInit();			//��ʼ�����������ģ��
	RedRayInit();					//��ʼ��ѭ��ģ�鼰����ģ��ĺ���Թ�
	ServoInit();					//��ʼ��3·������ƽӿ�
	SPI1_Init();					//��ʼ��SPI1�ӿ�		
//	OLED_Init();					//��ʼ��Һ����ʾ
	BrightInit();					//��ʼ�����ȼ��ӿ�
	LightInit();					//��ʼ�����ƽӿ�
	KEY_Init();						//��ʼ�������ӿ�
	mem_init();						//��ʼ���ڴ��
	uart_init(115200);  	//��ʼ������1��������Ϊ115200������USMART
	USART3_Init(115200);  //��ʼ������3��������Ϊ115200������WIFIģ��
	//usmart_init(72);		//��ʼ��USMART@72MHz
	MPU_Init();						//��ʼ�����Բ���ģ��
	mpu_dmp_init();				//��ʼ�����Բ���ģ��DMP
	
	
	EXTI8_Init();													//ʹ�ܶ�ʱ������
	
	//=====����ΪOLEDҺ������ʾ����======================
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
	//WIFIģ���ϵ��Զ�����͸��ģʽ���粻��Ҫ�޸�WIFI���ã��ɽ���ʼ������ע�͵�
	//atk_8266_wifista_Init();
	//=========================================================================

//	display_list_char(0,0,"0 00000");
	
 //=======����Ϊ��������ѭ��==============================================
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
	 
			
	  if(tick_5ms >= 5) //5ms��ʱ��
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
			if(key == KEY2_PRES) //����2���Ƿ�ѭ���г�
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
			
			
			
		//	if(tick_200ms >= 4)	//ʵ�����
			//{
				//tick_200ms = 0;
//LEDToggle(LED_PIN); //�û�ָʾ����˸,����������������
	//			atk_8266_wifista_Tran();					//wifi������������λ�� ����
							
		//	}
			
		//	MeasureSpeed();							//����
				
	//		PostureCheck();				//��̬���

		
			
		//	BrightCheck();			//ִ�л��������ȼ��
			
		//	SignalReceive();		//�����źŽ���
			
	//		if(posture != 1)
	//		{
	//			BarrierDetect();					//���ϲ��
	//		}
			SearchLine();
		if (Is_Movebyline != 0 )
		{ 	
         			//ִ��ѭ�����
		Move();	
    		
		}
		
					//С���˶�����

		}
		
		

 }
}

