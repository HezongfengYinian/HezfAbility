#ifndef _NET_APP_H_
#define _NET_APP_H_

#include <rtthread.h>
#include <app_protocol.h>
#define START_FRAME_DATA	0xABCDDCBA
#define AFFILIATED_SYSTEM	0x00000001 
#define VERSION_NUMBER		0x0000

#define CARD_REPORT_DATA	0x0001
#define SYNC_TIME		0x0000

typedef struct
{
    char url[20];
    int port;
} server_info;


typedef struct
{
	rt_int16_t u16Year;   		//��
	rt_int16_t u16Month;		//��
	rt_int16_t u16DayOfWeek;	//���ڣ�0=�����գ�1=����һ.....
	rt_int16_t u16Day;			//��
	rt_int16_t u16Hour;			//ʱ
	rt_int16_t u16Minute;		//��
	rt_int16_t u16Second;		//��
	rt_int16_t u16Milliseconds;	//����
} SYSTEMTIME;


typedef struct
{
    	rt_int32_t u32StartField;		//��ʼ֡
    	rt_int32_t u32FrameLength;		//֡��
	rt_int32_t u32System;			//����ϵͳ
	rt_int16_t u16Versions;			//�汾��
	SYSTEMTIME strTime;			//ϵͳʱ��
	rt_int16_t u16DataType;			//��������
} FRAMEDATA;


typedef struct
{
	app_header_t app_tof_head;
	rt_int16_t u16SeqNum;		//�����к�
	rt_int8_t  u8status;			//��״̬
	rt_int8_t  u8CardType;		//������
	rt_int16_t u16ShortAddr;		//���̵�ַ(����)
	rt_int8_t  u8CardStatus;    		// ����̬
	rt_int8_t  u8SignalIntensity;  	//���ź�
	rt_int16_t u16Battery;		//������
	rt_int16_t u16AngleX;			//X��Ƕ�
	rt_int16_t u16AngleY;			//Y��Ƕ�
	rt_int16_t u16AngleZ;			//Z��Ƕ�
}app_tof_Tunnel_custom_data_ts;


typedef struct
{	
	rt_int16_t u16DeviceNum;		//�豸��(��վ��)	
	rt_int16_t u16CardNum;			//����	
	rt_int16_t u16SeqNum;       		//�����к�	
	rt_int8_t u8Status; 				//����̬		
	rt_int8_t u8CardType;				//������	
	rt_int8_t u8CardStatus;			//��״̬
	rt_int8_t u8SignalIntensity;  		//���ź�
	rt_int16_t u16Battery;			//������	
	rt_int16_t u16AngleX;				//x����̬	
	rt_int16_t u16AngleY;				//y����̬	
	rt_int16_t u16AngleZ;				//z����̬	
}app_tof_report_data_ts;


#define NET_POOL_MAX    (1024 * 16)

extern rt_uint8_t u8NetPool[NET_POOL_MAX];
extern struct rt_messagequeue net_mq;

extern unsigned char net_recv_fine_flag;
extern unsigned char net_send_fine_flag;

rt_bool_t start_net_work(void);
void net_thread_entry(void *parameter);
void app_display_ipaddr(rt_uint32_t IPaddress);
void net_report_restart_msg(void);
void net_report_running_state_msg(int state_mode, const char* p_msg, int msg_len);

#endif
