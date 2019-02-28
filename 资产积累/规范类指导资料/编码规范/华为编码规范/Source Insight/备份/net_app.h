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
	rt_int16_t u16Year;   		//年
	rt_int16_t u16Month;		//月
	rt_int16_t u16DayOfWeek;	//星期，0=星期日，1=星期一.....
	rt_int16_t u16Day;			//日
	rt_int16_t u16Hour;			//时
	rt_int16_t u16Minute;		//分
	rt_int16_t u16Second;		//秒
	rt_int16_t u16Milliseconds;	//毫秒
} SYSTEMTIME;


typedef struct
{
    	rt_int32_t u32StartField;		//起始帧
    	rt_int32_t u32FrameLength;		//帧长
	rt_int32_t u32System;			//所属系统
	rt_int16_t u16Versions;			//版本号
	SYSTEMTIME strTime;			//系统时间
	rt_int16_t u16DataType;			//数据类型
} FRAMEDATA;


typedef struct
{
	app_header_t app_tof_head;
	rt_int16_t u16SeqNum;		//卡序列号
	rt_int8_t  u8status;			//卡状态
	rt_int8_t  u8CardType;		//卡类型
	rt_int16_t u16ShortAddr;		//卡短地址(卡号)
	rt_int8_t  u8CardStatus;    		// 卡姿态
	rt_int8_t  u8SignalIntensity;  	//卡信号
	rt_int16_t u16Battery;		//卡电量
	rt_int16_t u16AngleX;			//X轴角度
	rt_int16_t u16AngleY;			//Y轴角度
	rt_int16_t u16AngleZ;			//Z轴角度
}app_tof_Tunnel_custom_data_ts;


typedef struct
{	
	rt_int16_t u16DeviceNum;		//设备号(基站号)	
	rt_int16_t u16CardNum;			//卡号	
	rt_int16_t u16SeqNum;       		//卡序列号	
	rt_int8_t u8Status; 				//卡姿态		
	rt_int8_t u8CardType;				//卡类型	
	rt_int8_t u8CardStatus;			//卡状态
	rt_int8_t u8SignalIntensity;  		//卡信号
	rt_int16_t u16Battery;			//卡电量	
	rt_int16_t u16AngleX;				//x轴姿态	
	rt_int16_t u16AngleY;				//y轴姿态	
	rt_int16_t u16AngleZ;				//z轴姿态	
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
