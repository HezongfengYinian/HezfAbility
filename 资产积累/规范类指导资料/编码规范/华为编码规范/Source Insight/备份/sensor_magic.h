#include <stdio.h>
#include <mbus_app.h>

#define SENSORHEAD    		0x7E
#define monitor_maxtype     10
/*
#define CH4_SENSOR         1
#define CO_SENSOR          2
#define O2_SENSOR          3
#define T_SENSOR           4
#define WIND_SENSOR        5
#define P_SENSOR           6*/

typedef enum
{
        COMM_DEFAULT = 0,
        COMM_MANUAL_OUTAGE = 111, //手动断电
        COMM_MANUAL_RESET = 2, //手动复电
        COMM_AUTO_OUTAGE_CONTROL = 3, //自动断电
        COMM_AUTO_RESET_CONTROL = 4, //自动复电
        COMM_SENSOR_INFO = 5, //传感器基本信息包
        COMM_POLL_SENSOR = 6, //传感器轮询包
        COMM_VALUE_POSITION_SENSOR = 7, //传感器value位置包
        COMM_OUTAGE_SENSOR = 8, //传感器断电包
        COMM_RESET_SENSOR = 9, //传感器复电包
        COMM_DOWN_ANALOG_UNTRALIMIT_CONFIG = 10, //下发模拟量超限报警/断电配置
        COMM_DOWN_SWITCH_STATUS_CONFIG = 11, //下发开关量状态异常报警/断电配置
        COMM_DOWN_CH4_LOCK_CONFIG = 12, //下发甲烷风电闭锁配置
        //COMM_LIGHT_WARN_CONTROL = 13, //声光报警
        COMM_DOWN_COORDINATED_CONFIG = 14, //下发联动配置包
        COMM_DOWN_COORDINATED_CONFIG_DELETE = 15, //下发联动配置删除命令
        COMM_UP_SITE_INFO = 100, //分站信息上报
        COMM_UP_SENSOR_DATA = 101, //传感器数据上报

        COMM_DOWN_TIME_SYNC = 200, //时间同步  
}app_recv_sensor_msgtype;

typedef enum
{
	CH4_SENSOR=0x1,   //甲烷传感器
	CO_SENSOR=0x2,    //CO 传感器
	O2_SENSOR= 0x3,   //氧气传感器
	T_SENSOR =0x4,    //温度传感器
	WIND_SENSOR =0x5, //风速传感器
	P_SENSOR=0x6,     //压差传感器
}Sensor_Type;


typedef struct
{
	unsigned char head;       //先导符  先导符---------传感器类型
	unsigned char type;       //传感器类型
	//unsigned char name_addr;  //传感器地址号
	unsigned char subaddr;    //传感器地址
	unsigned char company;     //生产厂家
	
}Monitor_msg;


typedef struct
{
	Monitor_msg monitor;
	unsigned char value;     //生产厂家
	unsigned char ERR;     //生产厂家
	
}Monitor_Port_list;

typedef struct       //解析传感器的内容值
{
	unsigned char head;//先导符
	char type_bit;     //sensor's type bit
	char type_len;     //sensor's type length
	char value_bit;    //sensor's value bit
	char value_len;    //sensor's value length
	char statue_bit;   //sensor's statue bit
	char statue_len;   //sensor's statue length
	char CRC_bit;      //sensor's CRC bit
	char CRC_len;      //sensor's CRC length
}Monitor_Analysis;


typedef struct       //传感器断电复电信息
{
	char port;       //端口号
    char addr;       //传感器地址
    char payload[13];
}Monitor_on_or_off;















