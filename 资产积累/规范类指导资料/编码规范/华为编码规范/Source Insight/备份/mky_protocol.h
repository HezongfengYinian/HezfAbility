#include <stdio.h>
#include <stdint.h>
#define CAN_HENDER_LEN       5



typedef enum
{
	NORMAL=0,           //正常
	SOME_ERR=1,         //有故障
	SENSOR_OFF_LINE=2,  //传感器断线（分站用）
	INTERLOCK=3,        //闭锁（分站用）
	SENSRO_BREAK=4,     //传感器突变（分站用）
	ALARM=5,            //报警（led屏用）
}MKY_SENSOR_ERR;

typedef enum
{
	CH4_4P =1,
	CH4_40P =2,
	
}MKY_SENSOR_TYPE;


typedef struct 
{
    char state;
    char port;
	char mode:1;            //模式
	char RTP:1;             //远程帧
	char reserves:2;        //CAN保留
	char len:4;             //数据长度
	char priority:3;        //优先级
	char sensor_type:6;     //传感器类型码
	char sensor_addr:7;     //传感器地址码
	char DIR:1;             //方向位
	char frame_type:2;      //帧类型码
	char frame_len:5;       //后续帧长度
	char command:5;         //命令码   MKY_COMMAND
	char can_reseves:3;     //保留
    char data[8];             //数据   
}MKY_CAN_protocol;





extern MKY_CAN_protocol can_sensor_header[18];
//extern SENSOR_DATA sensor_data[18];

void CAN_HEADER_save(unsigned int id , int RxBuf,unsigned char port);


