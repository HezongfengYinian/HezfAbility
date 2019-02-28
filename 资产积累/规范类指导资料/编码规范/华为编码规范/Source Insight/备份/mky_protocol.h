#include <stdio.h>
#include <stdint.h>
#define CAN_HENDER_LEN       5



typedef enum
{
	NORMAL=0,           //����
	SOME_ERR=1,         //�й���
	SENSOR_OFF_LINE=2,  //���������ߣ���վ�ã�
	INTERLOCK=3,        //��������վ�ã�
	SENSRO_BREAK=4,     //������ͻ�䣨��վ�ã�
	ALARM=5,            //������led���ã�
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
	char mode:1;            //ģʽ
	char RTP:1;             //Զ��֡
	char reserves:2;        //CAN����
	char len:4;             //���ݳ���
	char priority:3;        //���ȼ�
	char sensor_type:6;     //������������
	char sensor_addr:7;     //��������ַ��
	char DIR:1;             //����λ
	char frame_type:2;      //֡������
	char frame_len:5;       //����֡����
	char command:5;         //������   MKY_COMMAND
	char can_reseves:3;     //����
    char data[8];             //����   
}MKY_CAN_protocol;





extern MKY_CAN_protocol can_sensor_header[18];
//extern SENSOR_DATA sensor_data[18];

void CAN_HEADER_save(unsigned int id , int RxBuf,unsigned char port);


