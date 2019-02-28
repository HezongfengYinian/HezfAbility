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
        COMM_MANUAL_OUTAGE = 111, //�ֶ��ϵ�
        COMM_MANUAL_RESET = 2, //�ֶ�����
        COMM_AUTO_OUTAGE_CONTROL = 3, //�Զ��ϵ�
        COMM_AUTO_RESET_CONTROL = 4, //�Զ�����
        COMM_SENSOR_INFO = 5, //������������Ϣ��
        COMM_POLL_SENSOR = 6, //��������ѯ��
        COMM_VALUE_POSITION_SENSOR = 7, //������valueλ�ð�
        COMM_OUTAGE_SENSOR = 8, //�������ϵ��
        COMM_RESET_SENSOR = 9, //�����������
        COMM_DOWN_ANALOG_UNTRALIMIT_CONFIG = 10, //�·�ģ�������ޱ���/�ϵ�����
        COMM_DOWN_SWITCH_STATUS_CONFIG = 11, //�·�������״̬�쳣����/�ϵ�����
        COMM_DOWN_CH4_LOCK_CONFIG = 12, //�·��������������
        //COMM_LIGHT_WARN_CONTROL = 13, //���ⱨ��
        COMM_DOWN_COORDINATED_CONFIG = 14, //�·��������ð�
        COMM_DOWN_COORDINATED_CONFIG_DELETE = 15, //�·���������ɾ������
        COMM_UP_SITE_INFO = 100, //��վ��Ϣ�ϱ�
        COMM_UP_SENSOR_DATA = 101, //�����������ϱ�

        COMM_DOWN_TIME_SYNC = 200, //ʱ��ͬ��  
}app_recv_sensor_msgtype;

typedef enum
{
	CH4_SENSOR=0x1,   //���鴫����
	CO_SENSOR=0x2,    //CO ������
	O2_SENSOR= 0x3,   //����������
	T_SENSOR =0x4,    //�¶ȴ�����
	WIND_SENSOR =0x5, //���ٴ�����
	P_SENSOR=0x6,     //ѹ�����
}Sensor_Type;


typedef struct
{
	unsigned char head;       //�ȵ���  �ȵ���---------����������
	unsigned char type;       //����������
	//unsigned char name_addr;  //��������ַ��
	unsigned char subaddr;    //��������ַ
	unsigned char company;     //��������
	
}Monitor_msg;


typedef struct
{
	Monitor_msg monitor;
	unsigned char value;     //��������
	unsigned char ERR;     //��������
	
}Monitor_Port_list;

typedef struct       //����������������ֵ
{
	unsigned char head;//�ȵ���
	char type_bit;     //sensor's type bit
	char type_len;     //sensor's type length
	char value_bit;    //sensor's value bit
	char value_len;    //sensor's value length
	char statue_bit;   //sensor's statue bit
	char statue_len;   //sensor's statue length
	char CRC_bit;      //sensor's CRC bit
	char CRC_len;      //sensor's CRC length
}Monitor_Analysis;


typedef struct       //�������ϵ縴����Ϣ
{
	char port;       //�˿ں�
    char addr;       //��������ַ
    char payload[13];
}Monitor_on_or_off;















