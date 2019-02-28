
#ifndef _LCD_SPI_H_
#define _LCD_SPI_H_
#include "stdint.h"
#include "bootcfg.h"

#define minitor_x  0
#define minitor_y  0
#define radio_x  1
#define radio_y  0
#define locate_x  2
#define locate_y  0

#define electric_interface1_x 0
#define electric_interface1_y 1
#define electric_interface2_x 1
#define electric_interface2_y 1
#define electric_interface3_x 2
#define electric_interface3_y 1
#define electric_interface4_x 3
#define electric_interface4_y 1
#define electric_interface5_x 4
#define electric_interface5_y 1
#define electric_interface6_x 5
#define electric_interface6_y 1
#define electric_interface7_x 6
#define electric_interface7_y 1
#define electric_interface8_x 7
#define electric_interface8_y 1

#define _100MB_light1_x 0
#define _100MB_light1_y 2
#define _100MB_light2_x 1
#define _100MB_light2_y 2
#define _100MB_light3_x 2
#define _100MB_light3_y 2
#define _100MB_light4_x 3
#define _100MB_light4_y 2
#define _1000MB_light1_x 4
#define _1000MB_light1_y 2
#define _1000MB_light2_x 5
#define _1000MB_light2_y 2
#define _1000MB_light3_x 6
#define _1000MB_light3_y 2
#define _1000MB_light4_x 7
#define _1000MB_light4_y 2
#define COM1_x 8
#define COM1_y 2
#define COM2_x 9
#define COM2_y 2
#define COM3_x 10
#define COM3_y 2
#define COM4_x 11
#define COM4_y 2


struct COORDINATED_CONDITION
{
	uint8_t ConditionLen;
	uint8_t reserved[3];	
};


struct COORDINATED_CONDITION_CONFIG
{
	uint8_t port;
	uint8_t subAddr;
	uint8_t reserve;
	uint8_t symbol;
	uint32_t value;
};

struct COORDINATED_ACTION       
{
	uint8_t ActionLen;
	uint8_t SensorWarn;
	uint8_t Evacuate;
	uint8_t Broadcast;
};

struct EVACUATE_PACKAGE     //撤退包    
{
	uint8_t EvacuateListLen;
	uint8_t reserved[3];
};

struct EVACUATE_INFORMATION	 //撤退包 信息	
{
	uint16_t EvacuateSiteNum;
	uint8_t EvacuateType;
	uint8_t reserved;
};

struct BROADCAST_PACKAGE      //广播包信息
{
	uint8_t BroadcastListLen;
	uint8_t reserved[3]; 
};

struct BROADCAST_INFORMATION 
{
	uint16_t BroadcastSiteNum;
	uint8_t BroadcastType;
	uint8_t reserved;
};

struct COORDINATED_DELECT
{
	uint8_t port;
	uint8_t subAddr;
	uint16_t reserved;
};

struct COORDINATED_ACK
{
	uint8_t state;
	uint8_t Len;
	uint16_t reserved;
};

struct COORDINATED_ACK_INFORMATION
{
	uint8_t port;
	uint8_t subAddr;
	uint16_t reserved;
};


struct COORDINATED_CONFIG
{
	struct COORDINATED_CONDITION coordinated_condition;
	struct COORDINATED_CONDITION_CONFIG coordinated_condition_config[5];
	struct COORDINATED_ACTION  coordinated_action;   
	struct EVACUATE_PACKAGE evacuate_pack;
	struct EVACUATE_INFORMATION evacuate_information[6];
	struct BROADCAST_PACKAGE broadcast_pack;
	struct BROADCAST_INFORMATION broadcast_information[6];	
};


extern struct rt_messagequeue coordinated_mq;


void gpio_test(void);
void LCD_QuitWinMode();
void test_lcd(int count);
void time_show();
float ConverToByeFloat(uint8_t *tran);
void ConvertToByte(uint8_t *sensor_result,float dvalue) ;
void String(unsigned char *str);
void sensor_type_addr_show(int x,int y,char type,char addr,float data,char state);
void save_coordinateddata_to_flash(uint8_t *pdata,uint32_t num,uint16_t len);
void flash_data_config(uint8_t *pdata);
void LCD_FUNCTION_STATE_SET(uint8_t x_order,uint8_t y_order,uint8_t state);
void station_status_monitoring(void );
void coordinated_build_packet(uint16_t station_id,uint8_t protocoltype ,uint8_t msgtype,uint8_t voice_type);

#endif

