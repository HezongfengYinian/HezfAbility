/*
 * mcp2515.c
 * Common Function Definition for mcp2515 CAN Controller
 * Date	: 2004/04/29
 * By	: Zhang Jing
 */
#define	MCP2515_FLAG
#include	"mcp2510.h"
#include "mky_protocol.h"
//#include	"spi_cmd.h"
#include "board.h"
//#include    "spi.h"
#include "mbus_app.h"
#include "msg_center.h"
#include "sensor_magic.h"
#include "math.h"
#include "stm32_flash.h"
#include "bootcfg.h"
#include "bsmac_parser.h"


struct SENSOR_RETURN_ACK
{
	uint8_t port;
	uint8_t addr;
	uint8_t reserved;
	uint8_t state;
};

struct SENSOR_LOCK_D_ACK
{
	uint8_t state;
	uint8_t sign;
	uint16_t reserved;
	uint8_t fan_port;
	uint8_t fan_addr;	
	uint8_t ram_air_port;
	uint8_t ram_air_addr;
};


struct SENSOR_LOCK_E_ACK
{
	uint8_t state;
	uint8_t sign;
	uint8_t fan_port;
	uint8_t fan_addr;
	uint8_t heading_face_port;
	uint8_t heading_face_addr;
	uint8_t return_current_port;
	uint8_t return_current_addr;
};




struct SENSOR_INTERRUPTER
{
	uint8_t port;
	uint8_t addr;
	uint16_t reserve;
	uint8_t payload[13];
};

struct SENSOR_INFORMATION
{
	uint8_t port;
	uint8_t addr;
	uint16_t reserve;
};


struct SENSOR_CONFIGURE
{
	uint8_t port;
	uint8_t addr;
	uint16_t reserve;
	uint32_t alarm_up;
	uint32_t alarm_down;
	uint32_t blackout_up;
	uint32_t blackout_down;
	uint32_t recover_up;
	uint32_t recover_down;
};

struct MINITOR_LOCK_D
{
	uint8_t reserve[3];
	uint8_t sign;
	uint8_t sensor_fan_port;
	uint8_t sensor_fan_addr;
	uint8_t sensor_fan_value;
	uint8_t reserve1;  
	uint8_t sensor_airduct_port;
	uint8_t sensor_airduct_addr;
	uint8_t sensor_airduct_value;
	uint8_t reserve2;
	struct SENSOR_INFORMATION psensor_lock_d_blackout[6];   //甲烷风电闭锁d断电
};

  
struct MINITOR_LOCK_E
{
	uint8_t reserve[3];
	uint8_t sign;
	uint32_t blackout_lock_len;
	uint8_t sensor_fan_port;
	uint8_t sensor_fan_addr;
	uint8_t sensor_fan_value;
	uint8_t reserve1; 

	uint8_t sensor_drivingface_up_port;
	uint8_t sensor_drivingface_up_addr;
	uint16_t reserve2;
	uint32_t sensor_drivingface_up_value;

	uint8_t sensor_returncurrent_up_port;
	uint8_t sensor_returncurrent_up_addr;
	uint16_t reserve3;
	uint32_t sensor_returncurrent_up_value;

	uint32_t reply_unlock_len;

	uint8_t sensor_drivingface_down_port;
	uint8_t sensor_drivingface_down_addr;
	uint16_t reserve4;
	uint32_t sensor_drivingface_down_value;

	uint8_t sensor_returncurrent_down_port;
	uint8_t sensor_returncurrent_down_addr;
	uint16_t reserve5;
	uint32_t sensor_returncurrent_down_value;   

	struct SENSOR_INFORMATION psensor_lock_e_blackout[6];   //甲烷风电闭锁e断电
};





struct MINITOR_ALARM_DATA
{
	uint8_t port;
	uint8_t addr;
	uint16_t reserve;
	uint32_t alarm_up;
	uint32_t alarm_down;
	uint32_t blackout_up;
	uint32_t blackout_down;
	uint32_t recover_up;
	uint32_t recover_down;
	uint32_t num_up; 
	struct SENSOR_INFORMATION psensor_alarm_blackout_up[6];   	   //断电上限
	uint32_t num_down; 
	struct SENSOR_INFORMATION psensor_alarm_blackout_down[6];      //断电下限
};


struct MINITOR_SWITCH_DATA
{
	uint8_t port;
  	uint8_t addr;
    	uint8_t alarm_state;
    	uint8_t reserve;	
	uint32_t num_onestate;
	struct SENSOR_INFORMATION psensor_switch_onestate_blackout[6];   	 	//开关量一态断电
	uint32_t num_secondstate;
	struct SENSOR_INFORMATION psensor_switch_secondstate_blackout[6];           //开关量二态断电
};


struct MINITOR_INFORMATION
{
	unsigned char port;
	unsigned char addr;
	unsigned char type;
	unsigned char reserve;
};

struct SENSOR_SWITCH_CONFIGURE
{
    uint8_t port;
    uint8_t addr;
    uint8_t alarm_state;
    uint8_t reserve;
};


typedef union                                        
{
   float Temp;
   unsigned char  Buf[4];
}DtformConver;


//软件定时延时n毫秒（不精准）
void Delay_ms(unsigned int n)
{
	unsigned int m;

	for (;n>0;n--)
		for (m=0;m<100;m++);
}


void ConvertToByte(uint8_t *sensor_result,float dvalue)   //将1个浮点型的数据转换为3个字节的数据   
{
    unsigned char  kk;
	sensor_result[0] = sensor_result[1] = sensor_result[2] = 0;

	if(dvalue==0)
	{
		sensor_result[0] = 0x41;
	}
	else
	{
		if(dvalue<0)
		{
			sensor_result[0] = 0x80;
			dvalue = - dvalue;
		}
		
		if(dvalue >= 1.0)
		{
			int radix=1;
			double itemp = dvalue;
			while((itemp=itemp/2.0) >= 1.0)
			{
				radix += 1;
			}
			sensor_result[0] += radix;
			
			dvalue = dvalue/pow(2,radix);
			for( kk=0;kk<16;kk++)
			{
				if(((dvalue*2)-1) >=0)
				{
					if(kk<8) sensor_result[1] += (int)pow(2,8-kk-1);
					else    sensor_result[2] += (int)pow(2,16-kk-1);
					dvalue = dvalue*2-1;
				}
				else
				{
					dvalue = dvalue*2;
				}
			}
			
		}
		else
		{
			int radix=0;
			double dtemp = dvalue;
			while((dtemp=dtemp*2)<1)
			{
				radix += 1;
			}
			dvalue = dvalue * pow(2,radix);
			if(radix>0) 
			{
				radix = (radix ^ 0x3F) + 1 ;	//阶码为负数时采用反码
				sensor_result[0] = sensor_result[0] + 0x40 + radix;
			}
			
			for( kk=0;kk<16;kk++)
			{
				if(((dvalue*2)-1) >=0)
				{
					if(kk<8) sensor_result[1] += (int)pow(2,8-kk-1);
					else    sensor_result[2] += (int)pow(2,16-kk-1);
					dvalue = dvalue*2-1;
				}
				else
				{
					dvalue = dvalue*2;
				}
			}
		}		
	}
}    


float ConverToByeFloat(uint8_t *tran)     //将3个字节的数据转换为1个浮点型的数据 
{
    float ddvalue;
	unsigned int immm = tran[1] *256 + tran[2];
	int signal=((tran[0] & 0x80)>0?-1:1);//符号
	int radixsignal=((tran[0] & 0x40)>0?-1:1);//阶码符号
	int radix = tran[0] & 0x3F;
	if(radixsignal == -1)//阶码(补码)是负的,转换补码成为原码
	radix = (radix ^ 0x3F) + 1;
	radix = radix * radixsignal;
	ddvalue = signal * immm * pow(2,(radix-16));
	return(ddvalue);
}


float int_to_float(int data)
{
	DtformConver   data_change;
	unsigned char i;
	int i_Temp32;
	float f_Temp;	
	
 	//IEEE754标准转换成十进制float
	i_Temp32 = data;        
 	rt_memset((unsigned char *)&data_change.Buf[0],0,4);
  	for(i=0;i<4;i++)
 	{
   		data_change.Buf[i] = (unsigned char)(i_Temp32>>(i*8));
 	}
 	f_Temp = data_change.Temp;
 	return f_Temp;
}


int float_to_int(float data)
{
	DtformConver   data_change;
	unsigned char i;
	int i_Temp32;
	
	//十进制float型数据转换成IEEE754标准
	data_change.Temp = data;         
	i_Temp32=0;
 	for(i=0;i<4;i++)
 	{
  	  i_Temp32  |= (int)(data_change.Buf[i]<<(i*8));
 	}
 	return i_Temp32;
}




//给服务器回应  state=1表示接受命令正确  state=0表示接受命令错误  
void minitor_send_ack(uint8_t *buf,int size)
{	
    rt_err_t err = 0;
 
	err = rt_mq_send(&bsmac_mq,buf,size);
	if(err == -RT_EFULL)
	{
		rt_kprintf("send to bsmac_mq full\n");
	}
	else if(err == -RT_ERROR)
	{
		rt_kprintf("send to bsmac_mq err\n");
	}
}




unsigned char SPI_Write_Can(unsigned char dat)
{
	while (SPI_GetFlagStatus(SPI3, SPI_FLAG_TXE) == RESET);
	SPI_SendData(SPI3,dat);
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_ReceiveData(SPI3); //????SPIx???????	
}

unsigned char SPI_Read_Can(unsigned char dat)
{
	while (SPI_GetFlagStatus(SPI3, SPI_FLAG_TXE) == RESET);
	SPI_SendData(SPI3,dat);
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_ReceiveData(SPI3); //????SPIx???????	
}

void ID_Send( int IdType,uint32_t id ,unsigned char port,int offset)
{ 
    char temp;
    if(IdType == EXTID)
    {        
        temp = id >> 21;
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->TXB0SIDH))+offset, temp, ARG_UNUSED ,port);
        temp = ((id>>16)&0x03) | (((id>>18)&0x07)<<5)  | 0x08 ; 
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->TXB0SIDL))+offset, temp, ARG_UNUSED ,port);
        temp = id>>8; 
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->TXB0EID8))+offset, temp, ARG_UNUSED ,port);
        temp = id; 
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->TXB0EID0))+offset, temp, ARG_UNUSED ,port);
    }
  
}

int ID_Receive(int RxBuf,unsigned char port)
{
    uint8_t type;
    uint32_t id = 0;
    char id_buf[4];
    int offset = 0;
    
    switch( RxBuf ){
		case RXBUF0:
			offset = 0;
			break;
		case RXBUF1:
			offset = 0x10;
			break;
	}
          
    type = CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB0SIDL))+offset, ARG_UNUSED , ARG_UNUSED ,port);
    
    if(type & 0x08)
    {
        id_buf[0] = CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB0SIDH))+offset, ARG_UNUSED, ARG_UNUSED ,port);
        id_buf[1] = CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB0SIDL))+offset, ARG_UNUSED, ARG_UNUSED ,port );
        id_buf[2] = CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB0EID8))+offset, ARG_UNUSED, ARG_UNUSED ,port);
        id_buf[3] = CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB0EID0))+offset, ARG_UNUSED, ARG_UNUSED ,port);
                      
        id = ((uint32_t)id_buf[0])<<21;
		id |= ((uint32_t)((id_buf[1]&0xe0)>>5)<<18) | ((id_buf[1]&0x03)<<16) ;
		id |= ((uint32_t)id_buf[2])<<8;
		id |= (uint32_t)id_buf[3];
        
        return id;
                
    }
   
    return 0;
}

unsigned char CAN_SPI_CMD( unsigned char cmd, unsigned long addr, unsigned char arg1, unsigned char arg2 , unsigned char port )
{
	unsigned char ret=0;

    select_port1(port,ENABLE);
	switch( cmd ){
		case SPI_CMD_READ:
			
			ret = SPI_Write_Can(SPI_CMD_READ);
			ret = SPI_Write_Can(addr);					
			ret = SPI_Write_Can(0xFF);	
			break;
			
		case SPI_CMD_WRITE:
		
			ret = SPI_Write_Can(SPI_CMD_WRITE);
			ret = SPI_Write_Can(addr);
			ret = SPI_Write_Can(arg1);	
						
			break;
		case SPI_CMD_RTS:
			
			ret = SPI_Write_Can(SPI_CMD_RTS);
			
			break;
		case SPI_CMD_READSTA:
			
			ret = SPI_Write_Can(SPI_CMD_READSTA);
			ret = SPI_Write_Can(0xFF);			
			
			break;
		case SPI_CMD_BITMOD:
			
			ret = SPI_Write_Can(SPI_CMD_BITMOD);
			ret = SPI_Write_Can(addr);
			ret = SPI_Write_Can(arg1);	
			ret = SPI_Write_Can(arg2);				
			
			break;
		case SPI_CMD_RESET:
			ret = SPI_Write_Can(SPI_CMD_RESET);
			rt_kprintf("SPI_CMD_RESET----ret=%d ---1  \n",ret);
			break;
		default:
			ret = 0x30;										// any value is ok
	}
	
	select_port1(port,DISABLE);
	return ret;
}


/* Initialize the mcp2515 */
void 
MCP2515_Init()
{
    unsigned char port = 1;
    for(port=1;port<=4;port++)
    {
        // Reset controller
        CAN_SPI_CMD( SPI_CMD_RESET, ARG_UNUSED, ARG_UNUSED, ARG_UNUSED ,port );
        rt_kprintf("MCP2515_Init--------1  \n");
        
        CAN_SPI_CMD( SPI_CMD_BITMOD, TOLONG(&(MCP2515_MAP->CANCTRL)), 0xe0, 0x80 ,port);		// CONFIGUE MODE
        
        // make sure we are in configuration mode
        while((CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->CANSTAT)), ARG_UNUSED, ARG_UNUSED ,port))!=0x80)
        {
            CAN_SPI_CMD( SPI_CMD_BITMOD, TOLONG(&(MCP2515_MAP->CANCTRL)), 0xe0, 0x80 ,port);		// CONFIGUE MODE
        };
        rt_kprintf("MCP2515_Configure_mode------------  \n");
     
        //设置波特率为5Kbps
        //set CNF1,SJW=00,长度为1TQ,BRP=49,TQ=[2*(BRP+1)]/Fsoc=2*50/8M=12.5us
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->CNF1)), 	0x31, 	ARG_UNUSED,port);
        //set CNF2,SAM=0,在采样点对总线进行一次采样，PHSEG1=(6+1)TQ=7TQ,PRSEG=(1+1)TQ=2TQ
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->CNF2)), 	0xb1, 	ARG_UNUSED,port);
        //set CNF3,PHSEG2=(5+1)TQ=6TQ,同时当CANCTRL.CLKEN=1时设定CLKOUT引脚为时间输出使能位
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->CNF3)), 	0x05, 	ARG_UNUSED,port);

        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->TXB0SIDH)), 	0xff, 	ARG_UNUSED,port);    //发送缓冲器0标准标识符高位
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->TXB0SIDL)), 	0xeb, 	ARG_UNUSED,port);    //发送缓冲器0标准标识符低位(第3位为发送拓展标识符使能位)
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->TXB0EID8)), 	0xff, 	ARG_UNUSED,port);    //发送缓冲器0拓展标识符高位
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->TXB0EID0)), 	0xff, 	ARG_UNUSED,port);    //发送缓冲器0拓展标识符低位 

        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->TXB1SIDH)), 	0xff, 	ARG_UNUSED,port);    //发送缓冲器1标准标识符高位
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->TXB1SIDL)), 	0xeb, 	ARG_UNUSED,port);    //发送缓冲器1标准标识符低位(第3位为发送拓展标识符使能位)
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->TXB1EID8)), 	0xff, 	ARG_UNUSED,port);    //发送缓冲器1拓展标识符高位
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->TXB1EID0)), 	0xff, 	ARG_UNUSED,port);    //发送缓冲器1拓展标识符低位    
    
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->RXB0SIDH)), 	0x00, 	ARG_UNUSED,port);    //清空接收缓冲器0的标准标识符高位
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->RXB0SIDL)), 	0x00, 	ARG_UNUSED,port);    //清空接收缓冲器0的标准标识符低位
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->RXB0EID8)), 	0x00, 	ARG_UNUSED,port);    //清空接收缓冲器0的拓展标识符高位
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->RXB0EID0)), 	0x00, 	ARG_UNUSED,port);    //清空接收缓冲器0的拓展标识符低位
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->RXB0CTRL)), 	0x64, 	ARG_UNUSED,port);    //接收拓展和标准标识符的有效信息（不过滤）滚存
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->RXB0DLC)), 	0x08, 	ARG_UNUSED,port);    //设置接收数据的长度为8个字节
	
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->RXB1SIDH)), 	0x00, 	ARG_UNUSED,port);    //清空接收缓冲器1的标准标识符高位
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->RXB1SIDL)), 	0x00, 	ARG_UNUSED,port);    //清空接收缓冲器1的标准标识符低位
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->RXB1EID8)), 	0x00, 	ARG_UNUSED,port);    //清空接收缓冲器1的拓展标识符高位
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->RXB1EID0)), 	0x00, 	ARG_UNUSED,port);    //清空接收缓冲器1的拓展标识符低位
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->RXB1CTRL)), 	0x60, 	ARG_UNUSED,port);    //接收拓展和标准标识符的有效信息（不过滤）
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->RXB1DLC)), 	0x08, 	ARG_UNUSED,port);    //设置接收数据的长度为8个字节 
        
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->CANINTF)), 	0x00, 	ARG_UNUSED,port);    //清空CAN中断标志寄存器的所有位(必须由MCU清空)
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->CANINTE)), 	0x03, 	ARG_UNUSED,port);    //配置CAN中断使能寄存器的接收缓冲器0满中断使能,其它位禁止中断
            
        CAN_SPI_CMD( SPI_CMD_BITMOD, TOLONG(&(MCP2515_MAP->CANCTRL)), 0xe4, 0x04 ,port);	//将MCP2515设置为正常模式,退出配置模式
    
        while((CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->CANSTAT)), ARG_UNUSED, ARG_UNUSED,port)&0xe0)!=0x00)
        {
            CAN_SPI_CMD( SPI_CMD_BITMOD, TOLONG(&(MCP2515_MAP->CANCTRL)), 0xe4, 0x04 ,port);	// NORMAL OPERATION MODE and wake interrupt
        };
        rt_kprintf("PORT %d :MCP2515_NORMAL_MODE------------\n",port);
    }
}
/* Transmit data */
/*
 TxBuf	: select the transmit buffer( 0=buffer0 or 1=buffer1 2=buffer2 )
 IdType	: 0=standard id or 1=extended id
 id	: frame identifier
 DataLen	: the number of byte
 data	: the pointer to data byte
*/

void MCP2515_TX(int TxBuf, int IdType, unsigned int id, int DataLen, uint8_t *data,unsigned char port)
{	
    int offset = 0;
	unsigned char i,time;   
   
	switch( TxBuf ){
		case TXBUF0:
			offset = 0;
			break;
		case TXBUF1:
			offset = 0x10;
			break;
		case TXBUF2:
			offset = 0x20;
			break;
	}

    ID_Send(IdType,id,port,offset);
    CAN_SPI_CMD(SPI_CMD_WRITE,TOLONG(&(MCP2515_MAP->TXB0CTRL))+offset,0x00 ,ARG_UNUSED, port); 
	time=0;
	while((CAN_SPI_CMD(SPI_CMD_READ,TOLONG(&(MCP2515_MAP->TXB0CTRL))+offset,ARG_UNUSED,ARG_UNUSED,port)&0x08) )//快速读某些状态指令,等待TXREQ标志清零
	{
		Delay_ms(1);//通过软件延时约nms(不准确)
		time++;
	} 			
	for(i=0;i<8;i++)
	{
        CAN_SPI_CMD(SPI_CMD_WRITE,TOLONG(&(MCP2515_MAP->TXB0D0))+offset+i,data[i] ,ARG_UNUSED , port);
	}
	//将本帧待发送的数据长度写入发送缓冲器0的发送长度寄存器
    CAN_SPI_CMD(SPI_CMD_WRITE,TOLONG(&(MCP2515_MAP->TXB0DLC))+offset,DataLen ,ARG_UNUSED, port);
        
    //请求发送报文
	CAN_SPI_CMD(SPI_CMD_WRITE,TOLONG(&(MCP2515_MAP->TXB0CTRL))+offset,0x08 ,ARG_UNUSED, port);        
	
}






unsigned char select_port1(unsigned char port,unsigned char type)
{
	if(type == ENABLE)
	{
		if(port ==1)
			GPIO_ResetBits(SPI1_CS1_GPIO_PORT,SPI1_CS1_PIN);
		else if(port == 2)
			GPIO_ResetBits(SPI1_CS2_GPIO_PORT,SPI1_CS2_PIN);
		else if(port == 3)
			GPIO_ResetBits(SPI1_CS3_GPIO_PORT,SPI1_CS3_PIN);
		else if(port == 4)
			GPIO_ResetBits(SPI1_CS4_GPIO_PORT,SPI1_CS4_PIN);
		else if(port == 5)
			GPIO_ResetBits(SPI1_CS5_GPIO_PORT,SPI1_CS5_PIN);
		else if(port == 6)
			GPIO_ResetBits(SPI1_CS6_GPIO_PORT,SPI1_CS6_PIN);
	}
	else if(type ==DISABLE)
	{
		if(port ==1)
			GPIO_SetBits(SPI1_CS1_GPIO_PORT,SPI1_CS1_PIN);
		else if(port == 2)
			GPIO_SetBits(SPI1_CS2_GPIO_PORT,SPI1_CS2_PIN);
		else if(port == 3)
			GPIO_SetBits(SPI1_CS3_GPIO_PORT,SPI1_CS3_PIN);
		else if(port == 4)
			GPIO_SetBits(SPI1_CS4_GPIO_PORT,SPI1_CS4_PIN);
		else if(port == 5)
			GPIO_SetBits(SPI1_CS5_GPIO_PORT,SPI1_CS5_PIN);
		else if(port == 6)
			GPIO_SetBits(SPI1_CS6_GPIO_PORT,SPI1_CS6_PIN);
	}
	return 1;
}


 
char lock_state_d1 = 1;
char lock_state_d2 = 1;
 
char lock_state_e1 = 1;
char lock_state_e2 = 1;
char lock_state_e3 = 1;
char lock_state_e4 = 1;
char lock_state_e5 = 1;


//传感器下发数据信息存储
char sensor_config_port;
char sensor_config_addr;
uint32_t alarm_up;
uint32_t alarm_down;

//开关量传感器下发配置信息存储 
uint8_t alarm_state;


struct MINITOR_ALARM_DATA minitor_data[18];
struct MINITOR_SWITCH_DATA minitor_switch_data[8];
struct MINITOR_LOCK_D minitor_lock_d;
struct MINITOR_LOCK_E minitor_lock_e;



uint8_t alarm_data_num =0;
uint8_t switch_data_num = 0;
uint8_t lock_d_num = 0;
uint8_t lock_e_num = 0;



unsigned char save_num = 0;
unsigned char minitor_num = 0;
unsigned char minitor_switch_num = 0;

//flash 存储信息
int flash_read_state = 0;
CFG_OPTION_T save_option ;
short int sensor_all_len = 0;
char save_type;
short int sensor_type_len;
short int sensor_alarm_blackout_len = 0;
short int sensor_switch_state_len = 0;
short int sensor_lock_d_len = 0;
short int sensor_lock_e_len = 0;

char sensor_lock_d_buff[128];
char sensor_lock_e_buff[128];


static unsigned int id[2];
int sensor_num = 0;


/* Receive data */
/*
 * RxBuf		: The receive buffer from which the data is get
 * IdType		: Identifier type of the data frame ( STANDID, EXTID )
 * id		: The identifier of the received frame
 * DataLen	: The number of bytes received
 * data		: The received data
 */
void MCP2515_RX(unsigned char *data1,unsigned char *data2,unsigned char port)
{
    unsigned char i=0,len=0;
    char ret;
    char mode,RTP;
    char temp_type;
    ret = CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->CANINTF)),ARG_UNUSED,ARG_UNUSED, port)&0x0f;//读取中断使能寄存器
    ret &= 0x03;
    if(ret)
    {
        if(ret == 0x01)
        {
            len=CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB0DLC)),ARG_UNUSED,ARG_UNUSED, port)&0x0f;//读取接收缓冲器0接收到的数据长度(0~8个字节)    
            mode = (CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB0SIDL)),ARG_UNUSED,ARG_UNUSED, port)&0x08)<<4;  //0:标准帧  1:扩展帧
            RTP = CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB0DLC)),ARG_UNUSED,ARG_UNUSED, port)&0x40;//0:扩展数据帧  1:扩展远程帧
            while(i<len)
            {	
                //把接收缓冲区里的数据，放到内部RAM缓冲区
                data1[i]=CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB0D0))+i,ARG_UNUSED,ARG_UNUSED, port);
                i++;
            }
            rt_kprintf(" PORT %d ：RXBUF0 :",port);
            for(i=0;i<len;i++)
            {
                rt_kprintf("  %d ,",data1[i]);
            }
            rt_kprintf(" \n");  
            
            if( len > 0)
            {
                id[0] = ID_Receive(RXBUF0,port); 
                temp_type = (id[0]>> 20) & 0x3f ;
                if((temp_type >= 1) && (temp_type <= 53))
                {
                    sensor_data[sensor_num].port = port;
                    sensor_data[sensor_num].type = (id[0] >> 20) & 0x3f;
                    sensor_data[sensor_num].subAddr = (id[0] >> 13) & 0x7f;
                    sensor_data[sensor_num].subLen = len + 5;
                    
                    sensor_data[sensor_num].payload[0] = mode | RTP | len;
                    sensor_data[sensor_num].payload[1] = (id[0] >> 21)&0xff ;
                    sensor_data[sensor_num].payload[2] = (id[0] >> 13)&0xff;
                    sensor_data[sensor_num].payload[3] = (id[0] >> 5 )&0xff;
                    sensor_data[sensor_num].payload[4] = (id[0] << 3 )&0xf8;		    
                    
                    for(i=0;i<len;i++)
                    {
                        sensor_data[sensor_num].payload[i+5] = data1[i];
                    }
					
                    for(i=0;i<13;i++)
                    {
                        rt_kprintf(" %x ,",sensor_data[sensor_num].payload[i]);
                    }
              
                    rt_kprintf(" \n");  
                    sensor_num++;
                }            
            }
            
		
        }
        else if(ret == 0x02)
        {
            i=0;
            len=CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB1DLC)),ARG_UNUSED,ARG_UNUSED, port)&0x0f;//读取接收缓冲器0接收到的数据长度(0~8个字节)    
            mode = (CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB1SIDL)),ARG_UNUSED,ARG_UNUSED, port)&0x08)<<4;   //0:标准帧  1:扩展帧
            RTP = CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB1DLC)),ARG_UNUSED,ARG_UNUSED, port)&0x40;//0:扩展数据帧  1:扩展远程帧
            while(i<len)
            {	
                //把接收缓冲区里的数据，放到内部RAM缓冲区
                data2[i]=CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB1D0))+i,ARG_UNUSED,ARG_UNUSED, port);
                i++;
            }
            rt_kprintf(" PORT %d ：RXBUF1 :",port);
            for(i=0;i<len;i++)
            {
                rt_kprintf("  %d ,",data2[i]);
            }
            rt_kprintf(" \n");  
            
            if( len > 0)
            {
                id[1] = ID_Receive(RXBUF1,port); 
                temp_type = (id[1]>> 20) & 0x3f ;
                if((temp_type >= 1) && (temp_type <= 53))
                {
                    sensor_data[sensor_num].port = port;
                    sensor_data[sensor_num].type = (id[1] >> 20) & 0x3f;
                    sensor_data[sensor_num].subAddr = (id[1] >> 13) & 0x7f;
                    sensor_data[sensor_num].subLen = len + 5;
                    
                    sensor_data[sensor_num].payload[0] = mode | RTP | len;
                    sensor_data[sensor_num].payload[1] = (id[1] >> 21)&0xff ;
                    sensor_data[sensor_num].payload[2] = (id[1] >> 13)&0xff;
                    sensor_data[sensor_num].payload[3] = (id[1] >> 5 )&0xff;
                    sensor_data[sensor_num].payload[4] = (id[1] << 3 )&0xf8;
                    
                    for(i=0;i<len;i++)
                    {
                        sensor_data[sensor_num].payload[i+5] = data2[i];
                    }
                    for(i=0;i<13;i++)
                    {
                        rt_kprintf(" %x ,",sensor_data[sensor_num].payload[i]);
                    }

                    rt_kprintf(" \n");  
                    sensor_num++;
                }            
            }
        } 
        
        else if(ret == 0x03)
        {
            i=0;
            len=CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB0DLC)),ARG_UNUSED,ARG_UNUSED, port)&0x0f;//读取接收缓冲器0接收到的数据长度(0~8个字节)    
            mode = (CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB0SIDL)),ARG_UNUSED,ARG_UNUSED, port)&0x08)<<4; ;  //0:标准帧  1:扩展帧
            RTP = (CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB0DLC)),ARG_UNUSED,ARG_UNUSED, port)&0x40) ;//0:扩展数据帧  1:扩展远程帧
            while(i<len)
            {	
                //把接收缓冲区里的数据，放到内部RAM缓冲区
                data1[i]=CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB0D0))+i,ARG_UNUSED,ARG_UNUSED, port);
                i++;
            }
            rt_kprintf(" PORT %d ：RXBUF0 :",port);
            for(i=0;i<len;i++)
            {
                rt_kprintf("  %d ,",data1[i]);
            }
            rt_kprintf(" \n");  
            
            if( len > 0)
            {
                id[0] = ID_Receive(RXBUF0,port); 
                temp_type = (id[0]>> 20) & 0x3f ;
                if((temp_type >= 1) && (temp_type <= 53))
                {
                    sensor_data[sensor_num].port = port;
                    sensor_data[sensor_num].type = (id[0] >> 20) & 0x3f;
                    sensor_data[sensor_num].subAddr = (id[0] >> 13) & 0x7f;
                    sensor_data[sensor_num].subLen = len + 5;
                    
                    sensor_data[sensor_num].payload[0] = mode | RTP | len;
                    sensor_data[sensor_num].payload[1] = (id[0] >> 21)&0xff ;
                    sensor_data[sensor_num].payload[2] = (id[0] >> 13)&0xff;
                    sensor_data[sensor_num].payload[3] = (id[0] >> 5 )&0xff;
                    sensor_data[sensor_num].payload[4] = (id[0] << 3 )&0xf8;
                    
                    for(i=0;i<len;i++)
                    {
                        sensor_data[sensor_num].payload[i+5] = data1[i];
                    }
                    for(i=0;i<13;i++)
                    {
                        rt_kprintf(" %x ,",sensor_data[sensor_num].payload[i]);
                    }
			
                    rt_kprintf(" \n");  
                    sensor_num++;
                }            
            }
            
			
            //接收RXBUF1的数据
            i=0;
            len = CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB1DLC)),ARG_UNUSED,ARG_UNUSED, port)&0x0f;//读取接收缓冲器0接收到的数据长度(0~8个字节)    
            mode = (CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB1SIDL)),ARG_UNUSED,ARG_UNUSED, port)&0x08)<<4; ;  //0:标准帧  1:扩展帧
            RTP = (CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB1DLC)),ARG_UNUSED,ARG_UNUSED, port)&0x40);//0:扩展数据帧  1:扩展远程帧
            while(i<len)
            {	
                //把接收缓冲区里的数据，放到内部RAM缓冲区
                data2[i]=CAN_SPI_CMD( SPI_CMD_READ, TOLONG(&(MCP2515_MAP->RXB1D0))+i,ARG_UNUSED,ARG_UNUSED, port);
                i++;
            }
            rt_kprintf(" PORT %d :RXBUF1 :",port);
            for(i=0;i<len;i++)
            {
                rt_kprintf("  %d ,",data2[i]);
            }
            rt_kprintf(" \n");  
            
            if( len > 0)
            {
                
                id[1] = ID_Receive(RXBUF1,port); 
                temp_type = (id[1]>> 20) & 0x3f ;
                if((temp_type >= 1) && (temp_type <= 53))
                {
                    sensor_data[sensor_num].port = port;
                    sensor_data[sensor_num].type = (id[1] >> 20) & 0x3f;
                    sensor_data[sensor_num].subAddr = (id[1] >> 13) & 0x7f;
                    sensor_data[sensor_num].subLen = len + 5;
                    
                    sensor_data[sensor_num].payload[0] = mode | RTP | len;
                    sensor_data[sensor_num].payload[1] = (id[1] >> 21)&0xff ;
                    sensor_data[sensor_num].payload[2] = (id[1] >> 13)&0xff;
                    sensor_data[sensor_num].payload[3] = (id[1] >> 5 )&0xff;
                    sensor_data[sensor_num].payload[4] = (id[1] << 3 )&0xf8;
                    
                    for(i=0;i<len;i++)
                    {
                        sensor_data[sensor_num].payload[i+5] = data2[i];
                    }
                    for(i=0;i<13;i++)
                    {
                        rt_kprintf(" %x ,",sensor_data[sensor_num].payload[i]);
                    }

                    rt_kprintf(" \n");  
                    sensor_num++;
                }            
            }
        }
        CAN_SPI_CMD( SPI_CMD_WRITE, TOLONG(&(MCP2515_MAP->CANINTF)),0,ARG_UNUSED, port);
    }

}


void set_sensor_data(uint8_t *pdata_flash,int len)
{
	int i=0;
	unsigned char * pdata = pdata_flash;		
	save_option = *(CFG_OPTION_T *)pdata_flash; 
	sensor_all_len = *(short int *)(pdata +32);
	sensor_type_len = sensor_all_len;
	pdata = pdata + sizeof(CFG_OPTION_T) + sizeof(sensor_all_len);
	
	rt_kprintf(" Bsid = %d   Bsip = %d \n ",save_option.u32BsId,save_option.u32BsIp);
	rt_kprintf(" sensor_all_len = %d\n",sensor_all_len);
	
	if(sensor_all_len > 0)
	{				
		while( sensor_type_len > 0)
		{
			save_type = *pdata ;
			pdata++;
            		
//			rt_kprintf(" save_type = %d\n",save_type);
			if(save_type == SENSOR_ALARM_CONFIG)
			{				
				sensor_alarm_blackout_len = *(short *)pdata;
//				rt_kprintf(" sensor_alarm_blackout_len = %d\n",sensor_alarm_blackout_len);
				pdata += sizeof(sensor_alarm_blackout_len);
				
                		rt_memset( (struct MINITOR_ALARM_DATA *)minitor_data,0,18*(sizeof(struct MINITOR_ALARM_DATA)));
                        
				if(sensor_alarm_blackout_len > 0)
				{
					for(i=0;i<sensor_alarm_blackout_len/sizeof(struct MINITOR_ALARM_DATA);i++)
                           		{
						struct MINITOR_ALARM_DATA *psensor = (struct MINITOR_ALARM_DATA *)pdata;
						pdata += sizeof(struct MINITOR_ALARM_DATA);

			                        minitor_data[minitor_num].port = psensor->port;
			                        minitor_data[minitor_num].addr= psensor->addr;
			                        rt_kprintf(" port = %d addr = %d ",minitor_data[minitor_num].port,minitor_data[minitor_num].addr);
			                        
			                        minitor_data[minitor_num].blackout_up = psensor->blackout_up;
			                        minitor_data[minitor_num].blackout_down = psensor->blackout_down;					
			                        minitor_data[minitor_num].recover_up = psensor->recover_up;
			                        minitor_data[minitor_num].recover_down = psensor->recover_down;

			                        minitor_data[minitor_num].num_up = psensor->num_up;
			                        minitor_data[minitor_num].num_down = psensor->num_down;

						Delay_ms(1);						
						rt_memcpy(minitor_data[minitor_num].psensor_alarm_blackout_up,psensor->psensor_alarm_blackout_up,psensor->num_up * sizeof(struct SENSOR_INFORMATION));

						Delay_ms(1);	
						rt_memcpy(minitor_data[minitor_num].psensor_alarm_blackout_down,psensor->psensor_alarm_blackout_down,psensor->num_down * sizeof(struct SENSOR_INFORMATION));
						minitor_num++;
					}									
				}
				sensor_type_len -= sensor_alarm_blackout_len;
				rt_kprintf(" sensor_type_len = %d\n",sensor_type_len);
			}
			else if(save_type == SWITCH_STATUS_CONFIG)
			{
				sensor_switch_state_len = *(short *)pdata;
				pdata += sizeof(sensor_switch_state_len);
				
				rt_memset( (struct MINITOR_SWITCH_DATA *)minitor_switch_data,0,6*(sizeof(struct MINITOR_SWITCH_DATA)));
                        
				if(sensor_switch_state_len > 0)
				{
					for(i=0;i<sensor_switch_state_len/sizeof(struct MINITOR_SWITCH_DATA);i++)
					{
						struct MINITOR_SWITCH_DATA *psensor = (struct MINITOR_SWITCH_DATA *)pdata;
						pdata += sizeof(struct MINITOR_SWITCH_DATA);

			                        minitor_switch_data[minitor_switch_num].port = psensor->port;
			                        minitor_switch_data[minitor_switch_num].addr= psensor->addr;
			                        minitor_switch_data[minitor_switch_num].alarm_state= psensor->alarm_state;
			               
			                        minitor_switch_data[minitor_switch_num].num_onestate = psensor->num_onestate;
			                        minitor_switch_data[minitor_switch_num].num_secondstate = psensor->num_secondstate;
			                        
						Delay_ms(1);						
						rt_memcpy(minitor_switch_data[minitor_switch_num].psensor_switch_onestate_blackout,psensor->psensor_switch_onestate_blackout,psensor->num_onestate * sizeof(struct SENSOR_INFORMATION));
						Delay_ms(1);	
						rt_memcpy(minitor_switch_data[minitor_switch_num].psensor_switch_secondstate_blackout,psensor->psensor_switch_secondstate_blackout,psensor->num_secondstate * sizeof(struct SENSOR_INFORMATION));
						minitor_switch_num++;
					}									
				}			
				sensor_type_len -= sensor_switch_state_len;
			}
			else if(save_type == CH4_LOCK_D_CONFIG)
			{
				sensor_lock_d_len = *(short *)pdata;
				pdata += sizeof(sensor_lock_d_len);
				
				if( sensor_lock_d_len > 0)
				{
					struct MINITOR_LOCK_D * psensor = (struct MINITOR_LOCK_D *)pdata;
					pdata += sizeof(struct MINITOR_LOCK_D);
            
					minitor_lock_d.sign = psensor->sign;
					    
					minitor_lock_d.sensor_fan_port = psensor->sensor_fan_port;
					minitor_lock_d.sensor_fan_addr = psensor->sensor_fan_addr;
					minitor_lock_d.sensor_fan_value = psensor->sensor_fan_value;

					minitor_lock_d.sensor_airduct_port = psensor->sensor_airduct_port;
					minitor_lock_d.sensor_airduct_addr = psensor->sensor_airduct_addr;
					minitor_lock_d.sensor_airduct_value = psensor->sensor_airduct_value;

					rt_memset(minitor_lock_d.psensor_lock_d_blackout,0,6*sizeof(struct SENSOR_INFORMATION));
					rt_memcpy(minitor_lock_d.psensor_lock_d_blackout,psensor->psensor_lock_d_blackout,6*sizeof(struct SENSOR_INFORMATION));
				}			
				sensor_type_len -= sensor_lock_d_len;
			}
			else if(save_type == CH4_LOCK_E_CONFIG)
			{
				sensor_lock_e_len = *(short *)pdata;
				pdata += sizeof(sensor_lock_e_len);
				
				rt_memcpy(sensor_lock_e_buff,pdata,sensor_lock_e_len);
				
				if(sensor_lock_e_len > 0)
				{
					struct MINITOR_LOCK_E *psensor = (struct MINITOR_LOCK_E *)pdata; 
					pdata += sizeof(struct MINITOR_LOCK_E);
					
					minitor_lock_e.sign = psensor->sign;
                        
					minitor_lock_e.blackout_lock_len = psensor->blackout_lock_len;  

					minitor_lock_e.sensor_fan_port = psensor->sensor_fan_port;
					minitor_lock_e.sensor_fan_addr = psensor->sensor_fan_addr; 
					minitor_lock_e.sensor_fan_value = psensor->sensor_fan_value; 

					minitor_lock_e.sensor_drivingface_up_port = psensor->sensor_drivingface_up_port;
					minitor_lock_e.sensor_drivingface_up_addr = psensor->sensor_drivingface_up_addr ;
					minitor_lock_e.sensor_drivingface_up_value = psensor->sensor_drivingface_up_value; 

					minitor_lock_e.sensor_returncurrent_up_port = psensor->sensor_returncurrent_up_port ;
					minitor_lock_e.sensor_returncurrent_up_addr = psensor->sensor_returncurrent_up_addr ; 
					minitor_lock_e.sensor_returncurrent_up_value = psensor->sensor_returncurrent_up_value ;

					minitor_lock_e.reply_unlock_len = psensor->reply_unlock_len;

					minitor_lock_e.sensor_drivingface_down_port = psensor->sensor_drivingface_down_port ;
					minitor_lock_e.sensor_drivingface_down_addr = psensor->sensor_drivingface_down_addr ;
					minitor_lock_e.sensor_drivingface_down_value = psensor->sensor_drivingface_down_value ;

					minitor_lock_e.sensor_returncurrent_down_port = psensor->sensor_returncurrent_down_port ;
					minitor_lock_e.sensor_returncurrent_down_addr = psensor->sensor_returncurrent_down_addr;
					minitor_lock_e.sensor_returncurrent_down_value = psensor->sensor_returncurrent_down_value;

					rt_memset(minitor_lock_e.psensor_lock_e_blackout,0,6*sizeof(struct SENSOR_INFORMATION));
					rt_memcpy(minitor_lock_e.psensor_lock_e_blackout,psensor->psensor_lock_e_blackout,6*sizeof(struct SENSOR_INFORMATION));
					    									
				}		
				sensor_type_len -= sensor_lock_e_len;	
			}				
		}										
	}       
	
}




void save_data_to_flash(uint8_t state,uint8_t sign,uint16_t data_len,uint8_t *psensor_data)
{
	int offset = 0;
	uint8_t buff[2048];
	rt_memcpy(buff,&save_option,sizeof(CFG_OPTION_T));
	offset += sizeof(CFG_OPTION_T);
	switch(state)
	{
		case COMM_DOWN_ANALOG_UNTRALIMIT_CONFIG:
		{
			sensor_alarm_blackout_len = data_len;
            		break;
		}	
		
		case COMM_DOWN_SWITCH_STATUS_CONFIG:
		{
			sensor_switch_state_len = data_len;	
            		break;
		}	
		
		case COMM_DOWN_CH4_LOCK_CONFIG:
		{
			if(sign == 0)
				sensor_lock_d_len = data_len;
			else if(sign == 1)
				sensor_lock_e_len = data_len;	
            		break;
		}
		
		default:break;
	}	
	
	sensor_all_len = sensor_alarm_blackout_len + sensor_switch_state_len + sensor_lock_d_len + sensor_lock_e_len;
	rt_memcpy(buff+offset,&sensor_all_len,sizeof(sensor_all_len));
	offset += sizeof(sensor_all_len);
	
	if( sensor_alarm_blackout_len > 0)
	{
		save_type = SENSOR_ALARM_CONFIG;
		rt_memcpy(buff+offset,&save_type,sizeof(save_type));
		
		offset += sizeof(save_type);
		rt_memcpy(buff+offset,&sensor_alarm_blackout_len,sizeof(sensor_alarm_blackout_len));
		offset += sizeof(sensor_alarm_blackout_len);
		
		rt_memcpy(buff+offset,minitor_data,sensor_alarm_blackout_len);
		offset += sensor_alarm_blackout_len;
	}
	
	if( sensor_switch_state_len > 0)
	{
		save_type = SWITCH_STATUS_CONFIG;
		rt_memcpy(buff+offset,&save_type,sizeof(save_type));
		
		offset += sizeof(save_type);
		rt_memcpy(buff+offset,&sensor_switch_state_len,sizeof(sensor_switch_state_len));
		offset += sizeof(sensor_switch_state_len);			
	
		rt_memcpy(buff+offset,minitor_switch_data,sensor_switch_state_len);
		offset += sensor_switch_state_len;
	}	
	
	if( sensor_lock_d_len > 0)
	{
		save_type = CH4_LOCK_D_CONFIG;
		rt_memcpy(buff+offset,&save_type,sizeof(save_type));
		
		offset += sizeof(save_type);
		rt_memcpy(buff+offset,&sensor_lock_d_len,sizeof(sensor_lock_d_len));
		offset += sizeof(sensor_lock_d_len);		
						
		rt_memcpy(buff+offset,&minitor_lock_d,sensor_lock_d_len);
		offset += sensor_lock_d_len;
	}			
	
	if( sensor_lock_e_len > 0)
	{
		save_type = CH4_LOCK_E_CONFIG;
		rt_memcpy(buff+offset,&save_type,sizeof(save_type));
		
		offset += sizeof(save_type);
		rt_memcpy(buff+offset,&sensor_lock_e_len,sizeof(sensor_lock_e_len));
		offset += sizeof(sensor_lock_e_len);		
						
		rt_memcpy(buff+offset,&minitor_lock_e,sensor_lock_e_len);
		offset += sensor_lock_e_len;
	}	
	
	if(flash_save(OPTION_FLASH_ADDR,buff,offset))
		rt_kprintf("flash save OK--------------------\n");
	else 
		rt_kprintf("flash save ERROR--------------------\n");
}



char rev_state = 0;
unsigned short data_len =0;

void MCP2515_RUN()
{
	uint32_t temp=0;
	unsigned char data_blackout[8] = {0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned char data_reply[8] = {0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned char port = 1;
	unsigned char type1=0,type2=0;
	unsigned char command = 0;
	unsigned char sensor_addr = 0;


	uint8_t rev_buf[2048];
	uint8_t data_flash[2048];
		
	if(!flash_read_state)
	{
		if( flash_read(OPTION_FLASH_ADDR, (u8*)data_flash, sizeof(data_flash)))
		{
			set_sensor_data(data_flash,sizeof(data_flash));
			flash_read_state = 1;
		}
		else 
			flash_read_state = 0;
	}
       	    
	if(rt_mq_recv(&sensor_mq, rev_buf, sizeof(rev_buf), RT_WAITING_NO) == RT_EOK)
	{
		//todo:按port分发
		struct nwkhdr *pNwkHdr = (struct nwkhdr *)rev_buf;

		APP_HDR_T *pstAppHdr = (APP_HDR_T *)(pNwkHdr + 1);               

		data_len = pstAppHdr->len ;
		rt_kprintf(" datalen = %d\n",data_len);
		if(sizeof(struct nwkhdr) + pNwkHdr->len < MSG_COM_PKT_SIZE)
		{    
			switch(pstAppHdr->msgtype)   
			{
				case COMM_MANUAL_OUTAGE:      //手动断电
				{
					int j=0;
					int id = 0;
					char datalen = 0;
					uint8_t buff[128];
					struct SENSOR_RETURN_ACK psensor_ack;
					struct SENSOR_INTERRUPTER *psensor  = (struct SENSOR_INTERRUPTER *)(pstAppHdr + 1);
					rt_kprintf("COMM_MANUAL_OUTAGE come in\n");       
					rev_state = COMM_AUTO_OUTAGE_CONTROL;
					
					rt_kprintf("psensor->payload:");
					for(j=0;j<13;j++)
					{
						rt_kprintf(" %d ",psensor->payload[j]);
					}
					rt_kprintf("\n");
						
					datalen = psensor->payload[0] & 0x0f ;				
					id =(((psensor->payload[1]&0x0f)<<21) | (psensor->payload[2]<<13) );
		 
					rt_kprintf("COMM_MANUAL_OUTAGE: datalen = %d     id = %d\n",datalen,id);
					MCP2515_TX(TXBUF0,EXTID,id,datalen ,&psensor->payload[5],psensor->port);
					
					//ACK
					pNwkHdr->src = pNwkHdr->dst;
					pNwkHdr->dst = 0xffff;
					pNwkHdr->len = sizeof(APP_HDR_T)+sizeof(struct SENSOR_RETURN_ACK);
					pstAppHdr->len = sizeof(struct SENSOR_RETURN_ACK);;
					
					rt_memcpy(buff,pNwkHdr,sizeof(APP_HDR_T)+sizeof(struct nwkhdr));
					
					psensor_ack.port = psensor->port;
					psensor_ack.addr = psensor->addr;
					psensor_ack.reserved = 0;
					psensor_ack.state = 1;
					rt_memcpy(buff + sizeof(APP_HDR_T)+sizeof(struct nwkhdr),&psensor_ack,sizeof(struct SENSOR_RETURN_ACK));
					
					minitor_send_ack(buff,sizeof(struct nwkhdr)+ pNwkHdr->len);
					break;
				}
				case COMM_MANUAL_RESET:      //手动复电
				{
					int id = 0;
					char datalen = 0;
					uint8_t buff[128];
					struct SENSOR_RETURN_ACK psensor_ack;
					
					struct SENSOR_INTERRUPTER *psensor    = (struct SENSOR_INTERRUPTER *)(pstAppHdr + 1);
					rt_kprintf("COMM_MANUAL_RESET come in\n");       
					rev_state = COMM_AUTO_OUTAGE_CONTROL;
									
					datalen = psensor->payload[0] & 0x0f ;				
					id =(((psensor->payload[1]&0x0f)<<21) | (psensor->payload[2]<<13) );
		 
					rt_kprintf(" datalen = %d     id = %d\n",datalen,id);
					MCP2515_TX(TXBUF0,EXTID,id,datalen ,&psensor->payload[5],psensor->port); 
					
					//ACK
					pNwkHdr->src = pNwkHdr->dst;
					pNwkHdr->dst = 0xffff;
					pNwkHdr->len = sizeof(APP_HDR_T)+sizeof(struct SENSOR_RETURN_ACK);
					pstAppHdr->len = sizeof(struct SENSOR_RETURN_ACK);;
					
					rt_memcpy(buff,pNwkHdr,sizeof(APP_HDR_T)+sizeof(struct nwkhdr));
					
					psensor_ack.port = psensor->port;
					psensor_ack.addr = psensor->addr;
					psensor_ack.state = 1;
					rt_memcpy(buff + sizeof(APP_HDR_T)+sizeof(struct nwkhdr),&psensor_ack,sizeof(struct SENSOR_RETURN_ACK));
					
					minitor_send_ack(buff,sizeof(struct nwkhdr)+ pNwkHdr->len);
					
					break;
				}
				case COMM_AUTO_OUTAGE_CONTROL:      //自动断电
				{                        
					int id = 0;
					char datalen = 0;
					uint8_t buff[128];
					struct SENSOR_RETURN_ACK psensor_ack;
					
					struct SENSOR_INTERRUPTER *psensor  = (struct SENSOR_INTERRUPTER *)(pstAppHdr + 1);
					rt_kprintf("COMM_AUTO_OUTAGE_CONTROL come in\n");               
					rev_state = COMM_AUTO_OUTAGE_CONTROL;
									
					datalen = psensor->payload[0] & 0x0f ;				
					id =(((psensor->payload[1]&0x0f)<<21) | (psensor->payload[2]<<13) );
		 
					rt_kprintf(" datalen = %d     id = %d\n",datalen,id);
					MCP2515_TX(TXBUF0,EXTID,id,datalen ,&psensor->payload[5],psensor->port);      
				
					//ACK
					pNwkHdr->src = pNwkHdr->dst;
					pNwkHdr->dst = 0xffff;
					pNwkHdr->len = sizeof(APP_HDR_T)+sizeof(struct SENSOR_RETURN_ACK);
					pstAppHdr->len = sizeof(struct SENSOR_RETURN_ACK);;
					
					rt_memcpy(buff,pNwkHdr,sizeof(APP_HDR_T)+sizeof(struct nwkhdr));
					
					psensor_ack.port = psensor->port;
					psensor_ack.addr = psensor->addr;
					psensor_ack.state = 1;
					rt_memcpy(buff + sizeof(APP_HDR_T)+sizeof(struct nwkhdr),&psensor_ack,sizeof(struct SENSOR_RETURN_ACK));
					
					minitor_send_ack(buff,sizeof(struct nwkhdr)+ pNwkHdr->len);
					
					break;
				}
				case COMM_AUTO_RESET_CONTROL:       //自动复电
				{
					int id = 0;
					char datalen = 0;
					uint8_t buff[128];
					struct SENSOR_RETURN_ACK psensor_ack;
					
					struct SENSOR_INTERRUPTER *psensor = (struct SENSOR_INTERRUPTER *)(pstAppHdr + 1);
					rt_kprintf("COMM_AUTO_RESET_CONTROL come in\n");              
					rev_state = COMM_AUTO_OUTAGE_CONTROL;
									
					datalen = psensor->payload[0] & 0x0f ;				
					id =(((psensor->payload[1]&0x0f)<<21) | (psensor->payload[2]<<13) );
		 
					rt_kprintf(" datalen = %d     id = %d\n",datalen,id);
					MCP2515_TX(TXBUF0,EXTID,id,datalen ,&psensor->payload[5],psensor->port);                       
					
					//ACK
					pNwkHdr->src = pNwkHdr->dst;
					pNwkHdr->dst = 0xffff;
					pNwkHdr->len = sizeof(APP_HDR_T)+sizeof(struct SENSOR_RETURN_ACK);
					pstAppHdr->len = sizeof(struct SENSOR_RETURN_ACK);;
					
					rt_memcpy(buff,pNwkHdr,sizeof(APP_HDR_T)+sizeof(struct nwkhdr));
					
					psensor_ack.port = psensor->port;
					psensor_ack.addr = psensor->addr;
					psensor_ack.state = 1;
					rt_memcpy(buff + sizeof(APP_HDR_T)+sizeof(struct nwkhdr),&psensor_ack,sizeof(struct SENSOR_RETURN_ACK));
					
					minitor_send_ack(buff,sizeof(struct nwkhdr)+ pNwkHdr->len);
					
					break;
				}
				case COMM_DOWN_ANALOG_UNTRALIMIT_CONFIG:      //下发模拟量超限报警/断电配置
				{
                    			int i=0;
					int num_up;
					int num_down;
					uint8_t buff[128];
					struct SENSOR_RETURN_ACK psensor_ack;

					struct SENSOR_CONFIGURE *psensor_configure =(struct SENSOR_CONFIGURE *)(pstAppHdr + 1);
					struct SENSOR_INFORMATION *psensor_information = (struct SENSOR_INFORMATION *)((char *)(psensor_configure+1)+sizeof(num_up));
					if((psensor_configure->port != 0)&&(psensor_configure->addr != 0))
					{
					    if((minitor_num > 0)&&(minitor_num<18))
					    {

					        for(i=0;i<minitor_num;i++)
					        {
					            if((minitor_data[i].port == psensor_configure->port)&&(minitor_data[i].addr == psensor_configure->addr))
					            {
					                minitor_data[i].port = psensor_configure->port ;
					                minitor_data[i].addr = psensor_configure->addr ;     

							minitor_data[i].alarm_up =  psensor_configure->alarm_up;
					        	minitor_data[i].alarm_down =  psensor_configure->alarm_down;
					                                  
					                minitor_data[i].blackout_up = psensor_configure->blackout_up;
					                minitor_data[i].blackout_down = psensor_configure->blackout_down;
					        
					                minitor_data[i].recover_up = psensor_configure->recover_up;
					                minitor_data[i].recover_down = psensor_configure->recover_down;
					                rt_kprintf(" blackout_up = %x   blackout_down = %x  recover_up=%x  recover_down = %x\n",psensor_configure->blackout_up,psensor_configure->blackout_down, psensor_configure->recover_up,psensor_configure->recover_down);
					                num_up = *(int *)(psensor_configure+1);
					                num_down =*(int *)((char *)(psensor_configure+1)+sizeof(num_up)+ num_up*sizeof(struct SENSOR_INFORMATION));
					                minitor_data[i].num_up = num_up;
					                minitor_data[i].num_down = num_down;

					                rt_kprintf(" num_up = %d   num_down = %d\n",num_up,num_down );
					        
					                rt_memset((struct SENSOR_INFORMATION *)minitor_data[i].psensor_alarm_blackout_up,0,6*sizeof(struct SENSOR_INFORMATION)); 				
					                rt_memcpy(minitor_data[i].psensor_alarm_blackout_up,psensor_information,num_up*sizeof(struct SENSOR_INFORMATION));
					        
					                rt_memset((struct SENSOR_INFORMATION *)minitor_data[i].psensor_alarm_blackout_down,0,6*sizeof(struct SENSOR_INFORMATION));
					                psensor_information = (struct SENSOR_INFORMATION *)((char *)(psensor_configure+1)+sizeof(num_up)+sizeof(num_down)+num_up*sizeof(struct SENSOR_INFORMATION));	
					                rt_memcpy(minitor_data[i].psensor_alarm_blackout_down,psensor_information,num_down*sizeof(struct SENSOR_INFORMATION));
					                break;
					            }
					            else if(i == (minitor_num-1))
					            {
					                minitor_data[minitor_num].port = psensor_configure->port ;
					                minitor_data[minitor_num].addr = psensor_configure->addr ;
					                
					                minitor_data[minitor_num].alarm_up =  psensor_configure->alarm_up;
					        	minitor_data[minitor_num].alarm_down =  psensor_configure->alarm_down;
					                          
					                minitor_data[minitor_num].blackout_up = psensor_configure->blackout_up;
					                minitor_data[minitor_num].blackout_down = psensor_configure->blackout_down;
					        
					                minitor_data[minitor_num].recover_up = psensor_configure->recover_up;
					                minitor_data[minitor_num].recover_down = psensor_configure->recover_down;
					                rt_kprintf(" blackout_up = %x   blackout_down = %x  recover_up=%x  recover_down = %x\n",psensor_configure->blackout_up,psensor_configure->blackout_down, psensor_configure->recover_up,psensor_configure->recover_down);
					                num_up = *(int *)(psensor_configure+1);
					                num_down =*(int *)((char *)(psensor_configure+1)+sizeof(num_up)+ num_up*sizeof(struct SENSOR_INFORMATION));
					                minitor_data[minitor_num].num_up = num_up;
					                minitor_data[minitor_num].num_down = num_down;

					                rt_kprintf(" num_up = %d   num_down = %d\n",num_up,num_down );
					        
					                rt_memset((struct SENSOR_INFORMATION *)minitor_data[minitor_num].psensor_alarm_blackout_up,0,6*sizeof(struct SENSOR_INFORMATION));
					                psensor_information = (struct SENSOR_INFORMATION *)((char *)(psensor_configure+1)+sizeof(num_up));				
					                rt_memcpy(minitor_data[minitor_num].psensor_alarm_blackout_up,psensor_information,num_up*sizeof(struct SENSOR_INFORMATION));
					        
					                rt_memset((struct SENSOR_INFORMATION *)minitor_data[minitor_num].psensor_alarm_blackout_down,0,6*sizeof(struct SENSOR_INFORMATION));
					                psensor_information = (struct SENSOR_INFORMATION *)((char *)(psensor_configure+1)+sizeof(num_up)+sizeof(num_down)+num_up*sizeof(struct SENSOR_INFORMATION));	
					                rt_memcpy(minitor_data[minitor_num].psensor_alarm_blackout_down,psensor_information,num_down*sizeof(struct SENSOR_INFORMATION));
					                minitor_num++;
					                break;
					            }
					        }
					    }
					    else if(minitor_num == 0)
					    {
					        minitor_data[minitor_num].port = psensor_configure->port ;
					        minitor_data[minitor_num].addr= psensor_configure->addr ;

					        minitor_data[minitor_num].alarm_up =  psensor_configure->alarm_up;
					        minitor_data[minitor_num].alarm_down =  psensor_configure->alarm_down;
					                      
					        minitor_data[minitor_num].blackout_up = psensor_configure->blackout_up;
					        minitor_data[minitor_num].blackout_down = psensor_configure->blackout_down;

					        minitor_data[minitor_num].recover_up = psensor_configure->recover_up;
					        minitor_data[minitor_num].recover_down = psensor_configure->recover_down;
					        rt_kprintf(" blackout_up = %x   blackout_down = %x  recover_up=%x  recover_down = %x\n",psensor_configure->blackout_up,psensor_configure->blackout_down, psensor_configure->recover_up,psensor_configure->recover_down);
					        num_up = *(int *)(psensor_configure+1);
					        num_down =*(int *)((char *)(psensor_configure+1)+sizeof(num_up)+ num_up*sizeof(struct SENSOR_INFORMATION));
					        minitor_data[minitor_num].num_up = num_up;
					        minitor_data[minitor_num].num_down = num_down;

					        rt_kprintf(" num_up = %d   num_down = %d\n",num_up,num_down );

					        rt_memset((struct SENSOR_INFORMATION *)minitor_data[minitor_num].psensor_alarm_blackout_up,0,6*sizeof(struct SENSOR_INFORMATION));
					        psensor_information = (struct SENSOR_INFORMATION *)((char *)(psensor_configure+1)+sizeof(num_up));				
					        rt_memcpy(minitor_data[minitor_num].psensor_alarm_blackout_up,psensor_information,num_up*sizeof(struct SENSOR_INFORMATION));

					        rt_memset((struct SENSOR_INFORMATION *)minitor_data[minitor_num].psensor_alarm_blackout_down,0,6*sizeof(struct SENSOR_INFORMATION));
					        psensor_information = (struct SENSOR_INFORMATION *)((char *)(psensor_configure+1)+sizeof(num_up)+sizeof(num_down)+num_up*sizeof(struct SENSOR_INFORMATION));	
					        rt_memcpy(minitor_data[minitor_num].psensor_alarm_blackout_down,psensor_information,num_down*sizeof(struct SENSOR_INFORMATION));
					        minitor_num++;
					    }
					    
					    rev_state = COMM_DOWN_ANALOG_UNTRALIMIT_CONFIG;                     
					    //将接收服务器的值写入flash
	//				    save_data_to_flash(COMM_DOWN_ANALOG_UNTRALIMIT_CONFIG,0,minitor_num * sizeof(struct MINITOR_DATA),(uint8_t *)minitor_data);
					                        
					    //ACK
					    pNwkHdr->src = pNwkHdr->dst;
					    pNwkHdr->dst = 0xffff;
					    pNwkHdr->len = sizeof(APP_HDR_T)+sizeof(struct SENSOR_RETURN_ACK);
					    pstAppHdr->len = sizeof(struct SENSOR_RETURN_ACK);;
					    
					    rt_memcpy(buff,pNwkHdr,sizeof(APP_HDR_T)+sizeof(struct nwkhdr));
					    
					    psensor_ack.port = sensor_config_port;
					    psensor_ack.addr = sensor_config_addr;
					    psensor_ack.state = 1;
					    rt_memcpy(buff + sizeof(APP_HDR_T)+sizeof(struct nwkhdr),&psensor_ack,sizeof(struct SENSOR_RETURN_ACK));
					    
					    minitor_send_ack(buff,sizeof(struct nwkhdr)+ pNwkHdr->len);

					    break;
					}       
				}
				case COMM_DOWN_SWITCH_STATUS_CONFIG:      //下发开关量状态异常报警/断电配置	
				{				
					int num_onestate;
					int num_secondstate;
                    			int i=0;
					uint8_t buff[128];
					struct SENSOR_RETURN_ACK psensor_ack;			
					struct SENSOR_SWITCH_CONFIGURE *psensor_switch = (struct SENSOR_SWITCH_CONFIGURE *)(pstAppHdr + 1); 
					struct SENSOR_INFORMATION *psensor_information = (struct SENSOR_INFORMATION *)((char *)(psensor_switch + 1) + sizeof(num_onestate));
					           
					if((psensor_switch->port != 0)&&(psensor_switch->addr != 0))
					{
					    if((minitor_switch_num > 0)&&(minitor_switch_num<18))
					    {

					        for(i=0;i<minitor_switch_num;i++)
					        {
					            if((minitor_switch_data[i].port == psensor_switch->port)&&(minitor_switch_data[i].addr == psensor_switch->addr))
					            {
					                minitor_switch_data[i].port = psensor_switch->port ;
					                minitor_switch_data[i].addr = psensor_switch->addr ;
					                minitor_switch_data[i].alarm_state = psensor_switch->alarm_state;
					                alarm_state = psensor_switch->alarm_state;
					                
					                num_onestate = *(int *)(psensor_switch+1);
					                num_secondstate = *(int *)((char *)(psensor_switch+1)+sizeof(num_onestate)+ num_onestate*sizeof(struct SENSOR_INFORMATION));
					                minitor_switch_data[i].num_onestate = num_onestate;
					                minitor_switch_data[i].num_secondstate = num_secondstate;
					                
					                rt_memset((struct SENSOR_INFORMATION *)minitor_switch_data[i].psensor_switch_onestate_blackout,0,6*sizeof(struct SENSOR_INFORMATION));			
					                rt_memcpy(minitor_switch_data[i].psensor_switch_onestate_blackout,psensor_information,num_onestate*sizeof(struct SENSOR_INFORMATION));
					                
					                psensor_information = (struct SENSOR_INFORMATION *)((char *)(psensor_switch + 1) + sizeof(num_onestate) + sizeof(struct SENSOR_INFORMATION)*num_onestate + sizeof(num_secondstate)) ;
					                
					                rt_memset((struct SENSOR_INFORMATION *)minitor_switch_data[i].psensor_switch_secondstate_blackout,0,6*sizeof(struct SENSOR_INFORMATION));
					                rt_memcpy(minitor_switch_data[i].psensor_switch_secondstate_blackout,psensor_information,num_secondstate*sizeof(struct SENSOR_INFORMATION));
					                break;
					            }
					            else if(i == (minitor_switch_num-1))
					            {
					                minitor_switch_data[minitor_switch_num].port = psensor_switch->port ;
					                minitor_switch_data[minitor_switch_num].addr = psensor_switch->addr ;
					                minitor_switch_data[minitor_switch_num].alarm_state = psensor_switch->alarm_state;
					                alarm_state = psensor_switch->alarm_state;
					                
					                num_onestate = *(int *)(psensor_switch+1);
					                num_secondstate = *(int *)((char *)(psensor_switch+1)+sizeof(num_onestate)+ num_onestate*sizeof(struct SENSOR_INFORMATION));
					                minitor_switch_data[minitor_switch_num].num_onestate = num_onestate;
					                minitor_switch_data[minitor_switch_num].num_secondstate = num_secondstate;
					                
					                rt_memset((struct SENSOR_INFORMATION *)minitor_switch_data[minitor_switch_num].psensor_switch_onestate_blackout,0,6*sizeof(struct SENSOR_INFORMATION));			
					                rt_memcpy(minitor_switch_data[minitor_switch_num].psensor_switch_onestate_blackout,psensor_information,num_onestate*sizeof(struct SENSOR_INFORMATION));
					                
					                psensor_information = (struct SENSOR_INFORMATION *)((char *)(psensor_switch + 1) + sizeof(num_onestate) + sizeof(struct SENSOR_INFORMATION)*num_onestate + sizeof(num_secondstate)) ;
					                
					                rt_memset((struct SENSOR_INFORMATION *)minitor_switch_data[minitor_switch_num].psensor_switch_secondstate_blackout,0,6*sizeof(struct SENSOR_INFORMATION));
					                rt_memcpy(minitor_switch_data[minitor_switch_num].psensor_switch_secondstate_blackout,psensor_information,num_secondstate*sizeof(struct SENSOR_INFORMATION));

					                minitor_switch_num++;
					                break;
					            }
					        }
					    }
					    else if(minitor_switch_num == 0)
					    {
					        minitor_switch_data[minitor_switch_num].port = psensor_switch->port ;
					        minitor_switch_data[minitor_switch_num].addr = psensor_switch->addr ;
					        minitor_switch_data[minitor_switch_num].alarm_state = psensor_switch->alarm_state;
					        alarm_state = psensor_switch->alarm_state;
					        
					        num_onestate = *(int *)(psensor_switch+1);
					        num_secondstate = *(int *)((char *)(psensor_switch+1)+sizeof(num_onestate)+ num_onestate*sizeof(struct SENSOR_INFORMATION));
					        minitor_switch_data[minitor_switch_num].num_onestate = num_onestate;
					        minitor_switch_data[minitor_switch_num].num_secondstate = num_secondstate;
					        
					        rt_memset((struct SENSOR_INFORMATION *)minitor_switch_data[minitor_switch_num].psensor_switch_onestate_blackout,0,6*sizeof(struct SENSOR_INFORMATION));			
					        rt_memcpy(minitor_switch_data[minitor_switch_num].psensor_switch_onestate_blackout,psensor_information,num_onestate*sizeof(struct SENSOR_INFORMATION));
					        
					        psensor_information = (struct SENSOR_INFORMATION *)((char *)(psensor_switch + 1) + sizeof(num_onestate) + sizeof(struct SENSOR_INFORMATION)*num_onestate + sizeof(num_secondstate)) ;
					        
					        rt_memset((struct SENSOR_INFORMATION *)minitor_switch_data[minitor_switch_num].psensor_switch_secondstate_blackout,0,6*sizeof(struct SENSOR_INFORMATION));
					        rt_memcpy(minitor_switch_data[minitor_switch_num].psensor_switch_secondstate_blackout,psensor_information,num_secondstate*sizeof(struct SENSOR_INFORMATION));

					        minitor_switch_num++;
					        break;
					    }


					    //将接收服务器的值写入flash
					    save_data_to_flash(COMM_DOWN_SWITCH_STATUS_CONFIG,0,minitor_switch_num * sizeof(struct MINITOR_SWITCH_DATA),(uint8_t *)minitor_switch_data);
					    
					    //ACK
					    pNwkHdr->src = pNwkHdr->dst;
					    pNwkHdr->dst = 0xffff;
					    pNwkHdr->len = sizeof(APP_HDR_T)+sizeof(struct SENSOR_RETURN_ACK);
					    pstAppHdr->len = sizeof(struct SENSOR_RETURN_ACK);;
					    
					    rt_memcpy(buff,pNwkHdr,sizeof(APP_HDR_T)+sizeof(struct nwkhdr));
					    
					    psensor_ack.port = minitor_switch_data[minitor_switch_num].port;
					    psensor_ack.addr = minitor_switch_data[minitor_switch_num].addr;
					    psensor_ack.state = 1;
					    rt_memcpy(buff + sizeof(APP_HDR_T)+sizeof(struct nwkhdr),&psensor_ack,sizeof(struct SENSOR_RETURN_ACK));
					    
					    minitor_send_ack(buff,sizeof(struct nwkhdr)+ pNwkHdr->len);
					    
					    break;
					}
				}
					case COMM_DOWN_CH4_LOCK_CONFIG:      //下发甲烷风电闭锁配置
					{				
					uint8_t buff[128];
					if(*(char *)((pstAppHdr + 1)+3) == 0)          //风电闭锁d
					{
						struct MINITOR_LOCK_D *psensor_lock = (struct MINITOR_LOCK_D *)(pstAppHdr + 1); 
						struct SENSOR_LOCK_D_ACK psensor_ack;
                        
			                        minitor_lock_d.sign = psensor_lock->sign;
			                        
			                        minitor_lock_d.sensor_fan_port = psensor_lock->sensor_fan_port;
			                        minitor_lock_d.sensor_fan_addr = psensor_lock->sensor_fan_addr;
			                        minitor_lock_d.sensor_fan_value = psensor_lock->sensor_fan_value;
			                        
			                        minitor_lock_d.sensor_airduct_port = psensor_lock->sensor_airduct_port;
			                        minitor_lock_d.sensor_airduct_addr = psensor_lock->sensor_airduct_addr;
			                        minitor_lock_d.sensor_airduct_value = psensor_lock->sensor_airduct_value;
                        
						rt_memset(minitor_lock_d.psensor_lock_d_blackout,0,6*sizeof(struct SENSOR_INFORMATION));
                        			rt_memcpy(minitor_lock_d.psensor_lock_d_blackout,psensor_lock->psensor_lock_d_blackout,6*sizeof(struct SENSOR_INFORMATION));
			
						//将接收服务器的值写入flash
						save_data_to_flash(COMM_DOWN_CH4_LOCK_CONFIG,0,sizeof(minitor_lock_d),(uint8_t *)&minitor_lock_d);
						
						//ACK
						pNwkHdr->src = pNwkHdr->dst;						
						pNwkHdr->dst = 0xffff;
						pNwkHdr->len = sizeof(APP_HDR_T)+sizeof(struct SENSOR_RETURN_ACK);
						pstAppHdr->len = sizeof(struct SENSOR_LOCK_D_ACK);
						
						rt_memcpy(buff,pNwkHdr,sizeof(APP_HDR_T)+sizeof(struct nwkhdr));
						
						psensor_ack.fan_port = minitor_lock_d.sensor_fan_port ;
						psensor_ack.fan_addr = minitor_lock_d.sensor_fan_addr;
						psensor_ack.ram_air_port = minitor_lock_d.sensor_airduct_port;
						psensor_ack.ram_air_addr = minitor_lock_d.sensor_airduct_addr;
						psensor_ack.sign = 0 ;
						psensor_ack.state = 1;
						
						rt_memcpy(buff + sizeof(APP_HDR_T)+sizeof(struct nwkhdr),&psensor_ack,sizeof(struct SENSOR_LOCK_D_ACK));
						
						minitor_send_ack(buff,sizeof(struct nwkhdr)+ pNwkHdr->len);
								
					}
					else if(*(char *)(pstAppHdr + 1) == 1)              //风电闭锁e
					{
						uint8_t buff[128];
						struct SENSOR_LOCK_E_ACK psensor_ack;
						struct MINITOR_LOCK_E *psensor_lock = (struct MINITOR_LOCK_E *)(pstAppHdr + 1);
                  
			                        minitor_lock_e.sign = psensor_lock->sign;
			                        
			                        minitor_lock_e.blackout_lock_len = psensor_lock->blackout_lock_len;  
			                        
			                        minitor_lock_e.sensor_fan_port = psensor_lock->sensor_fan_port;
			                        minitor_lock_e.sensor_fan_addr = psensor_lock->sensor_fan_addr; 
			                        minitor_lock_e.sensor_fan_value = psensor_lock->sensor_fan_value; 
			                        
			                        minitor_lock_e.sensor_drivingface_up_port = psensor_lock->sensor_drivingface_up_port;
			                        minitor_lock_e.sensor_drivingface_up_addr = psensor_lock->sensor_drivingface_up_addr ;
			                        minitor_lock_e.sensor_drivingface_up_value = psensor_lock->sensor_drivingface_up_value; 
			                        
			                        minitor_lock_e.sensor_returncurrent_up_port = psensor_lock->sensor_returncurrent_up_port ;
			                        minitor_lock_e.sensor_returncurrent_up_addr = psensor_lock->sensor_returncurrent_up_addr ; 
			                        minitor_lock_e.sensor_returncurrent_up_value = psensor_lock->sensor_returncurrent_up_value ;
			                        
			                        minitor_lock_e.reply_unlock_len = psensor_lock->reply_unlock_len;
			                        
			                        minitor_lock_e.sensor_drivingface_down_port = psensor_lock->sensor_drivingface_down_port ;
			                        minitor_lock_e.sensor_drivingface_down_addr = psensor_lock->sensor_drivingface_down_addr ;
			                        minitor_lock_e.sensor_drivingface_down_value = psensor_lock->sensor_drivingface_down_value ;
			                        
			                        minitor_lock_e.sensor_returncurrent_down_port = psensor_lock->sensor_returncurrent_down_port ;
			                        minitor_lock_e.sensor_returncurrent_down_addr = psensor_lock->sensor_returncurrent_down_addr;
			                        minitor_lock_e.sensor_returncurrent_down_value = psensor_lock->sensor_returncurrent_down_value;
			                        
			                        rt_memset(minitor_lock_e.psensor_lock_e_blackout,0,6*sizeof(struct SENSOR_INFORMATION));
			                        rt_memcpy(minitor_lock_e.psensor_lock_e_blackout,psensor_lock->psensor_lock_e_blackout,6*sizeof(struct SENSOR_INFORMATION));
                        									
						//将接收服务器的值写入flash
						save_data_to_flash(COMM_DOWN_CH4_LOCK_CONFIG,1,sizeof(minitor_lock_e),(uint8_t *)&minitor_lock_e);
						
						//ACK
						pNwkHdr->src = pNwkHdr->dst;
						pNwkHdr->dst = 0xffff;
						pNwkHdr->len = sizeof(APP_HDR_T)+sizeof(struct SENSOR_LOCK_E_ACK);
						pstAppHdr->len = sizeof(struct SENSOR_RETURN_ACK);;
						
						rt_memcpy(buff,pNwkHdr,sizeof(APP_HDR_T)+sizeof(struct nwkhdr));
						
						psensor_ack.sign = 1;
						psensor_ack.state = 1;
						psensor_ack.fan_addr = minitor_lock_e.sensor_fan_addr;
						psensor_ack.fan_port = minitor_lock_e.sensor_fan_port;
						psensor_ack.heading_face_addr = minitor_lock_e.sensor_drivingface_up_addr ;
						psensor_ack.heading_face_port = minitor_lock_e.sensor_drivingface_up_port ;
						psensor_ack.return_current_port = minitor_lock_e.sensor_returncurrent_up_port;
						psensor_ack.return_current_addr = minitor_lock_e.sensor_returncurrent_up_addr;
												
						rt_memcpy(buff + sizeof(APP_HDR_T)+sizeof(struct nwkhdr),&psensor_ack,sizeof(struct SENSOR_LOCK_E_ACK));
						
						minitor_send_ack(buff,sizeof(struct nwkhdr)+ pNwkHdr->len);							
					}
					break;
				}
				case COMM_DOWN_TIME_SYNC:
		             	{
		                  	uint32_t timetick = *(uint32_t *)(pstAppHdr + 1); 
					rt_kprintf(" Timetick = %d ---------------\n",timetick);
					break;
		               	}
				default: 
					rt_kprintf("The data send to sensor_mq too long  %d \n",sizeof(struct nwkhdr) + pNwkHdr->len );	
					break; 
				
			}
            
		}          
    }
    
    else
    {
    for(port=1;port<=4;port++)
    {
        int i=0;
        rt_memset((uint32_t *)id,0,sizeof(id));
        rt_memset((uint8_t *)rxbuf0,0,sizeof(rxbuf0));
        rt_memset((uint8_t *)rxbuf1,0,sizeof(rxbuf1));
        
        MCP2515_RX(rxbuf0,rxbuf1 , port);
        
        type1 = (id[0]>>20) & 0x3f;
        type2 = (id[1]>>20) & 0x3f;
   
        if( type1>0 && type1< 54)
        {
            command = id[0] & 0x1f;
            sensor_addr = (id[0] >> 13) & 0x7f; 
            switch (command)
            {
                case POWER_DOWN:         //断电 
                {    
		    
                    break;
                }
                case INIT_REPORT:        //初始化上传
                { 
                    temp = id[0] & 0xffffefff;
                    MCP2515_TX(TXBUF0,EXTID,temp,8,(uint8_t *)data_reply,port);
                    break;
                }  
                case MSG_REPORT:         //上报数据
                { 
                          
                    if(rev_state == COMM_DOWN_ANALOG_UNTRALIMIT_CONFIG)
                    {
                        rt_kprintf("COMM_DOWN_ANALOG_UNTRALIMIT_CONFIG coming \n");
                        if((port == sensor_config_port)&&(sensor_addr == sensor_config_addr ))
                        {
                            unsigned char buf_send[8];
                            int ID = id[0];
                            int i=0;
                            ID = ID & 0x3ffefe0;   //清空优先级26-28、方向位12、命令码0-4
                            ID |= (0x03 << 26 | 0x03) ;       //命令码3、优先级3、方向位0
                            buf_send[0] = 0;
                            buf_send[1] = 0;
                            ConvertToByte(&buf_send[2],int_to_float(alarm_up));
                            ConvertToByte(&buf_send[5],int_to_float(alarm_down));
                                 
                            rt_kprintf("ID = %d   psensor_configure data :",ID);
                            for(i=0;i<8;i++)
                            {
                                rt_kprintf(" %d ",buf_send[i]);
                            }
                            rt_kprintf("\n");
                            
                            MCP2515_TX(TXBUF0,EXTID,ID,8,buf_send,port);
                            
                            rev_state = 0;
                        }
                                      
                    }                    
                                   
                    //传感器数据检测  断电 or 复电
                    for(i=0;i<minitor_num;i++)
                    {
                        if((minitor_data[i].port == port)&&(minitor_data[i].addr == sensor_addr))
                        {
                            	if( ConverToByeFloat(&rxbuf0[2]) > int_to_float(minitor_data[i].blackout_up) )    //超过上限断电
                      		{      
                        		int j=0;
                          		int ID = 0;    
                                	rt_kprintf("BUF1:blackout_up coming\n");
	                                for(j=0;j<6;j++)
	                                {             
	                                    if((minitor_data[i].psensor_alarm_blackout_up[j].addr!=0)&&(minitor_data[i].psensor_alarm_blackout_up[j].port!=0))
	                                    {
	                                        ID = 0;
	                                        ID |= ((minitor_data[i].psensor_alarm_blackout_up[j].addr << 13) | (0x0f <<20)) ;   
	                                        rt_kprintf(" ID = %d \n",ID);      
	                                        MCP2515_TX(TXBUF1,EXTID,ID,8,data_blackout,minitor_data[i].psensor_alarm_blackout_up[j].port);                           
	                                    }
	                                }
                            	}
                       		else if(ConverToByeFloat(&rxbuf0[2]) < int_to_float(minitor_data[i].blackout_down))   //低于下限断电
                        	{                     
	                                int j=0;
	                                int ID = 0;
	                                rt_kprintf("BUF1:blackout_down coming\n");
	                                for(j=0;j<6;j++)
	                                {
	                                    if((minitor_data[i].psensor_alarm_blackout_down[j].addr!=0)&&(minitor_data[i].psensor_alarm_blackout_down[j].port!=0))
	                                    {
	                                        ID = 0;
	                                        ID |= ((minitor_data[i].psensor_alarm_blackout_down[j].addr << 13) | (0x0f <<20)) ;   
	                                        rt_kprintf(" ID = %d \n",ID);      
	                                    
	                                        MCP2515_TX(TXBUF1,EXTID,ID,8,data_blackout,minitor_data[i].psensor_alarm_blackout_down[j].port);                           
	                                    }
	                                }                
                      		 }    
                            	else if(ConverToByeFloat(&rxbuf0[2]) < int_to_float(minitor_data[i].recover_up))   //低于复电上限 复电
                        	{
	                                int j=0;
	                                int ID = 0;
	                                rt_kprintf("BUF1:recover_up coming\n");
	                           		for(j=0;j<6;j++)
	                                {
	                                    if((minitor_data[i].psensor_alarm_blackout_up[j].addr!=0)&&(minitor_data[i].psensor_alarm_blackout_up[j].port!=0))
	                                    {
	                                        ID = 0;
	                                        ID |= ((minitor_data[i].psensor_alarm_blackout_up[j].addr << 13) | (0x0f <<20)) ;   
	                                        rt_kprintf(" ID = %d \n",ID);      
	                                        MCP2515_TX(TXBUF1,EXTID,ID,8,data_reply,minitor_data[i].psensor_alarm_blackout_up[j].port);                           
	                                    }
	                                }
                       		}
                        	else if(ConverToByeFloat(&rxbuf0[2]) > int_to_float(minitor_data[i].recover_down))   //高于复电下限 复电
                        	{
	                                int j=0;
	                                int ID = 0;
	                                rt_kprintf("BUF1:recover_down coming\n");
	                                for(j=0;j<6;j++)
	                                {
	                                    if((minitor_data[i].psensor_alarm_blackout_down[j].addr!=0)&&(minitor_data[i].psensor_alarm_blackout_down[j].port!=0))
	                                    {
	                                        ID = 0;
	                                        ID |= ((minitor_data[i].psensor_alarm_blackout_down[j].addr << 13) | (0x0f <<20)) ;   
	                                        rt_kprintf(" ID = %d \n",ID);      
	                                        MCP2515_TX(TXBUF1,EXTID,ID,8,data_reply,minitor_data[i].psensor_alarm_blackout_down[j].port);                           
	                                    }
	                            
	                                }       
                        	}
                        }
                    }
                  
                                      
                    for(i=0;i<minitor_switch_num;i++)
                    {
                        if((minitor_switch_data[i].port == port)&&(minitor_switch_data[i].addr == sensor_addr))
                        {
                            if( (char)ConverToByeFloat(&rxbuf0[2]) == minitor_switch_data[i].alarm_state )   
                            {      
                                int j=0;
                                int ID = 0;
                                if( minitor_switch_data[i].alarm_state == 0)        //一态断电
                                {
                                    rt_kprintf(" onestate blackout coming \n");                  
                                    while((minitor_switch_data[i].psensor_switch_onestate_blackout[j].port != 0)&&(minitor_switch_data[i].psensor_switch_onestate_blackout[j].addr != 0))
                                    {
                                        ID = 0; 
                                        ID |= ((minitor_switch_data[i].psensor_switch_onestate_blackout[j].addr << 13) | (0x0f <<20)) ;   
                                        rt_kprintf(" ID = %d \n",ID);      
                                        MCP2515_TX(TXBUF1,EXTID,ID,8,data_blackout,minitor_switch_data[i].psensor_switch_onestate_blackout[j].port);
                                        j++;
                                    }      
                                }
                                else if( minitor_switch_data[i].alarm_state == 1)  
                                {
                                    rt_kprintf(" secondstate blackout coming \n");                         
                                    while((minitor_switch_data[i].psensor_switch_secondstate_blackout[j].port != 0)&&(minitor_switch_data[i].psensor_switch_secondstate_blackout[j].addr != 0))
                                    {
                                        ID = 0; 
                                        ID |= ((minitor_switch_data[i].psensor_switch_secondstate_blackout[j].addr << 13) | (0x0f <<20)) ;   
                                        rt_kprintf(" ID = %d \n",ID);      
                                        MCP2515_TX(TXBUF1,EXTID,ID,8,data_blackout,minitor_switch_data[i].psensor_switch_secondstate_blackout[j].port);
                                        j++;
                                    }              
                                }
                                               
                            }   
                        }
                    }
                    
                    
                    //甲烷光电闭锁检测
					
					if(( minitor_lock_d.sensor_fan_addr == sensor_addr)&&(minitor_lock_d.sensor_fan_port == port))
					{
						if((char)ConverToByeFloat(&rxbuf0[2]) != minitor_lock_d.sensor_fan_value)
							lock_state_d1 = 0;
						else 
							lock_state_d1 = 1;
					}
					else if((minitor_lock_d.sensor_airduct_addr == sensor_addr)&&(minitor_lock_d.sensor_airduct_port == port))
					{
						if((char)ConverToByeFloat(&rxbuf0[2]) == minitor_lock_d.sensor_airduct_value)
							lock_state_d2 = 0;
						else 
							lock_state_d2 = 1;
					}	
					else if((minitor_lock_e.sensor_fan_addr == sensor_addr)&&(minitor_lock_e.sensor_fan_port == port))
					{
						if((char)ConverToByeFloat(&rxbuf0[2]) != minitor_lock_e.sensor_fan_value)            //局部通风机停止运转值                            
							lock_state_e1 = 0;                         
						else 
							lock_state_e1 = 1;
					}
					else if((minitor_lock_e.sensor_drivingface_up_addr == sensor_addr)&&(minitor_lock_e.sensor_drivingface_up_port == port))    
					{
						if((char)ConverToByeFloat(&rxbuf0[2]) < int_to_float(minitor_lock_e.sensor_drivingface_up_value))   //掘进工作面CH4浓度值上限
							lock_state_e2 = 0;
						else 
							lock_state_e2 = 1;
					}
					else if((minitor_lock_e.sensor_returncurrent_up_addr == sensor_addr)&&(minitor_lock_e.sensor_returncurrent_up_port == port))
					{
						if(ConverToByeFloat(&rxbuf0[2]) < int_to_float(minitor_lock_e.sensor_returncurrent_up_value))     //回风流CH4浓度值上限
							lock_state_e3 = 0;
						else 
							lock_state_e3 = 1;
					}
					else if((minitor_lock_e.sensor_drivingface_down_addr == sensor_addr)&&(minitor_lock_e.sensor_drivingface_down_port == port))
					{
						if((char)ConverToByeFloat(&rxbuf0[2]) > int_to_float(minitor_lock_e.sensor_drivingface_down_value))       //掘进工作面CH4浓度值下限
							lock_state_e4 = 0;
						else 
							lock_state_e4 = 1;
					}
					else if((minitor_lock_e.sensor_returncurrent_down_addr == sensor_addr)&&(minitor_lock_e.sensor_returncurrent_down_port == port))
					{
						if((char)ConverToByeFloat(&rxbuf0[2]) > int_to_float(minitor_lock_e.sensor_returncurrent_down_value))      //回风流CH4浓度值下限
							lock_state_e5 = 0;
						else 
							lock_state_e5 = 1;
					}
					
					//d条件下的闭锁和解锁
					if(lock_state_d1 || lock_state_d2)
					{
						//断电闭锁
						int j=0;
						int ID = 0;
						
						while((minitor_lock_d.psensor_lock_d_blackout[j].addr != 0) && (minitor_lock_d.psensor_lock_d_blackout[j].port != 0))
						{
							ID |= ((minitor_lock_d.psensor_lock_d_blackout[j].addr << 13) | (0x0f <<20)) ; 
							rt_kprintf(" ID = %d \n",ID);                                
							MCP2515_TX(TXBUF1,EXTID,ID,8,data_blackout,minitor_lock_d.psensor_lock_d_blackout[j].port);   
							j++;								
						}
					}
					if(!(lock_state_d1 || lock_state_d2))
					{
						//复电解锁
						int j=0;
						int ID = 0;

						while((minitor_lock_d.psensor_lock_d_blackout[j].addr != 0) && (minitor_lock_d.psensor_lock_d_blackout[j].port != 0))
						{
							ID |= ((minitor_lock_d.psensor_lock_d_blackout[j].addr << 13) | (0x0f <<20)) ; 
							rt_kprintf(" ID = %d \n",ID);                                
							MCP2515_TX(TXBUF1,EXTID,ID,8,data_reply,minitor_lock_d.psensor_lock_d_blackout[j].port);   
							j++;								
						}
					}
					
					
					//e条件下的闭锁和解锁
					
					if(lock_state_e4 && lock_state_e5)
					{
						//复电解锁
						int j=0;
						int ID = 0;

						while((minitor_lock_e.psensor_lock_e_blackout[j].addr != 0) && (minitor_lock_e.psensor_lock_e_blackout[j].port != 0))
						{
							ID |= ((minitor_lock_e.psensor_lock_e_blackout[j].addr << 13) | (0x0f <<20)) ; 
							rt_kprintf(" ID = %d \n",ID);                                
							MCP2515_TX(TXBUF1,EXTID,ID,8,data_blackout,minitor_lock_e.psensor_lock_e_blackout[j].port);   
							j++;								
						}
					}
					else if(lock_state_e1 && (lock_state_e2 || lock_state_e3))
					{
						//断电闭锁
						int j=0;
						int ID = 0;
						
						while((minitor_lock_e.psensor_lock_e_blackout[j].addr != 0) && (minitor_lock_e.psensor_lock_e_blackout[j].port != 0))
						{
							ID |= ((minitor_lock_e.psensor_lock_e_blackout[j].addr << 13) | (0x0f <<20)) ; 
							rt_kprintf(" ID = %d \n",ID);                                
							MCP2515_TX(TXBUF1,EXTID,ID,8,data_blackout,minitor_lock_e.psensor_lock_e_blackout[j].port);   
							j++;								
						}
					}
              
                    break;
                }
                case ISSUE_ALARM_SHRESHOLD:    //下发报警值
                { 
                   
                    break;
                } 
                case BROADCAST_TIME:     //广播同步时间
                { 
                
                    break;
                } 
                case GET_BLACKBOX_DATA:      //黑匣子取数据:
                { 
                
                    break;
                } 
                case SENSOR_UP:             //传感器启动中
                { 
                
                    break;
                } 
                case STATION_GET_MSG:       // 分站向传感器去数据
                { 
                
                    break;
                } 
                case INSSUE_AV_VALUE:       // 分站下发（电流，电压）变比值
                { 
                
                    break;
                } 
                case ISSUE_ALART_MSG:       //分站给语音报警器或led显示屏下发报警数据
                { 
                
                    break;
                }  
                case ISSUE_POWERDOWN_TEST:      //分站给电影箱下发断电实验指令
                { 
                
                    break;
                } 
                case ISSUE_ALARM_COMMAD:        //上位机下发分级报警命令
                { 
                
                    break;
                } 	
            }                
        }
        
        if( type2>0 && type2< 54)
        {
            command = id[1] & 0x1f;
            switch (command)
            {                                      
                case POWER_DOWN:         //断电 
                {
                  
                    break;
                }
                case INIT_REPORT:        //初始化上传
                { 
                    temp = id[0] & 0xffffefff;
                    MCP2515_TX(TXBUF0,EXTID,temp,8,(uint8_t *)data_reply,port);
                    break;
                }  
                case MSG_REPORT:         //上报数据
                { 
                          
                    if(rev_state == COMM_DOWN_ANALOG_UNTRALIMIT_CONFIG)
                    {
                        rt_kprintf("COMM_DOWN_ANALOG_UNTRALIMIT_CONFIG coming \n");
                        if((port == sensor_config_port)&&(sensor_addr == sensor_config_addr ))
                        {
                            unsigned char buf_send[8];
                            int ID = id[0];
                            int i=0;
                            ID = ID & 0x3ffefe0;   //清空优先级26-28、方向位12、命令码0-4
                            ID |= (0x03 << 26 | 0x03) ;       //命令码3、优先级3、方向位0
                            buf_send[0] = 0;
                            buf_send[1] = 0;
                            ConvertToByte(&buf_send[2],int_to_float(alarm_up));
                            ConvertToByte(&buf_send[5],int_to_float(alarm_down));
                                 
                            rt_kprintf("ID = %d   psensor_configure data :",ID);
                            for(i=0;i<8;i++)
                            {
                                rt_kprintf(" %d ",buf_send[i]);
                            }
                            rt_kprintf("\n");
                            
                            MCP2515_TX(TXBUF0,EXTID,ID,8,buf_send,port);
                            
                            rev_state = 0;
                        }
                                      
                    }                    
                                   
                    //传感器数据检测  断电 or 复电
                    for(i=0;i<minitor_num;i++)
                    {
                        if((minitor_data[i].port == port)&&(minitor_data[i].addr == sensor_addr))
                        {
                            if( ConverToByeFloat(&rxbuf0[2]) > int_to_float(minitor_data[i].blackout_up) )    //超过上限断电
                      		{     
                        		int j=0;
                          		int ID = 0; 
                                rt_kprintf("BUF2:blackout_up coming\n");
                                for(j=0;j<6;j++)
                                {             
                                    if((minitor_data[i].psensor_alarm_blackout_up[j].addr!=0)&&(minitor_data[i].psensor_alarm_blackout_up[j].port!=0))
                                    {
                                        ID = 0;
                                        ID |= ((minitor_data[i].psensor_alarm_blackout_up[j].addr << 13) | (0x0f <<20)) ;   
                                        rt_kprintf(" ID = %d \n",ID);      
                                        MCP2515_TX(TXBUF1,EXTID,ID,8,data_blackout,minitor_data[i].psensor_alarm_blackout_up[j].port);                           
                                    }
                                }
                            }
                       		else if(ConverToByeFloat(&rxbuf0[2]) < int_to_float(minitor_data[i].blackout_down))   //低于下限断电
                        	{                     
                                int j=0;
                                int ID = 0;
                                rt_kprintf("BUF2:blackout_down coming\n");
                                for(j=0;j<6;j++)
                                {
                                    if((minitor_data[i].psensor_alarm_blackout_down[j].addr!=0)&&(minitor_data[i].psensor_alarm_blackout_down[j].port!=0))
                                    {
                                        ID = 0;
                                        ID |= ((minitor_data[i].psensor_alarm_blackout_down[j].addr << 13) | (0x0f <<20)) ;   
                                        rt_kprintf(" ID = %d \n",ID);      
                                    
                                        MCP2515_TX(TXBUF1,EXTID,ID,8,data_blackout,minitor_data[i].psensor_alarm_blackout_down[j].port);                           
                                    }
                                }                
                      		  }    
                            else if(ConverToByeFloat(&rxbuf0[2]) < int_to_float(minitor_data[i].recover_up))   //低于复电上限 复电
                        	{
                                int j=0;
                                int ID = 0;
                                rt_kprintf("BUF2:recover_up coming\n");
                           		for(j=0;j<6;j++)
                                {
                                    if((minitor_data[i].psensor_alarm_blackout_up[j].addr!=0)&&(minitor_data[i].psensor_alarm_blackout_up[j].port!=0))
                                    {
                                        ID = 0;
                                        ID |= ((minitor_data[i].psensor_alarm_blackout_up[j].addr << 13) | (0x0f <<20)) ;   
                                        rt_kprintf(" ID = %d \n",ID);      
                                        MCP2515_TX(TXBUF1,EXTID,ID,8,data_reply,minitor_data[i].psensor_alarm_blackout_up[j].port);                           
                                    }
                                }
                       		}
                        	else if(ConverToByeFloat(&rxbuf0[2]) > int_to_float(minitor_data[i].recover_down))   //高于复电下限 复电
                        	{
                                int j=0;
                                int ID = 0;
                                rt_kprintf("BUF2:recover_down coming\n");
                                for(j=0;j<6;j++)
                                {
                                    if((minitor_data[i].psensor_alarm_blackout_down[j].addr!=0)&&(minitor_data[i].psensor_alarm_blackout_down[j].port!=0))
                                    {
                                        ID = 0;
                                        ID |= ((minitor_data[i].psensor_alarm_blackout_down[j].addr << 13) | (0x0f <<20)) ;   
                                        rt_kprintf(" ID = %d \n",ID);      
                                        MCP2515_TX(TXBUF1,EXTID,ID,8,data_reply,minitor_data[i].psensor_alarm_blackout_down[j].port);                           
                                    }
                            
                                }       
                        	}
                        }
                    }
                  
                                      
                    for(i=0;i<minitor_switch_num;i++)
                    {
                        if((minitor_switch_data[i].port == port)&&(minitor_switch_data[i].addr == sensor_addr))
                        {
                            if( (char)ConverToByeFloat(&rxbuf0[2]) == minitor_switch_data[i].alarm_state )   
                            {      
                                int j=0;
                                int ID = 0;
                                if( minitor_switch_data[i].alarm_state == 0)        //一态断电
                                {
                                    rt_kprintf(" onestate blackout coming \n");                  
                                    while((minitor_switch_data[i].psensor_switch_onestate_blackout[j].port != 0)&&(minitor_switch_data[i].psensor_switch_onestate_blackout[j].addr != 0))
                                    {
                                        ID = 0; 
                                        ID |= ((minitor_switch_data[i].psensor_switch_onestate_blackout[j].addr << 13) | (0x0f <<20)) ;   
                                        rt_kprintf(" ID = %d \n",ID);      
                                        MCP2515_TX(TXBUF1,EXTID,ID,8,data_blackout,minitor_switch_data[i].psensor_switch_onestate_blackout[j].port);
                                        j++;
                                    }      
                                }
                                else if( minitor_switch_data[i].alarm_state == 1)  
                                {
                                    rt_kprintf(" secondstate blackout coming \n");                         
                                    while((minitor_switch_data[i].psensor_switch_secondstate_blackout[j].port != 0)&&(minitor_switch_data[i].psensor_switch_secondstate_blackout[j].addr != 0))
                                    {
                                        ID = 0; 
                                        ID |= ((minitor_switch_data[i].psensor_switch_secondstate_blackout[j].addr << 13) | (0x0f <<20)) ;   
                                        rt_kprintf(" ID = %d \n",ID);      
                                        MCP2515_TX(TXBUF1,EXTID,ID,8,data_blackout,minitor_switch_data[i].psensor_switch_secondstate_blackout[j].port);
                                        j++;
                                    }              
                                }
                                               
                            }   
                        }
                    }
                    
					
					
					//甲烷光电闭锁检测
					
					if(( minitor_lock_d.sensor_fan_addr == sensor_addr)&&(minitor_lock_d.sensor_fan_port == port))
					{
						if((char)ConverToByeFloat(&rxbuf0[2]) != minitor_lock_d.sensor_fan_value)
							lock_state_d1 = 0;
						else 
							lock_state_d1 = 1;
					}
					else if((minitor_lock_d.sensor_airduct_addr == sensor_addr)&&(minitor_lock_d.sensor_airduct_port == port))
					{
						if((char)ConverToByeFloat(&rxbuf0[2]) == minitor_lock_d.sensor_airduct_value)
							lock_state_d2 = 0;
						else 
							lock_state_d2 = 1;
					}	
					else if((minitor_lock_e.sensor_fan_addr == sensor_addr)&&(minitor_lock_e.sensor_fan_port == port))
					{
						if((char)ConverToByeFloat(&rxbuf0[2]) != minitor_lock_e.sensor_fan_value)            //局部通风机停止运转值                            
							lock_state_e1 = 0;                         
						else 
							lock_state_e1 = 1;
					}
					else if((minitor_lock_e.sensor_drivingface_up_addr == sensor_addr)&&(minitor_lock_e.sensor_drivingface_up_port == port))    
					{
						if((char)ConverToByeFloat(&rxbuf0[2]) < int_to_float(minitor_lock_e.sensor_drivingface_up_value))   //掘进工作面CH4浓度值上限
							lock_state_e2 = 0;
						else 
							lock_state_e2 = 1;
					}
					else if((minitor_lock_e.sensor_returncurrent_up_addr == sensor_addr)&&(minitor_lock_e.sensor_returncurrent_up_port == port))
					{
						if(ConverToByeFloat(&rxbuf0[2]) < int_to_float(minitor_lock_e.sensor_returncurrent_up_value))     //回风流CH4浓度值上限
							lock_state_e3 = 0;
						else 
							lock_state_e3 = 1;
					}
					else if((minitor_lock_e.sensor_drivingface_down_addr == sensor_addr)&&(minitor_lock_e.sensor_drivingface_down_port == port))
					{
						if((char)ConverToByeFloat(&rxbuf0[2]) > int_to_float(minitor_lock_e.sensor_drivingface_down_value))       //掘进工作面CH4浓度值下限
							lock_state_e4 = 0;
						else 
							lock_state_e4 = 1;
					}
					else if((minitor_lock_e.sensor_returncurrent_down_addr == sensor_addr)&&(minitor_lock_e.sensor_returncurrent_down_port == port))
					{
						if((char)ConverToByeFloat(&rxbuf0[2]) > int_to_float(minitor_lock_e.sensor_returncurrent_down_value))      //回风流CH4浓度值下限
							lock_state_e5 = 0;
						else 
							lock_state_e5 = 1;
					}
					
					//d条件下的闭锁和解锁
					if(lock_state_d1 || lock_state_d2)
					{
						//断电闭锁
						int j=0;
						int ID = 0;
						
						while((minitor_lock_d.psensor_lock_d_blackout[j].addr != 0) && (minitor_lock_d.psensor_lock_d_blackout[j].port != 0))
						{
							ID |= ((minitor_lock_d.psensor_lock_d_blackout[j].addr << 13) | (0x0f <<20)) ; 
							rt_kprintf(" ID = %d \n",ID);                                
							MCP2515_TX(TXBUF1,EXTID,ID,8,data_blackout,minitor_lock_d.psensor_lock_d_blackout[j].port);   
							j++;								
						}
					}
					if(!(lock_state_d1 || lock_state_d2))
					{
						//复电解锁
						int j=0;
						int ID = 0;

						while((minitor_lock_d.psensor_lock_d_blackout[j].addr != 0) && (minitor_lock_d.psensor_lock_d_blackout[j].port != 0))
						{
							ID |= ((minitor_lock_d.psensor_lock_d_blackout[j].addr << 13) | (0x0f <<20)) ; 
							rt_kprintf(" ID = %d \n",ID);                                
							MCP2515_TX(TXBUF1,EXTID,ID,8,data_reply,minitor_lock_d.psensor_lock_d_blackout[j].port);   
							j++;								
						}
					}
					
					
					//e条件下的闭锁和解锁
					
					if(lock_state_e4 && lock_state_e5)
					{
						//复电解锁
						int j=0;
						int ID = 0;

						while((minitor_lock_e.psensor_lock_e_blackout[j].addr != 0) && (minitor_lock_e.psensor_lock_e_blackout[j].port != 0))
						{
							ID |= ((minitor_lock_e.psensor_lock_e_blackout[j].addr << 13) | (0x0f <<20)) ; 
							rt_kprintf(" ID = %d \n",ID);                                
							MCP2515_TX(TXBUF1,EXTID,ID,8,data_blackout,minitor_lock_e.psensor_lock_e_blackout[j].port);   
							j++;								
						}
					}
					else if(lock_state_e1 && (lock_state_e2 || lock_state_e3))
					{
						//断电闭锁
						int j=0;
						int ID = 0;
						
						while((minitor_lock_e.psensor_lock_e_blackout[j].addr != 0) && (minitor_lock_e.psensor_lock_e_blackout[j].port != 0))
						{
							ID |= ((minitor_lock_e.psensor_lock_e_blackout[j].addr << 13) | (0x0f <<20)) ; 
							rt_kprintf(" ID = %d \n",ID);                                
							MCP2515_TX(TXBUF1,EXTID,ID,8,data_blackout,minitor_lock_e.psensor_lock_e_blackout[j].port);   
							j++;								
						}
					}
                    
                    break;
                }
                case ISSUE_ALARM_SHRESHOLD:    //下发报警值
                { 
                   
                    break;
                } 
                case BROADCAST_TIME:     //广播同步时间
                { 
                
                    break;
                } 
                case GET_BLACKBOX_DATA:      //黑匣子取数据:
                { 
                
                    break;
                } 
                case SENSOR_UP:             //传感器启动中
                { 
                
                    break;
                } 
                case STATION_GET_MSG:       // 分站向传感器去数据
                { 
                
                    break;
                } 
                case INSSUE_AV_VALUE:       // 分站下发（电流，电压）变比值
                { 
                
                    break;
                } 
                case ISSUE_ALART_MSG:       //分站给语音报警器或led显示屏下发报警数据
                { 
                
                    break;
                }  
                case ISSUE_POWERDOWN_TEST:      //分站给电影箱下发断电实验指令
                { 
                
                    break;
                } 
                case ISSUE_ALARM_COMMAD:        //上位机下发分级报警命令
                { 
                
                    break;
                } 	
            }                
        }
        
    }
    	}
    sensordata_to_uartbuf();  
}


