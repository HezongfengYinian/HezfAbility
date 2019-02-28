/*============================================================
        C file about MCP2515   V1.00
==============================================================
Chip:      MCP2515
Function:    The controller of CAN-BUS
Writer:     
Data:      
Reference:   "mcp2515.c" 
=============================================================*/
//*********************
//* 头 文 件 配 置 区 *
//*********************


#include "stm32f2xx.h"
//#include "IncludePath.h"
//#include "user.h"
//#include "hscom.h"


#include "MCP2515.h"


#define CanIdRecMask 0x265
//设置接收屏蔽ID
#define CanIdCorrectMask 0x7cf //设置接收屏蔽ID


#define SPI_MCP2515_CS_H()    GPIO_SetBits(GPIOG, GPIO_Pin_1);
#define SPI_MCP2515_CS_L()    GPIO_ResetBits(GPIOG, GPIO_Pin_1);


#define  SPIBUF         SPI1->DR



//**********************
//*   函 数 声 明 区   *
//**********************
uint8_t SPI_MasterTransmit(uint8_t data);
void mcp2515_init(void);
void mcp2515_write_register(uint8_t data, uint8_t adress);
uint8_t mcp2515_read_register(uint8_t adress);
void mcp2515_bit_modify(uint8_t data, uint8_t mask, uint8_t adress);
void mcp2515_write_register_p(uint8_t adress, uint8_t *data, uint8_t length);
extern void Delay(uint8_t num);


//const uint8_t  MaskCode_s[4] = { 0xff, 0xf0, 0x00, 0x00 };
//完全标识符屏蔽,16位数据则不屏蔽 标准帧
//const uint8_t  FilterCode_s[4] = { 0x7e,0B00000000, 0x00, 0x00 };
//要接收的目标标识符 3F0 标准帧（后两字节不用）


uint8_t  MaskCodeStandard[4] = { 0xff, 0xe0, 0x00, 0x00 };   //标准帧
uint8_t  MaskCode[4] = { 0xff, 0xff, 0xff, 0xff };          //完全标识符屏蔽 扩展帧
uint8_t  FilterCode_0[4] = { (CanIdCorrectMask >> 3),(CanIdCorrectMask << 5), 0x00, 0x00 };
uint8_t  FilterCode_2[4] = { (CanIdRecMask >> 3),  (CanIdRecMask << 5), 0x00, 0x00 };
//后16bit会自动应用前两数据字节的滤波）-不使用时屏蔽寄存器中那16bit配置为0即可


//8bit 5bit 8bit 8bit  PC_CAN调试软件显示时将5bit拆成3it 2bit再在中间插入3bit，重组成8bit,即共32bit(4个寄存器)
//const uint8_t  FilterCode_3[4] = { 0x7e,0B00001000, 0x59, 0x61 };
//要接收的目标标识符 FC05961 去除第2Byte中bit2-4后重组即得此数
//const uint8_t  FilterCode_4[4] = { 0x7e,0B00001000, 0x59, 0x62 };
//要接收的目标标识符 FC05962 //bit3=EXIDE  1 = 报文滤波仅应用于扩展帧
//const uint8_t  FilterCode_5[4] = { 0x7e,0B00001000, 0x59, 0x63 };
//要接收的目标标识符 FC05963
//0B00001000 bit3=1 扩展ID   bit3=0 标准ID
//如要兼容：RXB0配置接收标准帧，RXB1配置接收扩展帧。（注意标准帧时后16bit EID
//会自动应用前两数据字节的滤波）-无须使用时屏蔽寄存器中那16bit配置为0即可。




//**********************
//*   函 数 定 义 区   *
//**********************
//**********************************************************//
//   函数说明：MCP2515初始化程序                            //
//   输入：    无                                   //
//   输出：    无                                          //
//   调用函数：                                  //
//**********************************************************//
void mcp2515_init(void)     //在8M内部时钟时，初始化时间为1.8MS     SPI = 687.5K
{
  //初始化MCU的SPI总线
  //SPI_MasterInit();
  SPI_MCP2515_CS_H();
  mDelayUs(20);
  
  
  // MCP2515 启动前进行软件复位
  SPI_MCP2515_CS_L();     //MCP2515的CS有效
  SPI_MasterTransmit(SPI_RESET); //发送复位命令
  SPI_MCP2515_CS_H(); //MCP2515的CS无效


  //使用位修改指令将MCP2515设置为配置模式
  //也就是将CANCTRL寄存器的REQOP[2:0]设置为100
  //mcp2515_bit_modify( CANCTRL, 0xEC, (1<<REQOP2)|(1<<OSM) ); 
//NOT CLKEN   OSM=1只尝试发送一次.
  mcp2515_bit_modify(CANCTRL, 0xEC, (1 << REQOP2));   //NOT CLKEN
  mDelayUs(20);
  while ((mcp2515_read_register(CANSTAT) & 0x80) != 0x80)
  {
    mDelayUs(20);      
    //asm("clrwdt"); //清内部看门狗
  }


 //设置传播段 Prop Seg 为00，即1TQ，相位缓冲段 Phase Seg1的长度3TQ
  //mcp2515_write_register( CNF2, (1<<BTLMODE)|(1<<PHSEG11) );// (1<<SAM)|   1 = 在采样点对总线进行三次采样(这时相位缓冲段1至少5TQ)
  //设置 相位缓冲段 Phase Seg2为 3TQ ， 禁用唤醒滤波器
  //mcp2515_write_register( CNF3, (1<<PHSEG21)|(1<<WAKFIL) );
//3TQ=2+1 （SOF）0=CLKOUT引脚使能为时间输出功能,CANCTRL.CLKEN = 1时该设置才有效。
  //唤醒滤波器使能
  //时钟频率：Fosc  = 16MHz
  //分频控制器 CNF1.BRP[5:0] = 7
  //最小时间份额 TQ = 2 * ( BRP + 1 ) / Fosc   = 2*(7+1)/16M = 1uS
  //同步段 Sync Seg   = 1TQ
  //传播段 Prop Seg   = ( PRSEG + 1 ) * TQ  = 1 TQ
  //相位缓冲段 Phase Seg1 = ( PHSEG1 + 1 ) * TQ = 3 TQ
  //相位缓冲段 Phase Seg2 = ( PHSEG2 + 1 ) * TQ = 3 TQ
  //同步跳转长度设置为 CNF1.SJW[1:0] = 00, 即 1TQ
  //总线波特率 NBR = Fbit =  1/(sync seg + Prop seg + PS1 + PS2 )
  //                       = 1/(8TQ) = 1/8uS = 125kHz
  
  //mcp2515_write_register(CNF1, 0);//500K
  //mcp2515_write_register(CNF1, 1);//250K
  mcp2515_write_register(CNF1, 3);//125K


  //cp2515_write_register( CNF2, 0XA7 ); //采样点 87.5%  16：8:5:2
  //cp2515_write_register( CNF3, 0X01 );


  //cp2515_write_register( CNF2, 0XBB ); //采样点 81.25%  16：4:8:3
  //cp2515_write_register( CNF3, 0X02 );


  //cp2515_write_register( CNF2, 0XB4 ); //采样点 81.25%  16：5:7:3
  //cp2515_write_register( CNF3, 0X02 );


  mcp2515_write_register(CNF2, 0XAD); //采样点 81.25%  16：6:6:3
  mcp2515_write_register(CNF3, 0X02);


  //cp2515_write_register( CNF2, 0XA6 ); //采样点 81.25%  16：7:3:3
  //cp2515_write_register( CNF3, 0X02 );
  //
  //cp2515_write_register( CNF2, 0X9F ); //采样点 81.25%  16：8:4:3
  //cp2515_write_register( CNF3, 0X02 ); 




  // 设置MCP2515中断使能寄存器，禁用所有中断
  // mcp2515_write_register( CANINTE, /*(1<<RX1IE)|(1<<RX0IE)*/ 0 );


  // 设置MCP2515中断使能寄存器,使能接收缓冲器中断
  //mcp2515_write_register( CANINTE, (1<<RX1IE)|(1<<RX0IE) );
  mcp2515_write_register(CANINTE, (1 << WAKIE));  //唤醒中断，使能后才能从CANINTF中读到WAKIF=1 其它位类似


  //设置数据接收相关寄存器


  //设置RXM[1:0]=11,关闭接收缓冲器0屏蔽/滤波功能，接收所有报文；禁止滚存功能
  //mcp2515_write_register( RXB0CTRL, (1<<RXM1)|(1<<RXM0) );
  mcp2515_write_register(RXB0CTRL, 0x00);    //接收扩展或标准ID报文则由验收滤波寄存器中的RFXnSIDL.EXIDE控制位决定


  //设置RXM[1:0]=11,关闭接收缓冲器1屏蔽/滤波功能，接收所有报文；
  //mcp2515_write_register( RXB1CTRL, (1<<RXM1)|(1<<RXM0) );
  mcp2515_write_register(RXB1CTRL, 0X00);   //
/*
  //设置2个验收屏蔽寄存器全为1，
  //mcp2515_write_register_p( RXM0SIDH, MaskCode, 4 );
//完全标识符屏蔽
  mcp2515_write_register_p(RXM0SIDH, MaskCodeStandard, 4);
  mcp2515_write_register_p(RXM1SIDH, MaskCodeStandard, 4);  //标准帧，11b屏蔽
  //如要兼容：RXB0配置接收标准帧，RXB1配置接收扩展帧。（注意标准帧时后16bit EID
  //会自动应用前两数据字节的滤波）-无须使用时屏蔽寄存器中那16bit配置为0即可。
  //设置6个验收滤波寄存器，


  mcp2515_write_register_p(RXF0SIDH, FilterCode_0, 4);  //对应作用域是屏蔽寄存器0 page 25
  mcp2515_write_register_p(RXF1SIDH, MaskCode, 4);      //不使用RXB0,全设置为1


  mcp2515_write_register_p(RXF2SIDH, FilterCode_2, 4);  //对应作用域是屏蔽寄存器1
  mcp2515_write_register_p(RXF3SIDH, MaskCode, 4);
  mcp2515_write_register_p(RXF4SIDH, MaskCode, 4);
  mcp2515_write_register_p(RXF5SIDH, MaskCode, 4);*/


  mcp2515_write_register(TXB0CTRL, 0x03);    //最高发送优先级


  //配置引脚
  //设置接收相关引脚控制寄存器，配置它们禁用第二功能
  mcp2515_write_register(BFPCTRL, 0x00);


  //调试使用，设置BFPCTRL使RX0BF,RX1BF设置为数字输出。
  //mcp2515_bit_modify( BFPCTRL, (1<<B1BFE)|(1<<B0BFE)|(1<<B1BFM)|(1<<B0BFM), (1<<B1BFE)|(1<<B0BFE) );


  //设置发送相关引脚控制寄存器，配置它们禁用第二功能
  mcp2515_write_register(TXRTSCTRL, 0);


  //MCP2515进入环回模式，进行功能测试
  //mcp2515_bit_modify( CANCTRL, 0XE0, (1<<REQOP1) );


  //MCP2515进入正常模式
  mcp2515_bit_modify(CANCTRL, 0xE0, 0); //
  mDelayUs(20);
  while ((mcp2515_read_register(CANSTAT) & 0xe0) != 0X00)
  {
    mDelayUs(20);       
    //asm("clrwdt"); //清内部看门狗
  }


  //初始化协议ID 9bit 必须初始化高8位
  //mcp2515_write_register(TXB0SIDH,ProtocolID);  //高8位
  //mcp2515_write_register(TXB0SIDL,ProtocolLastBit );
//最后一位


}

//**********************************************************//
//   函数说明：MCP2515写控制寄存器程序                      //
//   输入：    寄存器地址，写入数据                         //
//   输出：    无                                          //
//   调用函数：SPI发送程序SPI_MasterTransmit                //
//**********************************************************//
void mcp2515_write_register(uint8_t adress, uint8_t data)
{
	// CS low ,MCP2515 enable
	SPI_MCP2515_CS_L();


	SPI_MasterTransmit(SPI_WRITE); // 发送SPI写寄存器控制字


	SPI_MasterTransmit(adress);  //发送寄存器地址


	SPI_MasterTransmit(data);   //发送寄存器数据


	//CS high ,MCP2515 disable
	SPI_MCP2515_CS_H();
}

//**********************************************************//
//   函数说明：MCP2515读控制寄存器程序                      //
//   输入：    寄存器地址，                             //
//   输出：    寄存器数据                                  //
//   调用函数：SPI发送程序SPI_MasterTransmit                //
//**********************************************************//
uint8_t mcp2515_read_register(uint8_t adress)
{
	uint8_t data;


	// CS low ,MCP2515 enable
	SPI_MCP2515_CS_L();


	SPI_MasterTransmit(SPI_READ); // 发送SPI写寄存器控制字


	SPI_MasterTransmit(adress); //发送寄存器地址


	data = SPI_MasterTransmit(0xff); //回读寄存器数据


	//CS high ,MCP2515 disable
	SPI_MCP2515_CS_H();


	return (data);
}

//**********************************************************//
//   函数说明：读MCP2515接收缓冲器程序                      //
//   输入：    缓冲器地址，                             //
//   输出：    缓冲器数据                                  //
//   调用函数：SPI发送程序SPI_MasterTransmit                //
//**********************************************************//
uint8_t mcp2515_read_rx_buffer(uint8_t adress)
{
	uint8_t data;


	// 判断adress是否有效，除了1，2位，其余都应为0
	if (adress & 0xF9) return (0);


	// CS low ,MCP2515 enable
	SPI_MCP2515_CS_L();


	SPI_MasterTransmit(SPI_READ_RX | adress); //发送读取控制字


	data = SPI_MasterTransmit(0xff); //读回数据


	//CS high ,MCP2515 disable
	SPI_MCP2515_CS_H();


	return (data);
}

//**********************************************************//
//   函数说明：MCP2515控制寄存器位修改程序                //
//   输入：    寄存器地址，修改位，修改数据                //
//   输出：    无                                      //
//   调用函数：SPI发送程序SPI_MasterTransmit                //
//**********************************************************//
void mcp2515_bit_modify(uint8_t adress, uint8_t mask, uint8_t data)
{
	// CS low ,MCP2515 enable
	SPI_MCP2515_CS_L();


	SPI_MasterTransmit(SPI_BIT_MODIFY); //SPI位修改指令


	SPI_MasterTransmit(adress);    //发送寄存器地址


	SPI_MasterTransmit(mask);     //发送屏蔽字节，
								//屏蔽字节中“1”表示允许对相应位修改，“0”表示禁止修改
	SPI_MasterTransmit(data);     //发送数据字节


	//CS high ,MCP2515 disable
	SPI_MCP2515_CS_H();
}


//**********************************************************//
//   函数说明：对MCP2515连续寄存器进行连续写操作            //
//   输入：    连续寄存器起始地址，数据指针，数据长度      //
//   输出：    无                                      //
//   调用函数：SPI发送程序SPI_MasterTransmit                //
//**********************************************************//
void mcp2515_write_register_p(uint8_t adress, uint8_t *data, uint8_t length)
{
	uint8_t i;


	// CS low ,MCP2515 enable
	SPI_MCP2515_CS_L();


	SPI_MasterTransmit(SPI_WRITE);  //发送SPI写指令


	SPI_MasterTransmit(adress);    //发送起始寄存器地址


	for (i = 0; i < length; i++) SPI_MasterTransmit(*data++);  //发送数据


	//CS high ,MCP2515 disable
	SPI_MCP2515_CS_H();
}



//**********************************************************//
//   函数说明：对MCP2515连续寄存器进行连续读操作            //
//   输入：    连续寄存器起始地址，数据指针，数据长度      //
//   输出：    无                                      //
//   调用函数：SPI发送程序SPI_MasterTransmit                //
//**********************************************************//
void mcp2515_read_register_p(uint8_t adress, uint8_t *data, uint8_t length)
{
	uint8_t i;


	// CS low ,MCP2515 enable
	SPI_MCP2515_CS_L();


	SPI_MasterTransmit(SPI_READ);  //发送SPI读指令


	SPI_MasterTransmit(adress);   //发送起始寄存器地址


	for (i = 0; i < length; i++) *data++ = SPI_MasterTransmit(0xff);  //数据保存


	//CS high ,MCP2515 disable
	SPI_MCP2515_CS_H();
}


uint8_t SPI_MasterTransmit(uint8_t data)
{
	uint8_t tempdata;


	SPIBUF = data;


	tempdata = 0;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
	{
		mDelayUs(1);


		tempdata++;
		if (tempdata >= 10)
		{
			break;
		}
	}


	tempdata = SPIBUF;
	return (tempdata);
}

void vMCP2515_CAN_Transmit(CanTxMsg *pCanTxMsg)
{
	uint8_t CanIdArr[4];


	CanIdArr[0] = (uint8_t)(pCanTxMsg->ExtId>>21);
	CanIdArr[1] = (uint8_t)(pCanTxMsg->ExtId>>16);
	CanIdArr[1] &= 0x03;//低两位
	CanIdArr[1] |= (uint8_t)(pCanTxMsg->ExtId>>13) & 0xe0;//高三位
	CanIdArr[1] |= (uint8_t)1<<EXIDE;//扩展帧


	CanIdArr[2] = (uint8_t)(pCanTxMsg->ExtId>>8);
	CanIdArr[3] = (uint8_t)(pCanTxMsg->ExtId);


	mcp2515_bit_modify(TXB0CTRL,0x08,0);
	 //清零TXREQ  
	mcp2515_write_register_p( TXB0SIDH, CanIdArr, 4 );

	mcp2515_write_register(TXB0DLC,pCanTxMsg->DLC);  
	mcp2515_write_register_p( TXB0D0, &pCanTxMsg->Data[0], 8); 

	mcp2515_bit_modify(TXB0CTRL,0x08,(1<<TXREQ));
 //置位TXREQ，开始发送 
}

/*
定时读取数据
*/


void vMCP2515_CAN_Receive(CanRxMsg *pCanRxMsg)
{
	uint8_t u8TempRead;
	uint8_t CanIdArr[4];



	u8TempRead = mcp2515_read_register(CANINTF);  //读取是否接收到报文


	if ((u8TempRead & 0X02) == 0X02)   // bit1 = RX1IF
	{
		//每次接收到数据就清零 通信错误计数器
		mcp2515_read_register_p(RXB1SIDH, CanIdArr, 4);
		pCanRxMsg->ExtId  = ((uint32_t)CanIdArr[0])<<21;
		pCanRxMsg->ExtId |= ((uint32_t)(((CanIdArr[0]&0x07)<<5) | ((CanIdArr[1]&0xe0)>>3) | (CanIdArr[1]&0x03)))<<16;
		pCanRxMsg->ExtId |= ((uint32_t)CanIdArr[2])<<8;
		pCanRxMsg->ExtId |= (uint32_t)CanIdArr[3];


		// pCanRxMsg->ExtId |= u32TempExtid;


		//   if ((CanIdArr[1]&0x08) == 0)
		//   {
		//   pCanRxMsg->IDE = CAN_Id_Standard;
		//   }
		//   else
		//   {
		//   pCanRxMsg->IDE = CAN_Id_Extended;
		//   }


		pCanRxMsg->DLC = mcp2515_read_register(RXB1DLC);      //数据长度


		mcp2515_read_register_p(RXB1D0, &pCanRxMsg->Data[0], pCanRxMsg->DLC);   //读取接收到的数据


		mcp2515_write_register(CANINTF, 0);                      //清零所有中断标志
	}
	else if( (u8TempRead & 0X01) == 0X01 )
	// bit0 = RX0IF 
	{
		//每次接收到数据就清零 通信错误计数器
		mcp2515_read_register_p(RXB0SIDH, CanIdArr, 4);
		pCanRxMsg->ExtId = CanIdArr[0];
		pCanRxMsg->ExtId <<= 3;
		pCanRxMsg->ExtId |= CanIdArr[1]>>5;
		pCanRxMsg->ExtId |= (uint32_t)CanIdArr[2]<<19;
		pCanRxMsg->ExtId |= (uint32_t)CanIdArr[3]<<11;

		// if ((CanIdArr[1]&0x08) == 0)
		// {
		// pCanRxMsg->IDE = CAN_Id_Standard;
		// }
		// else
		// {
		// pCanRxMsg->IDE = CAN_Id_Extended;
		// }


		pCanRxMsg->DLC = mcp2515_read_register(RXB0DLC);      //数据长度


		mcp2515_read_register_p(RXB0D0, &pCanRxMsg->Data[0], pCanRxMsg->DLC); 


		mcp2515_write_register(CANINTF, 0);                      //清零所有中断标志 
	} 
}

