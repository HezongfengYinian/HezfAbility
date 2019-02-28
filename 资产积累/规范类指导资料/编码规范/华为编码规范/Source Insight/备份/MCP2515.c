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
//* ͷ �� �� �� �� �� *
//*********************


#include "stm32f2xx.h"
//#include "IncludePath.h"
//#include "user.h"
//#include "hscom.h"


#include "MCP2515.h"


#define CanIdRecMask 0x265
//���ý�������ID
#define CanIdCorrectMask 0x7cf //���ý�������ID


#define SPI_MCP2515_CS_H()    GPIO_SetBits(GPIOG, GPIO_Pin_1);
#define SPI_MCP2515_CS_L()    GPIO_ResetBits(GPIOG, GPIO_Pin_1);


#define  SPIBUF         SPI1->DR



//**********************
//*   �� �� �� �� ��   *
//**********************
uint8_t SPI_MasterTransmit(uint8_t data);
void mcp2515_init(void);
void mcp2515_write_register(uint8_t data, uint8_t adress);
uint8_t mcp2515_read_register(uint8_t adress);
void mcp2515_bit_modify(uint8_t data, uint8_t mask, uint8_t adress);
void mcp2515_write_register_p(uint8_t adress, uint8_t *data, uint8_t length);
extern void Delay(uint8_t num);


//const uint8_t  MaskCode_s[4] = { 0xff, 0xf0, 0x00, 0x00 };
//��ȫ��ʶ������,16λ���������� ��׼֡
//const uint8_t  FilterCode_s[4] = { 0x7e,0B00000000, 0x00, 0x00 };
//Ҫ���յ�Ŀ���ʶ�� 3F0 ��׼֡�������ֽڲ��ã�


uint8_t  MaskCodeStandard[4] = { 0xff, 0xe0, 0x00, 0x00 };   //��׼֡
uint8_t  MaskCode[4] = { 0xff, 0xff, 0xff, 0xff };          //��ȫ��ʶ������ ��չ֡
uint8_t  FilterCode_0[4] = { (CanIdCorrectMask >> 3),(CanIdCorrectMask << 5), 0x00, 0x00 };
uint8_t  FilterCode_2[4] = { (CanIdRecMask >> 3),  (CanIdRecMask << 5), 0x00, 0x00 };
//��16bit���Զ�Ӧ��ǰ�������ֽڵ��˲���-��ʹ��ʱ���μĴ�������16bit����Ϊ0����


//8bit 5bit 8bit 8bit  PC_CAN���������ʾʱ��5bit���3it 2bit�����м����3bit�������8bit,����32bit(4���Ĵ���)
//const uint8_t  FilterCode_3[4] = { 0x7e,0B00001000, 0x59, 0x61 };
//Ҫ���յ�Ŀ���ʶ�� FC05961 ȥ����2Byte��bit2-4�����鼴�ô���
//const uint8_t  FilterCode_4[4] = { 0x7e,0B00001000, 0x59, 0x62 };
//Ҫ���յ�Ŀ���ʶ�� FC05962 //bit3=EXIDE  1 = �����˲���Ӧ������չ֡
//const uint8_t  FilterCode_5[4] = { 0x7e,0B00001000, 0x59, 0x63 };
//Ҫ���յ�Ŀ���ʶ�� FC05963
//0B00001000 bit3=1 ��չID   bit3=0 ��׼ID
//��Ҫ���ݣ�RXB0���ý��ձ�׼֡��RXB1���ý�����չ֡����ע���׼֡ʱ��16bit EID
//���Զ�Ӧ��ǰ�������ֽڵ��˲���-����ʹ��ʱ���μĴ�������16bit����Ϊ0���ɡ�




//**********************
//*   �� �� �� �� ��   *
//**********************
//**********************************************************//
//   ����˵����MCP2515��ʼ������                            //
//   ���룺    ��                                   //
//   �����    ��                                          //
//   ���ú�����                                  //
//**********************************************************//
void mcp2515_init(void)     //��8M�ڲ�ʱ��ʱ����ʼ��ʱ��Ϊ1.8MS     SPI = 687.5K
{
  //��ʼ��MCU��SPI����
  //SPI_MasterInit();
  SPI_MCP2515_CS_H();
  mDelayUs(20);
  
  
  // MCP2515 ����ǰ���������λ
  SPI_MCP2515_CS_L();     //MCP2515��CS��Ч
  SPI_MasterTransmit(SPI_RESET); //���͸�λ����
  SPI_MCP2515_CS_H(); //MCP2515��CS��Ч


  //ʹ��λ�޸�ָ�MCP2515����Ϊ����ģʽ
  //Ҳ���ǽ�CANCTRL�Ĵ�����REQOP[2:0]����Ϊ100
  //mcp2515_bit_modify( CANCTRL, 0xEC, (1<<REQOP2)|(1<<OSM) ); 
//NOT CLKEN   OSM=1ֻ���Է���һ��.
  mcp2515_bit_modify(CANCTRL, 0xEC, (1 << REQOP2));   //NOT CLKEN
  mDelayUs(20);
  while ((mcp2515_read_register(CANSTAT) & 0x80) != 0x80)
  {
    mDelayUs(20);      
    //asm("clrwdt"); //���ڲ����Ź�
  }


 //���ô����� Prop Seg Ϊ00����1TQ����λ����� Phase Seg1�ĳ���3TQ
  //mcp2515_write_register( CNF2, (1<<BTLMODE)|(1<<PHSEG11) );// (1<<SAM)|   1 = �ڲ���������߽������β���(��ʱ��λ�����1����5TQ)
  //���� ��λ����� Phase Seg2Ϊ 3TQ �� ���û����˲���
  //mcp2515_write_register( CNF3, (1<<PHSEG21)|(1<<WAKFIL) );
//3TQ=2+1 ��SOF��0=CLKOUT����ʹ��Ϊʱ���������,CANCTRL.CLKEN = 1ʱ�����ò���Ч��
  //�����˲���ʹ��
  //ʱ��Ƶ�ʣ�Fosc  = 16MHz
  //��Ƶ������ CNF1.BRP[5:0] = 7
  //��Сʱ��ݶ� TQ = 2 * ( BRP + 1 ) / Fosc   = 2*(7+1)/16M = 1uS
  //ͬ���� Sync Seg   = 1TQ
  //������ Prop Seg   = ( PRSEG + 1 ) * TQ  = 1 TQ
  //��λ����� Phase Seg1 = ( PHSEG1 + 1 ) * TQ = 3 TQ
  //��λ����� Phase Seg2 = ( PHSEG2 + 1 ) * TQ = 3 TQ
  //ͬ����ת��������Ϊ CNF1.SJW[1:0] = 00, �� 1TQ
  //���߲����� NBR = Fbit =  1/(sync seg + Prop seg + PS1 + PS2 )
  //                       = 1/(8TQ) = 1/8uS = 125kHz
  
  //mcp2515_write_register(CNF1, 0);//500K
  //mcp2515_write_register(CNF1, 1);//250K
  mcp2515_write_register(CNF1, 3);//125K


  //cp2515_write_register( CNF2, 0XA7 ); //������ 87.5%  16��8:5:2
  //cp2515_write_register( CNF3, 0X01 );


  //cp2515_write_register( CNF2, 0XBB ); //������ 81.25%  16��4:8:3
  //cp2515_write_register( CNF3, 0X02 );


  //cp2515_write_register( CNF2, 0XB4 ); //������ 81.25%  16��5:7:3
  //cp2515_write_register( CNF3, 0X02 );


  mcp2515_write_register(CNF2, 0XAD); //������ 81.25%  16��6:6:3
  mcp2515_write_register(CNF3, 0X02);


  //cp2515_write_register( CNF2, 0XA6 ); //������ 81.25%  16��7:3:3
  //cp2515_write_register( CNF3, 0X02 );
  //
  //cp2515_write_register( CNF2, 0X9F ); //������ 81.25%  16��8:4:3
  //cp2515_write_register( CNF3, 0X02 ); 




  // ����MCP2515�ж�ʹ�ܼĴ��������������ж�
  // mcp2515_write_register( CANINTE, /*(1<<RX1IE)|(1<<RX0IE)*/ 0 );


  // ����MCP2515�ж�ʹ�ܼĴ���,ʹ�ܽ��ջ������ж�
  //mcp2515_write_register( CANINTE, (1<<RX1IE)|(1<<RX0IE) );
  mcp2515_write_register(CANINTE, (1 << WAKIE));  //�����жϣ�ʹ�ܺ���ܴ�CANINTF�ж���WAKIF=1 ����λ����


  //�������ݽ�����ؼĴ���


  //����RXM[1:0]=11,�رս��ջ�����0����/�˲����ܣ��������б��ģ���ֹ���湦��
  //mcp2515_write_register( RXB0CTRL, (1<<RXM1)|(1<<RXM0) );
  mcp2515_write_register(RXB0CTRL, 0x00);    //������չ���׼ID�������������˲��Ĵ����е�RFXnSIDL.EXIDE����λ����


  //����RXM[1:0]=11,�رս��ջ�����1����/�˲����ܣ��������б��ģ�
  //mcp2515_write_register( RXB1CTRL, (1<<RXM1)|(1<<RXM0) );
  mcp2515_write_register(RXB1CTRL, 0X00);   //
/*
  //����2���������μĴ���ȫΪ1��
  //mcp2515_write_register_p( RXM0SIDH, MaskCode, 4 );
//��ȫ��ʶ������
  mcp2515_write_register_p(RXM0SIDH, MaskCodeStandard, 4);
  mcp2515_write_register_p(RXM1SIDH, MaskCodeStandard, 4);  //��׼֡��11b����
  //��Ҫ���ݣ�RXB0���ý��ձ�׼֡��RXB1���ý�����չ֡����ע���׼֡ʱ��16bit EID
  //���Զ�Ӧ��ǰ�������ֽڵ��˲���-����ʹ��ʱ���μĴ�������16bit����Ϊ0���ɡ�
  //����6�������˲��Ĵ�����


  mcp2515_write_register_p(RXF0SIDH, FilterCode_0, 4);  //��Ӧ�����������μĴ���0 page 25
  mcp2515_write_register_p(RXF1SIDH, MaskCode, 4);      //��ʹ��RXB0,ȫ����Ϊ1


  mcp2515_write_register_p(RXF2SIDH, FilterCode_2, 4);  //��Ӧ�����������μĴ���1
  mcp2515_write_register_p(RXF3SIDH, MaskCode, 4);
  mcp2515_write_register_p(RXF4SIDH, MaskCode, 4);
  mcp2515_write_register_p(RXF5SIDH, MaskCode, 4);*/


  mcp2515_write_register(TXB0CTRL, 0x03);    //��߷������ȼ�


  //��������
  //���ý���������ſ��ƼĴ������������ǽ��õڶ�����
  mcp2515_write_register(BFPCTRL, 0x00);


  //����ʹ�ã�����BFPCTRLʹRX0BF,RX1BF����Ϊ���������
  //mcp2515_bit_modify( BFPCTRL, (1<<B1BFE)|(1<<B0BFE)|(1<<B1BFM)|(1<<B0BFM), (1<<B1BFE)|(1<<B0BFE) );


  //���÷���������ſ��ƼĴ������������ǽ��õڶ�����
  mcp2515_write_register(TXRTSCTRL, 0);


  //MCP2515���뻷��ģʽ�����й��ܲ���
  //mcp2515_bit_modify( CANCTRL, 0XE0, (1<<REQOP1) );


  //MCP2515��������ģʽ
  mcp2515_bit_modify(CANCTRL, 0xE0, 0); //
  mDelayUs(20);
  while ((mcp2515_read_register(CANSTAT) & 0xe0) != 0X00)
  {
    mDelayUs(20);       
    //asm("clrwdt"); //���ڲ����Ź�
  }


  //��ʼ��Э��ID 9bit �����ʼ����8λ
  //mcp2515_write_register(TXB0SIDH,ProtocolID);  //��8λ
  //mcp2515_write_register(TXB0SIDL,ProtocolLastBit );
//���һλ


}

//**********************************************************//
//   ����˵����MCP2515д���ƼĴ�������                      //
//   ���룺    �Ĵ�����ַ��д������                         //
//   �����    ��                                          //
//   ���ú�����SPI���ͳ���SPI_MasterTransmit                //
//**********************************************************//
void mcp2515_write_register(uint8_t adress, uint8_t data)
{
	// CS low ,MCP2515 enable
	SPI_MCP2515_CS_L();


	SPI_MasterTransmit(SPI_WRITE); // ����SPIд�Ĵ���������


	SPI_MasterTransmit(adress);  //���ͼĴ�����ַ


	SPI_MasterTransmit(data);   //���ͼĴ�������


	//CS high ,MCP2515 disable
	SPI_MCP2515_CS_H();
}

//**********************************************************//
//   ����˵����MCP2515�����ƼĴ�������                      //
//   ���룺    �Ĵ�����ַ��                             //
//   �����    �Ĵ�������                                  //
//   ���ú�����SPI���ͳ���SPI_MasterTransmit                //
//**********************************************************//
uint8_t mcp2515_read_register(uint8_t adress)
{
	uint8_t data;


	// CS low ,MCP2515 enable
	SPI_MCP2515_CS_L();


	SPI_MasterTransmit(SPI_READ); // ����SPIд�Ĵ���������


	SPI_MasterTransmit(adress); //���ͼĴ�����ַ


	data = SPI_MasterTransmit(0xff); //�ض��Ĵ�������


	//CS high ,MCP2515 disable
	SPI_MCP2515_CS_H();


	return (data);
}

//**********************************************************//
//   ����˵������MCP2515���ջ���������                      //
//   ���룺    ��������ַ��                             //
//   �����    ����������                                  //
//   ���ú�����SPI���ͳ���SPI_MasterTransmit                //
//**********************************************************//
uint8_t mcp2515_read_rx_buffer(uint8_t adress)
{
	uint8_t data;


	// �ж�adress�Ƿ���Ч������1��2λ�����඼ӦΪ0
	if (adress & 0xF9) return (0);


	// CS low ,MCP2515 enable
	SPI_MCP2515_CS_L();


	SPI_MasterTransmit(SPI_READ_RX | adress); //���Ͷ�ȡ������


	data = SPI_MasterTransmit(0xff); //��������


	//CS high ,MCP2515 disable
	SPI_MCP2515_CS_H();


	return (data);
}

//**********************************************************//
//   ����˵����MCP2515���ƼĴ���λ�޸ĳ���                //
//   ���룺    �Ĵ�����ַ���޸�λ���޸�����                //
//   �����    ��                                      //
//   ���ú�����SPI���ͳ���SPI_MasterTransmit                //
//**********************************************************//
void mcp2515_bit_modify(uint8_t adress, uint8_t mask, uint8_t data)
{
	// CS low ,MCP2515 enable
	SPI_MCP2515_CS_L();


	SPI_MasterTransmit(SPI_BIT_MODIFY); //SPIλ�޸�ָ��


	SPI_MasterTransmit(adress);    //���ͼĴ�����ַ


	SPI_MasterTransmit(mask);     //���������ֽڣ�
								//�����ֽ��С�1����ʾ�������Ӧλ�޸ģ���0����ʾ��ֹ�޸�
	SPI_MasterTransmit(data);     //���������ֽ�


	//CS high ,MCP2515 disable
	SPI_MCP2515_CS_H();
}


//**********************************************************//
//   ����˵������MCP2515�����Ĵ�����������д����            //
//   ���룺    �����Ĵ�����ʼ��ַ������ָ�룬���ݳ���      //
//   �����    ��                                      //
//   ���ú�����SPI���ͳ���SPI_MasterTransmit                //
//**********************************************************//
void mcp2515_write_register_p(uint8_t adress, uint8_t *data, uint8_t length)
{
	uint8_t i;


	// CS low ,MCP2515 enable
	SPI_MCP2515_CS_L();


	SPI_MasterTransmit(SPI_WRITE);  //����SPIдָ��


	SPI_MasterTransmit(adress);    //������ʼ�Ĵ�����ַ


	for (i = 0; i < length; i++) SPI_MasterTransmit(*data++);  //��������


	//CS high ,MCP2515 disable
	SPI_MCP2515_CS_H();
}



//**********************************************************//
//   ����˵������MCP2515�����Ĵ�����������������            //
//   ���룺    �����Ĵ�����ʼ��ַ������ָ�룬���ݳ���      //
//   �����    ��                                      //
//   ���ú�����SPI���ͳ���SPI_MasterTransmit                //
//**********************************************************//
void mcp2515_read_register_p(uint8_t adress, uint8_t *data, uint8_t length)
{
	uint8_t i;


	// CS low ,MCP2515 enable
	SPI_MCP2515_CS_L();


	SPI_MasterTransmit(SPI_READ);  //����SPI��ָ��


	SPI_MasterTransmit(adress);   //������ʼ�Ĵ�����ַ


	for (i = 0; i < length; i++) *data++ = SPI_MasterTransmit(0xff);  //���ݱ���


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
	CanIdArr[1] &= 0x03;//����λ
	CanIdArr[1] |= (uint8_t)(pCanTxMsg->ExtId>>13) & 0xe0;//����λ
	CanIdArr[1] |= (uint8_t)1<<EXIDE;//��չ֡


	CanIdArr[2] = (uint8_t)(pCanTxMsg->ExtId>>8);
	CanIdArr[3] = (uint8_t)(pCanTxMsg->ExtId);


	mcp2515_bit_modify(TXB0CTRL,0x08,0);
	 //����TXREQ  
	mcp2515_write_register_p( TXB0SIDH, CanIdArr, 4 );

	mcp2515_write_register(TXB0DLC,pCanTxMsg->DLC);  
	mcp2515_write_register_p( TXB0D0, &pCanTxMsg->Data[0], 8); 

	mcp2515_bit_modify(TXB0CTRL,0x08,(1<<TXREQ));
 //��λTXREQ����ʼ���� 
}

/*
��ʱ��ȡ����
*/


void vMCP2515_CAN_Receive(CanRxMsg *pCanRxMsg)
{
	uint8_t u8TempRead;
	uint8_t CanIdArr[4];



	u8TempRead = mcp2515_read_register(CANINTF);  //��ȡ�Ƿ���յ�����


	if ((u8TempRead & 0X02) == 0X02)   // bit1 = RX1IF
	{
		//ÿ�ν��յ����ݾ����� ͨ�Ŵ��������
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


		pCanRxMsg->DLC = mcp2515_read_register(RXB1DLC);      //���ݳ���


		mcp2515_read_register_p(RXB1D0, &pCanRxMsg->Data[0], pCanRxMsg->DLC);   //��ȡ���յ�������


		mcp2515_write_register(CANINTF, 0);                      //���������жϱ�־
	}
	else if( (u8TempRead & 0X01) == 0X01 )
	// bit0 = RX0IF 
	{
		//ÿ�ν��յ����ݾ����� ͨ�Ŵ��������
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


		pCanRxMsg->DLC = mcp2515_read_register(RXB0DLC);      //���ݳ���


		mcp2515_read_register_p(RXB0D0, &pCanRxMsg->Data[0], pCanRxMsg->DLC); 


		mcp2515_write_register(CANINTF, 0);                      //���������жϱ�־ 
	} 
}

