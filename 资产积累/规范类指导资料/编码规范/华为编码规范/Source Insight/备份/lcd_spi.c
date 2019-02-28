//----------------------------------------------------------------------
// Module    : 7.0 INCH TFT LCD  800*480   7 INCH  Resistive Touch Screen
// Lanuage   : C51 Code
// Drive IC  : RA8875    Font chip  23L32S4W    Flash:128M bit
// INTERFACE : 4-Wire SPI
// MCU 		 : STC12LE5C60S2  //51 series  1T MCU
// MCU VDD		 : 3.3V   
//----------------------------------------------------------------------
/*
1. CS------NC
2. RS------NC
3. WR------NC
4. RD------NC
5.RST---------------P3.4
6~21. DB0~DB15-----NC
22.WAIT
23.SCS---------------P3.3
24.SCK---------------P3.3
25.MISO--------------P3.3
26.MOSI--------------P3.3
27.INT
28.BLA-------------+5.0V
29.VDD-------------+3.3V
30.GND-------------0V
*/

//#include <absacc.h>
//#include <REG52.H>
#include <stdio.h>
#include "board.h"
#include "appliation.h"
#include "mbus_app.h"
#include "msg_center.h"
#include "bsmac_parser.h"
#include "ptl_nwk.h"
#include "pic.h"
#include "math.h"
#include "lcd_spi.h"
#include "stm32_flash.h"
#include "sensor_magic.h"
#include "net_app.h"
#include "time.h"
#include "string.h"
#include "stm32f2xx_flash.h"


uint8_t Is_coordinated_condition(uint8_t port,uint8_t addr,uint8_t symbol,uint32_t data);


/*
sbit  	MCU_RST=P3^4;
sbit  	SCLK=P1^1;
sbit  	SDI=P1^0;
sbit  	SDO=P1^2;
sbit  	SCS=P1^3;
*/

struct SENSOR_DATA_SAVE
{
	uint32_t timestamp;
	uint8_t state;
	uint8_t type; 
	uint8_t addr;
	uint8_t reserve;
	float data_last;
	float data_now;
	uint8_t updata_state;
	uint8_t sign;
	uint16_t reserved;    
};


struct SENSOR_COORDINATED
{
	uint8_t addr;
	volatile  float data;
};



SENSOR_DATA sensor_data[18];

struct SENSOR_DATA_SAVE sensor_data_save[6][3];
struct SENSOR_COORDINATED coordinated_data[6][3];

#define write_data_addr  0x0c  //slave addresses with write data
#define read_data_addr  0x0d  //slave addresses with write data
#define write_cmd_addr  0x0e  //slave addresses with write command
#define read_cmd_addr  0x0f  //slave addresses with read status

CFG_OPTION_T save_option ;


unsigned int X1,Y1,X2,Y2,X3,Y3,X4,Y4;
uint8_t taby[4];
uint8_t tabx[4];
uint16_t x[6],y[6],xmin,ymin,xmax,ymax;

#define start_xrow     4
#define color_brown   0x40c0
#define color_black   0x0000
#define color_white   0xffff
#define color_red     0xf800
#define color_green   0x07e0
#define color_blue    0x001f
#define color_yellow  color_red|color_green
#define color_cyan    color_green|color_blue
#define color_purple  color_red|color_blue
#define color_1       0xf0f0


rt_uint8_t evacuate_port  = 0;
rt_uint8_t broadcast_port = 0;
rt_uint8_t minitor_port = 0;
rt_uint8_t evacuate_state  = 0;
rt_uint8_t broadcast_state = 0;
rt_uint8_t minitor_state = 0;
rt_uint16_t evacuate_id  = 0;
rt_uint16_t broadcast_id = 0;
rt_uint16_t minitor_id = 0;

typedef struct {
	int type;
	int company;
	char namelist[30];
	char unit_logic[10];
}name_logic;
name_logic my_namelist[10];
//int color_list[6]={color_blue,color_yellow,color_green,color_brown,color_1,color_cyan};
uint16_t color_list[6]={color_green,color_green,color_green,color_green,color_green,color_green};

uint8_t list1[8][15];
uint8_t *list2_word[8]={" 电口1"," 电口2"," 电口3"," 电口4"," 电口5"," 电口6"," 电口7"," 电口8"};	
uint8_t *list3_word[12]={"百兆光     1","百兆光     2","百兆光     3","百兆光     4",
												 "千兆光     1","千兆光     2","千兆光     3","千兆光     4",
												"COM1","COM2","COM3","COM4"};																		
uint8_t list4[6][3][25];	
                                              

void Delay(void)
{	
		uint8_t j;
	
  	for(j=0;j<25;j++);
}


                                               
void Delay1ms(uint16_t i)
{uint8_t j;
	while(i--)
  	for(j=0;j<125;j++);
}


void Delay10ms(uint16_t i)
{	while(i--)
	Delay1ms(10);
}

void Delay100ms(uint16_t i)
{	while(i--)
	Delay1ms(100);
}

void delay_us(uint32_t us)
{
    volatile rt_uint32_t s ;

    for (s = 0; s < us; s++)
    {
        // for (value = 0; value < 32; value++);
        __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
        __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
        __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
        __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    }
}

void delay_ms(uint32_t ms)
{
 	unsigned int i=0;
	for(i=0;i<ms;i++)
	{
		delay_us(1000);
	}
}
void NextStep(void)
{ 
	int j=0;
 for(j=0;j<10000;j++)
 	__NOP();
}


void gpio_simulate_waveform(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	int i=0;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
      
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

        /*!< SPI SCK pin configuration */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
        GPIO_Init(GPIOE, &GPIO_InitStructure);

	for(i=0;i<5000;i++)
	{
		if(i % 2)
			GPIO_SetBits(GPIOE, GPIO_Pin_10);	
		else
			GPIO_ResetBits(GPIOE, GPIO_Pin_10);	
		
		Delay();
		
	}
	GPIO_SetBits(GPIOE, GPIO_Pin_10);
	Delay100ms(100);
	GPIO_ResetBits(GPIOE, GPIO_Pin_10);	
	
}




typedef union                                        
{
   float Temp;
   unsigned char  Buf[4];
}DtformConver;

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


float int_to_float(uint32_t data)
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




//*********4W_SPI_Init()
void select_cs_lcd(int type)
{
	if(type == ENABLE)
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	}
	else if(type ==DISABLE)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_4);
	}
}

void reset_bit(int type)
{
	if(type == ENABLE)
	{
		GPIO_SetBits(GPIOE,GPIO_Pin_7);
	}
	else if(type ==DISABLE)
	{
		GPIO_ResetBits(GPIOE,GPIO_Pin_7);
	}
}


uint8_t SPI_WR(uint8_t Byte)
{
    uint8_t Data;
	while (SPI_GetFlagStatus(SPI3, SPI_FLAG_TXE) == RESET);
	SPI_SendData(SPI3,Byte);
	while (SPI_GetFlagStatus(SPI3, SPI_FLAG_RXNE) == RESET);
    Data = SPI_ReceiveData(SPI3);
	return Data; //????SPIx???????	
}



//////////////SPI Write command
void LCD_CmdWrite(uint8_t cmd)
{	
	//SCLK = 1;	
	//SDI = 1;	
	select_cs_lcd(ENABLE);//SCS = 0;
	//SPI_Delay();
	SPI_WR(0x80); 
	SPI_WR(cmd);
	select_cs_lcd(DISABLE);//SCS = 1;
	//SPI_Delay();
}

//////////////SPI Write data or  parameter
void LCD_DataWrite(uint8_t Data)
{
	//SCLK = 1;	
	//SDI = 1;		
	select_cs_lcd(ENABLE);//SCS = 0;
	SPI_WR(0x00); 
	SPI_WR(Data);
	//SPI_Delay();
	select_cs_lcd(DISABLE);//SCS = 1;
}


void LCD_DataWrite16(uint16_t data)
{
	select_cs_lcd(ENABLE);//SCS = 0;
	SPI_WR(0x00);
	SPI_WR(data >> 8);
	select_cs_lcd(DISABLE);//SCS = 1;

	select_cs_lcd(ENABLE);//SCS = 0;
	SPI_WR(0x00);
	SPI_WR(data);
	select_cs_lcd(DISABLE);//SCS = 1;
    
}

///////////////Read data or  parameter
uint8_t LCD_DataRead(void)
{
	uint8_t Data;	
	//SCLK = 1;	
	//SDO = 1;	
	select_cs_lcd(ENABLE);//SCS = 0;
	//SPI_WR(0x40);  
	SPI_WR(0x40);
	Data = SPI_WR(0x00);
	select_cs_lcd(DISABLE);//SCS = 1;
	return Data;
}  

////////////////Write command and parameter
void Write_Dir(uint8_t Cmd,uint8_t Data)
{
  LCD_CmdWrite(Cmd);
  LCD_DataWrite(Data);
}

////////////////Read command and parameter
uint8_t Read_Dir(uint8_t Cmd)
{
  LCD_CmdWrite(Cmd);
  return  LCD_DataRead();
}


///////////SPI Read  status
uint8_t LCD_StatusRead(void)
{
	uint8_t Data;	
	select_cs_lcd(ENABLE);//SCS = 0;	
	SPI_WR(0xc0);
	Data = SPI_WR(0x00);
	select_cs_lcd(DISABLE);//SCS = 1;
	return Data;
}

////////LCM reset
void LCD_Reset(void)
{
	reset_bit(DISABLE);//MCU_RST = 0;
	Delay1ms(1);
	reset_bit(ENABLE);//MCU_RST = 1;
	Delay1ms(1);
}	

///////////////check busy
void Chk_Busy(void)
{
//	uint8_t temp; 	
//	do
//	{
//		temp=LCD_StatusRead();
//	}while((temp&0x80)==0x80);
//	Delay10ms(1);	
//	NextStep();
    
    while ((LCD_StatusRead() & 0x80) == 0x80);
}


void LCD_WaitBusy(void)
{
	while ((LCD_StatusRead() & 0x80) == 0x80);
}

///////////////check bte busy
void Chk_BTE_Busy(void)
{
//	uint8_t temp; 	
//	//do
//	//{
//	temp=LCD_StatusRead();
////	}while((temp&0x40)==0x40);		
    while ((LCD_StatusRead() & 0x40) == 0x40);
}
///////////////check dma busy
void Chk_DMA_Busy(void)
{
	uint8_t temp; 	
	//do
	{
	LCD_CmdWrite(0xBF);
	temp =LCD_DataRead();
	}//while((temp&0x01)==0x01);   
	rt_kprintf("LCD_DataRead = %d---",temp);
}

/////////////PLL setting
void PLL_ini(void)
{
    LCD_CmdWrite(0x88);      
    LCD_DataWrite(0x0C);
    Delay1ms(1);     
    LCD_CmdWrite(0x89);        
    LCD_DataWrite(0x02);  
    Delay1ms(1);
}	

/////////////////Set the working window area
void Active_Window(uint16_t XL,uint16_t XR ,uint16_t YT ,uint16_t YB)
{
	uint8_t temp;
    //setting active window X
	temp=XL;   
    LCD_CmdWrite(0x30);//HSAW0
	LCD_DataWrite(temp);
	temp=XL>>8;   
    LCD_CmdWrite(0x31);//HSAW1	   
	LCD_DataWrite(temp);

	temp=XR;   
    LCD_CmdWrite(0x34);//HEAW0
	LCD_DataWrite(temp);
	temp=XR>>8;   
    LCD_CmdWrite(0x35);//HEAW1	   
	LCD_DataWrite(temp);

    //setting active window Y
	temp=YT;   
    LCD_CmdWrite(0x32);//VSAW0
	LCD_DataWrite(temp);
	temp=YT>>8;   
    LCD_CmdWrite(0x33);//VSAW1	   
	LCD_DataWrite(temp);

	temp=YB;   
    LCD_CmdWrite(0x36);//VEAW0
	LCD_DataWrite(temp);
	temp=YB>>8;   
    LCD_CmdWrite(0x37);//VEAW1	   
	LCD_DataWrite(temp);
}

void LCD_HighSpeedSPI(void)
{
	SPI_InitTypeDef   SPI_InitStructure;

	SPI_Cmd(SPI3,DISABLE);


	/* 配置 SPI3工作模式 */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; 		/* 软件控制片选 */
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
    
	SPI_Init(SPI3,&SPI_InitStructure);

	/* 使能 SPI3 */
	SPI_Cmd(SPI3,ENABLE);
}




/////////////LCM initial
void LCD_Initial(void)
{ 	
    PLL_ini();
	LCD_CmdWrite(0x10);	 //SYSR   bit[4:3] color  bit[2:1]=  MPU interface
	LCD_DataWrite(0x0c);   //            65K						 
	
	LCD_CmdWrite(0x04);    //PCLK
	LCD_DataWrite(0x81);   //
	Delay1ms(1);

	 //Horizontal set
	LCD_CmdWrite(0x14); //HDWR//Horizontal Display Width Setting Bit[6:0]                      
	LCD_DataWrite(0x63);//Horizontal display width(pixels) = (HDWR + 1)*8                      
	LCD_CmdWrite(0x15);//Horizontal Non-Display Period Fine Tuning Option Register (HNDFTR)   
	LCD_DataWrite(0x00);//Horizontal Non-Display Period Fine Tuning(HNDFT) [3:0]               
	LCD_CmdWrite(0x16); //HNDR//Horizontal Non-Display Period Bit[4:0]                         
	LCD_DataWrite(0x03);//Horizontal Non-Display Period (pixels) = (HNDR + 1)*8                
	LCD_CmdWrite(0x17); //HSTR//HSYNC Start Position[4:0]                                      
	LCD_DataWrite(0x03);//HSYNC Start Position(PCLK) = (HSTR + 1)*8                            
	LCD_CmdWrite(0x18); //HPWR//HSYNC Polarity ,The period width of HSYNC.                     
	LCD_DataWrite(0x0B);//HSYNC Width [4:0]   HSYNC Pulse width(PCLK) = (HPWR + 1)*8    
	 //Vertical set                             
	LCD_CmdWrite(0x19); //VDHR0 //Vertical Display Height Bit [7:0]                            
	LCD_DataWrite(0xdf);//Vertical pixels = VDHR + 1                                           
	LCD_CmdWrite(0x1a); //VDHR1 //Vertical Display Height Bit [8]                              
	LCD_DataWrite(0x01);//Vertical pixels = VDHR + 1                                           
	LCD_CmdWrite(0x1b); //VNDR0 //Vertical Non-Display Period Bit [7:0]                        
	LCD_DataWrite(0x20);//Vertical Non-Display area = (VNDR + 1)                               
	LCD_CmdWrite(0x1c); //VNDR1 //Vertical Non-Display Period Bit [8]                          
	LCD_DataWrite(0x00);//Vertical Non-Display area = (VNDR + 1)                               
	LCD_CmdWrite(0x1d); //VSTR0 //VSYNC Start Position[7:0]                                    
	LCD_DataWrite(0x16);//VSYNC Start Position(PCLK) = (VSTR + 1)                              
	LCD_CmdWrite(0x1e); //VSTR1 //VSYNC Start Position[8]                                      
	LCD_DataWrite(0x00);//VSYNC Start Position(PCLK) = (VSTR + 1)                              
	LCD_CmdWrite(0x1f); //VPWR //VSYNC Polarity ,VSYNC Pulse Width[6:0]                        
	LCD_DataWrite(0x01);//VSYNC Pulse Width(PCLK) = (VPWR + 1)   
    
    LCD_CmdWrite(0x13); //GPO setting                     
	LCD_DataWrite(0x0D); // UD=0;  LR=1 ; DITHB=1 ; MODE=1
	
	LCD_HighSpeedSPI();
	Active_Window(0,799,0,479);
	
    LCD_CmdWrite(0x8a);//PWM setting
	LCD_DataWrite(0x80);
	LCD_CmdWrite(0x8a);//PWM setting
	LCD_DataWrite(0x81);//open PWM
	LCD_CmdWrite(0x8b);//Backlight brightness setting
	LCD_DataWrite(0xff);//Brightness parameter 0xff-0x00
}


///////////////Background color settings
void LCD_SetBackgroundColor(uint16_t b_color)
{
	
	LCD_CmdWrite(0x60);//BGCR0
	LCD_DataWrite((b_color & 0xF800) >> 11);
	
	LCD_CmdWrite(0x61);//BGCR0
	LCD_DataWrite((b_color & 0x07E0) >> 5);
	
	LCD_CmdWrite(0x62);//BGCR0
	LCD_DataWrite(b_color & 0x001F);
} 


////////////////Foreground color settings
void LCD_SetFontColor(uint16_t b_color)
{
	
	LCD_CmdWrite(0x63);//BGCR0
	LCD_DataWrite((b_color & 0xF800) >> 11);
	
	LCD_CmdWrite(0x64);//BGCR0
	LCD_DataWrite((b_color & 0x07E0) >> 5);
	
	LCD_CmdWrite(0x65);//BGCR0
	LCD_DataWrite(b_color & 0x001F);
} 


//////////////////Foreground color settings
//void Text_Foreground_Color(uint8_t setR,uint8_t setG,uint8_t setB)
//{	    
//    LCD_CmdWrite(0x63);//BGCR0
//	LCD_DataWrite(setR);
//   
//    LCD_CmdWrite(0x64);//BGCR0
//	LCD_DataWrite(setG);

//    LCD_CmdWrite(0x65);//BGCR0·
//	LCD_DataWrite(setB);
//}
//////////////////BTE area size settings
void BTE_Size(uint16_t width,uint16_t height)
{
    uint8_t temp;
	temp=width;   
    LCD_CmdWrite(0x5c);//BET area width literacy
	LCD_DataWrite(temp);
	temp=width>>8;   
    LCD_CmdWrite(0x5d);//BET area width literacy	   
	LCD_DataWrite(temp);

	temp=height;   
    LCD_CmdWrite(0x5e);//BET area height literacy
	LCD_DataWrite(temp);
	temp=height>>8;   
    LCD_CmdWrite(0x5f);//BET area height literacy	   
	LCD_DataWrite(temp);
}		

////////////////////BTE starting position
void BTE_Source(uint16_t SX,uint16_t DX ,uint16_t SY ,uint16_t DY)
{
	uint8_t temp,temp1;
    
	temp=SX;   
    LCD_CmdWrite(0x54);//BTE horizontal position of read/write data
	LCD_DataWrite(temp);
	temp=SX>>8;   
    LCD_CmdWrite(0x55);//BTE horizontal position of read/write data   
	LCD_DataWrite(temp);

	temp=DX;
    LCD_CmdWrite(0x58);//BET written to the target horizontal position
	LCD_DataWrite(temp);
	temp=DX>>8;   
    LCD_CmdWrite(0x59);//BET written to the target horizontal position	   
	LCD_DataWrite(temp); 
    
	temp=SY;   
    LCD_CmdWrite(0x56);//BTE vertical position of read/write data
	LCD_DataWrite(temp);
	temp=SY>>8;   
    LCD_CmdWrite(0x57);
	temp1 = LCD_DataRead();
	temp1 &= 0x80;
    temp=temp|temp1; 
	LCD_CmdWrite(0x57);//BTE vertical position of read/write data  
	LCD_DataWrite(temp);

	temp=DY;   
    LCD_CmdWrite(0x5a);//BET written to the target  vertical  position
	LCD_DataWrite(temp);
	temp=DY>>8;   
    LCD_CmdWrite(0x5b);
	temp1 = LCD_DataRead();
	temp1 &= 0x80;
	temp=temp|temp1;	
	LCD_CmdWrite(0x5b);//BET written to the target  vertical  position 
	LCD_DataWrite(temp);
}				
///////////////Memory write position
void MemoryWrite_Position(uint16_t X,uint16_t Y)
{
   
    Write_Dir(0x46, X);
    Write_Dir(0x47, X >> 8);
    Write_Dir(0x48, Y);
    Write_Dir(0x49, Y >> 8);
}

////////////////Text write position
void FontWrite_Position(uint16_t X,uint16_t Y)
{
	uint8_t temp;
	temp=X;   
    LCD_CmdWrite(0x2A);
	LCD_DataWrite(temp);
	temp=X>>8;   
    LCD_CmdWrite(0x2B);	   
	LCD_DataWrite(temp);

	temp=Y;   
    LCD_CmdWrite(0x2C);
	LCD_DataWrite(temp);
	temp=Y>>8;   
    LCD_CmdWrite(0x2D);	   
	LCD_DataWrite(temp);
}

//////////////writing text
void String(uint8_t *str)
{   
	//Write_Dir(0x40,0x80);//Set the character mode
	//LCD_CmdWrite(0x02);
	while(*str != '\0')
	{
	 Write_Dir(0x40,0x80);//Set the character mode
	 LCD_CmdWrite(0x02);
	 LCD_DataWrite(*str);
	 ++str;	 	
	 Chk_Busy();		
	} 
}




void String_font(uint8_t *str,int size)
{     
	while(*str != '\0')
	{
        Write_Dir(0x40,0x80);//Set the character mode
        
        if(size == 32)  
        {        
            Write_Dir(0x21,0x20);//External font ROM select
            Write_Dir(0x06,0x03);//set FLASH Frequency
            Write_Dir(0x2E,0x80);
            Write_Dir(0x2F,0x80);
            Write_Dir(0x05,0x28);   
        }   
        
        if(size == 24)  
        {
            Write_Dir(0x21,0x20);//External font ROM select    
            Write_Dir(0x06,0x03);//set FLASH Frequency          
            Write_Dir(0x2E,0x40);   //set font type(size)   
            Write_Dir(0x2F,0x80);  
            Write_Dir(0x05,0x28);   
        }   
        if(size == 16)  
        {
            Write_Dir(0x21,0x20);//External font ROM select
            Write_Dir(0x06,0x03);//set FLASH Frequency
            Write_Dir(0x2E,0x00);   //set font type(size)   
            Write_Dir(0x2F,0x80);   //set GT serial font ROM  bit7-5
            Write_Dir(0x05,0x28);   
        }   
        
        LCD_CmdWrite(0x02);
        LCD_DataWrite(*str);
        ++str;	 	
        Chk_Busy();		
	} 
}


void Draw_point(uint16_t X,uint16_t Y,uint16_t color)
{	
    MemoryWrite_Position(X,Y);	
	LCD_CmdWrite(0x02); 		
	LCD_DataWrite16(color);
}
		
/////////////////Scroll window size
void Scroll_Window(uint16_t XL,uint16_t XR ,uint16_t YT ,uint16_t YB)
{
	uint8_t temp;    
	temp=XL;   
    LCD_CmdWrite(0x38);//HSSW0
	LCD_DataWrite(temp);
	temp=XL>>8;   
    LCD_CmdWrite(0x39);//HSSW1	   
	LCD_DataWrite(temp);

	temp=XR;   
    LCD_CmdWrite(0x3c);//HESW0
	LCD_DataWrite(temp);
	temp=XR>>8;   
    LCD_CmdWrite(0x3d);//HESW1	   
	LCD_DataWrite(temp);   
    
	temp=YT;   
    LCD_CmdWrite(0x3a);//VSSW0
	LCD_DataWrite(temp);
	temp=YT>>8;   
    LCD_CmdWrite(0x3b);//VSSW1	   
	LCD_DataWrite(temp);

	temp=YB;   
    LCD_CmdWrite(0x3e);//VESW0
	LCD_DataWrite(temp);
	temp=YB>>8;   
    LCD_CmdWrite(0x3f);//VESW1	   
	LCD_DataWrite(temp);
}  

///////////////Window scroll offset Settings
void Scroll(uint16_t X,uint16_t Y)
{
	uint8_t temp;
    
	temp=X;   
    LCD_CmdWrite(0x24);//HOFS0
	LCD_DataWrite(temp);
	temp=X>>8;   
    LCD_CmdWrite(0x25);//HOFS1	   
	LCD_DataWrite(temp);
/*
	temp=Y;   
    LCD_CmdWrite(0x26);//VOFS0
	LCD_DataWrite(temp);
	temp=Y>>8;   
    LCD_CmdWrite(0x27);//VOFS1	   
	LCD_DataWrite(temp); */
}	   	  

///////////////The FLASH reading area   setting
void DMA_block_mode_size_setting(uint16_t BWR,uint16_t BHR,uint16_t SPWR)
{
  	LCD_CmdWrite(0xB4);
  	LCD_DataWrite(BWR);
  	LCD_CmdWrite(0xB5);
  	LCD_DataWrite(BWR>>8);

  	LCD_CmdWrite(0xB6);
  	LCD_DataWrite(BHR);
  	LCD_CmdWrite(0xB7);
  	LCD_DataWrite(BHR>>8);

  	LCD_CmdWrite(0xB8);
  	LCD_DataWrite(SPWR);
  	LCD_CmdWrite(0xB9);
  	LCD_DataWrite(SPWR>>8);  
}

/////////////FLASH read start position Settings
void DMA_Start_address_setting(uint32_t set_address)
{ 
  	LCD_CmdWrite(0xB0);
  	LCD_DataWrite(set_address);

  	LCD_CmdWrite(0xB1);
  	LCD_DataWrite(set_address>>8);

	LCD_CmdWrite(0xB2);
  	LCD_DataWrite(set_address>>16);

  	LCD_CmdWrite(0xB3);
  	LCD_DataWrite(set_address>>24);
}
///////////drawing circle
void  Draw_Circle(uint16_t X,uint16_t Y,uint16_t R)
{
	uint8_t temp;
    
	temp=X;   
    LCD_CmdWrite(0x99);
	LCD_DataWrite(temp);
	temp=X>>8;   
    LCD_CmdWrite(0x9a);	   
	LCD_DataWrite(temp);  
	  
	temp=Y;   
    LCD_CmdWrite(0x9b);
	LCD_DataWrite(temp);
	temp=Y>>8;   
    LCD_CmdWrite(0x9c);	   
	LCD_DataWrite(temp);

	temp=R;   
    LCD_CmdWrite(0x9d);
	LCD_DataWrite(temp);
} 



///////////drawing elliptic curve
void  Draw_Ellipse(uint16_t X,uint16_t Y,uint16_t R1,uint16_t R2)
{
	uint8_t temp;    
	temp=X;   
    LCD_CmdWrite(0xA5);
	LCD_DataWrite(temp);
	temp=X>>8;   
    LCD_CmdWrite(0xA6);	   
	LCD_DataWrite(temp);  
	  
	temp=Y;   
    LCD_CmdWrite(0xA7);
	LCD_DataWrite(temp);
	temp=Y>>8;   
    LCD_CmdWrite(0xA8);	   
	LCD_DataWrite(temp);

	temp=R1;   
    LCD_CmdWrite(0xA1);
	LCD_DataWrite(temp);
	temp=R1>>8;   
    LCD_CmdWrite(0xA2);	   
	LCD_DataWrite(temp);  
	  
	temp=R2;   
    LCD_CmdWrite(0xA3);
	LCD_DataWrite(temp);
	temp=R2>>8;   
    LCD_CmdWrite(0xA4);	   
	LCD_DataWrite(temp);
} 


///////////drawing line, rectangle, triangle
void Draw_Line(uint16_t XS,uint16_t XE ,uint16_t YS,uint16_t YE,uint16_t color)
{	
    Write_Dir(0x91,XS);
    Write_Dir(0x92,XS>>8);
	Write_Dir(0x93,YS);
    Write_Dir(0x94,YS>>8);
	Write_Dir(0x95,XE);
    Write_Dir(0x96,XE>>8);
	Write_Dir(0x97,YE);
    Write_Dir(0x98,YE>>8);
    
    LCD_SetFontColor(color);//Color Settings
	Write_Dir(0x90, (1 << 7) | (0 << 4) | (0 << 0));
    LCD_WaitBusy();
}

////////////draw a triangle of three point 
void Draw_Triangle(uint16_t X3,uint16_t Y3)
{
    uint8_t temp;    
	temp=X3;   
    LCD_CmdWrite(0xA9);
	LCD_DataWrite(temp);
	temp=X3>>8;   
    LCD_CmdWrite(0xAA);	   
	LCD_DataWrite(temp);

	temp=Y3;
    LCD_CmdWrite(0xAB);
	LCD_DataWrite(temp);
	temp=Y3>>8;   
    LCD_CmdWrite(0xAC);	   
	LCD_DataWrite(temp);
}


void Draw_line_window(uint16_t XS,uint16_t XE ,uint16_t YS,uint16_t YE,uint16_t color)
{
	Write_Dir(0X91,XS);
	Write_Dir(0X92,XS >> 8);
    Write_Dir(0X93,YS);
	Write_Dir(0X94,YS >> 8);
    
    Write_Dir(0X95,XE);
	Write_Dir(0X96,XE >> 8);
    Write_Dir(0X97,YE);
	Write_Dir(0X98,YE >> 8);
    
    LCD_SetFontColor(color);//Color Settings
    
    Write_Dir(0X90,(1 << 7) | (0 << 5) | (1 << 4) | (0 << 0));	/* 开始画矩形  */
    LCD_WaitBusy();
}


void Draw_line_window_fill(uint16_t XS,uint16_t XE ,uint16_t YS,uint16_t YE,uint16_t color)
{
	Write_Dir(0X91,XS);
	Write_Dir(0X92,XS >> 8);
    Write_Dir(0X93,YS);
	Write_Dir(0X94,YS >> 8);
    
    Write_Dir(0X95,XE);
	Write_Dir(0X96,XE >> 8);
    Write_Dir(0X97,YE);
	Write_Dir(0X98,YE >> 8);
    
    LCD_SetFontColor(color);//Color Settings
    
    Write_Dir(0X90,(1 << 7) | (1 << 5) | (1 << 4) | (0 << 0));	/* 开始画矩形  */
    LCD_WaitBusy();
}

/////////////Touch the interrupt judgment
uint8_t Touch_Status(void)
{	
    uint8_t temp;
	LCD_CmdWrite(0xF1);//INTC	
	temp =LCD_DataRead();
	if ((temp&0x04)==0x04)
	/*temp=LCD_StatusRead();
	if ((temp&0x10)==0x10)*/
	return 1;
	else 
	return 0;
}
//////////check interrupt flag bit
uint8_t Chk_INT(void)
{
	uint8_t temp; 	
	temp=LCD_StatusRead();
    if ((temp&0x20)==0x20)
	return 1;
	else 
	return 0;	   
}

uint8_t Chk_INT2(void)
{
	uint8_t temp; 	
    LCD_CmdWrite(0x74);//INTC
	temp =LCD_DataRead();
    if ((temp&0x80)==0x80)
	return 1;
	else 
	return 0;	   
}

/////////Read TP the X coordinate 
uint8_t ADC_X(void)
{
    uint8_t temp;
	LCD_CmdWrite(0x72);//TPXH	 X_coordinate high byte
	//Chk_Busy();
	temp=LCD_DataRead();
	return temp;
}

/////////Read TP the Y coordinate 
uint8_t ADC_Y(void)
{
    uint8_t temp;
	LCD_CmdWrite(0x73);//TPYH	  Y_coordinate high byte
    //Chk_Busy();
	temp=LCD_DataRead();
	return temp;
}

////////////Read TP the XY coordinates, the coordinates (high)
uint8_t ADC_XY(void)
{	
    uint8_t temp;
 	LCD_CmdWrite(0x74);//TPXYL	  bit[3:2] Y_coordinate low byte  bit[1:0] X_coordinate low byte 
	//Chk_Busy();
	temp=LCD_DataRead();
	return temp;
}   

//////////////Touch the coordinate display
void TP(void)	
{unsigned int lx,ly,i;
	Delay10ms(1);
  	X1=0;
  	X1|=ADC_X();
  	i=(X1<<2);
  	X1= i|((ADC_XY()&0x03));
  
  	Y1=0;
  	Y1|=ADC_Y();
  	i=(Y1<<2);
  	Y1=i|((ADC_XY()>>2)&0x03);

  	tabx[0]=X1/1000;
  	tabx[1]=X1%1000/100;
  	tabx[2]=X1%100/10;
  	tabx[3]=X1%10;
  	taby[0]=Y1/1000;
  	taby[1]=Y1%1000/100;
  	taby[2]=Y1%100/10;
  	taby[3]=Y1%10;

  	FontWrite_Position(100,60);   //Set the display position
  	LCD_CmdWrite(0x02);
  	String("X = ");
  	LCD_DataWrite(tabx[0] |= 0x30);
	//Delay1ms(1);
  	LCD_DataWrite(tabx[1] |= 0x30);
	//Delay1ms(1);
  	LCD_DataWrite(tabx[2] |= 0x30);
	//Delay1ms(1);
  	LCD_DataWrite(tabx[3] |= 0x30);
	//Delay1ms(1);

  	FontWrite_Position(100, 140);   //Set the display position
  	LCD_CmdWrite(0x02);
  	String("Y = ");
  	LCD_DataWrite(taby[0] |= 0x30);
	//Delay1ms(1);
  	LCD_DataWrite(taby[1] |= 0x30);
	//Delay1ms(1);
  	LCD_DataWrite(taby[2] |= 0x30);
	//Delay1ms(1);
  	LCD_DataWrite(taby[3] |= 0x30);
	//Delay1ms(1);

    Write_Dir(0x40,0x00);//The drawing mode
	
	lx=(((X1-60)*20/23));
	ly=(((Y1-58)*20/37));
					  
	MemoryWrite_Position(lx,ly);//Memory write position
    LCD_CmdWrite(0x02);//Memory write Data	
    LCD_DataWrite(0x07);
	LCD_DataWrite(0xe0);
	LCD_DataWrite(0xe0);
	MemoryWrite_Position(lx,ly+1);//Memory write position
    LCD_CmdWrite(0x02);////Memory write Data					   
	LCD_DataWrite(0xe0);
	LCD_DataWrite(0xe0);					 
			
	Write_Dir(0xf1,0x04);//clear INT state      Must be clean TP_interrupt 
	X1=0;
	Y1=0;
}

////////////Show the picture of the flash
void Displaypicture(uint8_t picnum)
{  
   uint8_t picnumtemp;
   Write_Dir(0X06,0X00);//FLASH frequency setting
   Write_Dir(0X05,0X87);//FLASH setting 

   picnumtemp=picnum;

   Write_Dir(0XBF,0X02);//FLASH setting
   Active_Window(0,799,0,479); 
   MemoryWrite_Position(0,0);//Memory write position
   DMA_Start_address_setting(768000*(picnumtemp-1));//DMA Start address setting
   DMA_block_mode_size_setting(800,480,800);
   Write_Dir(0XBF,0X03);//FLASH setting
   Chk_DMA_Busy();
} 

//////Shear pictures
//Shear pictures number:picnum
//display position:x1,y1,x2,y2
//the upper left of the shear image coordinates :x,y
void CutPictrue(uint8_t picnum,uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,unsigned long x,unsigned long y)
{
    unsigned long cutaddress;uint8_t picnumtemp;
    Write_Dir(0X06,0X00);//FLASH frequency setting   
   	Write_Dir(0X05,0Xac);//FLASH setting

	picnumtemp=picnum;
   
   	Write_Dir(0XBF,0X02);//FLASH setting
   	Active_Window(x1,x2,y1,y2);		
   	MemoryWrite_Position(x1,y1);//Memory write position
   	cutaddress=(picnumtemp-1)*768000+y*1600+x*2;
   	DMA_Start_address_setting(cutaddress);
   	DMA_block_mode_size_setting(x2-x1+1,y2-y1+1,800);
   	Write_Dir(0XBF,0X03);//FLASH setting
	Chk_DMA_Busy();
}


void int2str(uint16_t n, uint8_t *str)
{
    uint8_t buf[10] = "";
    int i = 0;
    int len = 0;
    int temp = n < 0 ? -n: n;  // temp?n????
	
	if(n==0)
	{
		str[0] = 'O';
		return;
	}
	if (str == NULL)
    {
       return;
	}
    while(temp)
    {
        buf[i++] = (temp % 10) + '0';  //?temp?????????buf
		temp = temp / 10;
	 }

	 len = n < 0 ? ++i: i;  //??n???,???????????
	 str[i] = 0;            //??????0
	 while(1)
	 {         
        i--;
		if (buf[len-i-1] ==0)
		{
            break;
		}
		 str[i] = buf[len-i-1];  //?buf???????????
	 }
	 if (i == 0 )
	 {
		 str[i] = '-';          //?????,??????
	 }
}

void draw_list3_com(unsigned int xs,unsigned int ys)
{
	int i=0;
	for(i=0;i<12;i++)
	{
		NextStep();NextStep();NextStep();NextStep();
		NextStep();NextStep();NextStep();
		//if(j==0)
		{
			if(i!=0)
				Active_Window(xs+i*65+2,xs+(i+1)*65-2,ys+2,ys+60-2);//Set the work window size
			else
				Active_Window(xs,xs+(i+1)*65-2,ys+2,ys+60-2);//Set the work window size
		}
		NextStep();NextStep();
		
		Write_Dir(0X8E,0X40);//Set the screen clearing properties window (work window)
		Write_Dir(0X8E,0XC0);//Began to clear the screen
		
		if(i>=8)
			FontWrite_Position(xs+4+i*65+2,105+60);//Text written to the position
		else
			FontWrite_Position(xs+4+i*65+2,105+50);//Text written to the position
		NextStep();
		Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
		NextStep();
		
		String(list3_word[i]);
		NextStep();NextStep();
		NextStep();NextStep();
	}
}

void draw_monitor(unsigned int xs,unsigned int ys)
{
	int i=0,j=0,start=0;
	uint8_t str;
	for(i=0;i<6;i++)
	{
		start=xs+4+130*i;
		
		NextStep();NextStep();NextStep();NextStep();
		NextStep();NextStep();NextStep();
		//Active_Window(start+6,start+125-6,ys+30,399);//Set the work window size
		NextStep();NextStep();

		NextStep();NextStep();NextStep();
		LCD_SetFontColor(color_black);//Color Settings
		Draw_line_window(start+5,start+130-5,ys,430,color_black);  //外框
		NextStep();NextStep();NextStep();NextStep();
		NextStep();NextStep();NextStep();
		for(j=0;j<3;j++)
		{
			NextStep();NextStep();NextStep();NextStep();NextStep();
			Draw_Line(start+5,start+130-5,ys+30+j*60,ys+30+j*60,color_black);

		}
	}
    
    
	for(i=0;i<6;i++)
	{
		start=xs+4+130*i;
        FontWrite_Position(start+10,ys+5);//Text written to the position
        NextStep();
        Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
        NextStep();
        LCD_SetBackgroundColor(color_white);//Set the background color
        String("端口 ");
        int2str(i+1,&str);
        String(&str);
	}
}


void sensor_data_updata(int num)
{
    char port_num=0;
    int timetick = 0;
    int ret=0;
    int order_num=0;
	int i=0,j=0,start=0;
	unsigned int xs = 4;
    unsigned int ys = 220;
    int temp=0;
    timetick = time(RT_NULL);                //获取当前时间戳
    while(i<num)
    {
        ret =0;       
        port_num = sensor_data[i].port-1;
        if(sensor_data[i].subAddr != 0)
        {
            if((sensor_data_save[port_num][0].state!=0)&&(sensor_data_save[port_num][1].state!=0)&&(sensor_data_save[port_num][2].state!=0))
            {
                if( ( ((sensor_data[i].type == sensor_data_save[port_num][0].type) && (sensor_data[i].subAddr != sensor_data_save[port_num][0].addr)) || 
                     (sensor_data[i].type != sensor_data_save[port_num][0].type) )        &&
                    ( ((sensor_data[i].type == sensor_data_save[port_num][1].type) && (sensor_data[i].subAddr != sensor_data_save[port_num][1].addr)) || 
                     (sensor_data[i].type != sensor_data_save[port_num][1].type) )        &&
                    ( ((sensor_data[i].type == sensor_data_save[port_num][2].type) && (sensor_data[i].subAddr != sensor_data_save[port_num][2].addr)) || 
                     (sensor_data[i].type != sensor_data_save[port_num][2].type) )  )
                {
                    for(j=0;j<3;j++)
                    {
                        if(((timetick - sensor_data_save[port_num][j].timestamp)>=5) )           //5秒内未更新数据
                        {
                            if( sensor_data_save[port_num][j].state == 3)
                            {
                                order_num = j;
                                sensor_data_save[port_num][order_num].type = 0;
                                sensor_data_save[port_num][order_num].addr = 0;
                                sensor_data_save[port_num][order_num].timestamp = 0;
                                sensor_data_save[port_num][order_num].state = 0;
                                sensor_data_save[port_num][order_num].data_last = 0; 
                                sensor_data_save[port_num][order_num].data_now = 0;
                                sensor_data_save[port_num][order_num].updata_state = 0;
                                sensor_data_save[port_num][order_num].sign = 0;
                                ret = 1;
                                break;
                            }
                            else
                                sensor_data_save[port_num][j].state == 3;
                            
                        }    
                    }
                }
                else
                {
                    for(j=0;j<3;j++)
                    {
                        if(sensor_data_save[port_num][j].type == sensor_data[i].type)
                        {
                            if(sensor_data_save[port_num][j].addr == sensor_data[i].subAddr)
                            {
                                order_num = j;
                                ret = 1;
                                break;
                            }                 
                        }
                    }
                }
            }       
            else 
            {
                for(j=0;j<3;j++)
                {
                    if(sensor_data_save[port_num][j].state != 0)
                    {
                        if(sensor_data_save[port_num][j].type == sensor_data[i].type)
                        {
                            if(sensor_data_save[port_num][j].addr == sensor_data[i].subAddr)
                            {
                                order_num = j;
                                ret = 1;
                                break;
                            }                 
                        }
                    }
                    else if((sensor_data_save[port_num][j].state == 0) && (ret==0))
                    {
                        order_num = j;
                        ret = 1;
			break;
                    }  
                }
                
            }
            if(ret == 1)
            {
                sensor_data_save[port_num][order_num].timestamp = timetick;          //赋值当前时间戳
                if(sensor_data_save[port_num][order_num].state == 0)
                    sensor_data_save[port_num][order_num].state = 1;
                sensor_data_save[port_num][order_num].type = sensor_data[i].type;
                sensor_data_save[port_num][order_num].addr = sensor_data[i].subAddr;
                sensor_data_save[port_num][order_num].data_now = ConverToByeFloat(&sensor_data[i].payload[7]);
		sensor_data_save[port_num][order_num].updata_state = 1;
            }
        }
        
        i++;       
    }

    
    temp = 0;    
    for(i=0;i<6;i++)
    {   
        if(temp == 1)
            i--;
        
        temp = 0;
        
        for(j=2;j>0;j--)
        {
            if((sensor_data_save[i][j].state != 0) && (sensor_data_save[i][j-1].state == 0))
            {
                if( (j == 1) && (sensor_data_save[i][2].state != 0) )
                    temp = 1;
                sensor_data_save[i][j-1].type = sensor_data_save[i][j].type;
                sensor_data_save[i][j-1].addr = sensor_data_save[i][j].addr;
                sensor_data_save[i][j-1].timestamp = sensor_data_save[i][j].timestamp ;
                
                if(sensor_data_save[i][j].state == 2)
                    sensor_data_save[i][j-1].state = 1;
                else
                    sensor_data_save[i][j].state = sensor_data_save[i][j-1].state;
                    
                sensor_data_save[i][j-1].data_last = sensor_data_save[i][j].data_last; 
                sensor_data_save[i][j-1].data_now = sensor_data_save[i][j].data_now;
                sensor_data_save[i][j-1].updata_state = sensor_data_save[i][j].updata_state;
				
                if(sensor_data_save[i][j].state == 3)
                    sensor_data_save[i][j-1].sign = 0;
                else
                    sensor_data_save[i][j-1].sign = sensor_data_save[i][j].sign;
                
                sensor_data_save[i][j].type = 0;
                sensor_data_save[i][j].addr = 0;
                sensor_data_save[i][j].timestamp = 0;
                sensor_data_save[i][j].state = 0;
                sensor_data_save[i][j].data_last = 0; 
                sensor_data_save[i][j].data_now = 0;
                sensor_data_save[i][j].updata_state = 0;
                sensor_data_save[i][j].sign = 0;
            }
        }
    }
    
    //显示屏上传感数据显示
    for(i=0;i<6;i++)
    {
        start=xs+4+130*i; 
        for(j=0;j<3;j++)
        {
            if(sensor_data_save[i][j].state == 0)
            {
                NextStep();NextStep();NextStep();NextStep();NextStep();
                Active_Window(start+5+1,start+130-6,ys+30+j*60+1,ys+30+(j+1)*60-1);
                NextStep();NextStep();
                LCD_SetBackgroundColor(color_white);
                Write_Dir(0X8E,0X40);//Set the screen clearing properties window (work window)
                Write_Dir(0X8E,0XC0);//Began to clear the screen
                NextStep();NextStep();NextStep();NextStep();NextStep();
            }
            else if(sensor_data_save[i][j].state == 1)
            {
                NextStep();NextStep();NextStep();NextStep();NextStep();
                Active_Window(start+5+1,start+130-6,ys+30+j*60+1,ys+30+(j+1)*60-1);
                NextStep();NextStep();
                LCD_SetBackgroundColor(color_green);
                Write_Dir(0X8E,0X40);//Set the screen clearing properties window (work window)
                Write_Dir(0X8E,0XC0);//Began to clear the screen
                NextStep();NextStep();NextStep();NextStep();NextStep();
                sensor_type_addr_show(start+10,ys+30+j*60+10,sensor_data_save[i][j].type,sensor_data_save[i][j].addr,sensor_data_save[i][j].data_now,1);
                sensor_data_save[i][j].data_last = sensor_data_save[i][j].data_now;
                sensor_data_save[i][j].updata_state = 0;
                sensor_data_save[i][j].state = 2;
                sensor_data_save[i][j].sign = 0;                
            }
            else if(sensor_data_save[i][j].state == 2)
            {
                if(sensor_data_save[i][j].data_now != sensor_data_save[i][j].data_last)
                {
                    NextStep();NextStep();NextStep();NextStep();NextStep();
                    Active_Window(start+5+1,start+130-6,ys+30+j*60+1+30,ys+30+(j+1)*60-1);
                    NextStep();NextStep();
                    LCD_SetBackgroundColor(color_green);
                    Write_Dir(0X8E,0X40);//Set the screen clearing properties window (work window)
                    Write_Dir(0X8E,0XC0);//Began to clear the screen
                    NextStep();NextStep();NextStep();NextStep();NextStep();
                    sensor_type_addr_show(start+10,ys+30+j*60+10,sensor_data_save[i][j].type,sensor_data_save[i][j].addr,sensor_data_save[i][j].data_now,0);
                    sensor_data_save[i][j].data_last = sensor_data_save[i][j].data_now;
                    sensor_data_save[i][j].updata_state = 0;
                } 
                else if((sensor_data_save[i][j].data_now == sensor_data_save[i][j].data_last) )
                {
                    if((timetick - sensor_data_save[i][j].timestamp )>= 5 )
                    {
                        sensor_data_save[i][j].state = 3;
                        sensor_data_save[i][j].updata_state = 0;
                        sensor_data_save[i][j].sign = 0;
                    }
                }                
            }
            else if(sensor_data_save[i][j].state == 3)
            {
                if(((timetick - sensor_data_save[i][j].timestamp)>=10) )           //5秒内未更新数据
                {
                    sensor_data_save[i][j].type = 0;
                    sensor_data_save[i][j].addr = 0;
                    sensor_data_save[i][j].timestamp = 0;
                    sensor_data_save[i][j].state = 0;
                    sensor_data_save[i][j].data_last = 0; 
                    sensor_data_save[i][j].data_now = 0;
                    sensor_data_save[i][j].updata_state = 0;
                    sensor_data_save[i][j].sign = 0;
                    break;
                }    
                else 
                {
                    if(sensor_data_save[i][j].updata_state == 1)
                    {
                        NextStep();NextStep();NextStep();NextStep();NextStep();
                        Active_Window(start+5+1,start+130-6,ys+30+j*60+1,ys+30+(j+1)*60-1);
                        NextStep();NextStep();
                        LCD_SetBackgroundColor(color_green);
                        Write_Dir(0X8E,0X40);//Set the screen clearing properties window (work window)
                        Write_Dir(0X8E,0XC0);//Began to clear the screen
                        NextStep();NextStep();NextStep();NextStep();NextStep();
                        sensor_type_addr_show(start+10,ys+30+j*60+10,sensor_data_save[i][j].type,sensor_data_save[i][j].addr,sensor_data_save[i][j].data_now,1); 
                        sensor_data_save[i][j].state = 2;
                        sensor_data_save[i][j].updata_state = 0;
                        sensor_data_save[i][j].sign = 0;
                    }
                    else if(sensor_data_save[i][j].updata_state == 0)
                    {
                        
                        if( sensor_data_save[i][j].sign == 0 )
                        {
                            NextStep();NextStep();NextStep();NextStep();NextStep();
                            Active_Window(start+5+1,start+130-6,ys+30+j*60+1,ys+30+(j+1)*60-1);
                            NextStep();NextStep();
                            LCD_SetBackgroundColor(color_red);
                            Write_Dir(0X8E,0X40);//Set the screen clearing properties window (work window)
                            Write_Dir(0X8E,0XC0);//Began to clear the screen
                            NextStep();NextStep();NextStep();NextStep();NextStep();
                            sensor_type_addr_show(start+10,ys+30+j*60+10,sensor_data_save[i][j].type,sensor_data_save[i][j].addr,sensor_data_save[i][j].data_now,1);       
                            sensor_data_save[i][j].updata_state == 0;
                            sensor_data_save[i][j].sign = 1;
                        } 
                    }                   
                }
                
            }
                
        }
    }  
}


void Systime_transform_1806time(char  *p_buf,char *p_time)
{
	char buf_temp[4];
	rt_memcpy(p_buf,p_time+20,4);
	rt_memcpy(p_buf+4,"-",1);
	
	rt_memcpy(buf_temp,p_time+4,3);
	*(buf_temp +3) = '\0';
	if(strcmp(buf_temp,"Jan") == 0)
		rt_memcpy(p_buf+5,"01",2);
	else if(strcmp(buf_temp,"Feb") == 0)
		rt_memcpy(p_buf+5,"02",2);
	else if(strcmp(buf_temp,"Mar") == 0)
		rt_memcpy(p_buf+5,"03",2);
	else if(strcmp(buf_temp,"Apr") == 0)
		rt_memcpy(p_buf+5,"04",2);
	else if(strcmp(buf_temp,"May") == 0)
		rt_memcpy(p_buf+5,"05",2);
	else if(strcmp(buf_temp,"Jun") == 0)
		rt_memcpy(p_buf+5,"06",2);
	else if(strcmp(buf_temp,"Jul") == 0)
		rt_memcpy(p_buf+5,"07",2);
	else if(strcmp(buf_temp,"Aug") == 0)
		rt_memcpy(p_buf+5,"08",2);
	else if(strcmp(buf_temp,"Sep") == 0)
		rt_memcpy(p_buf+5,"09",2);
	else if(strcmp(buf_temp,"Oct") == 0)
		rt_memcpy(p_buf+5,"10",2);
	else if(strcmp(buf_temp,"Nov") == 0)
		rt_memcpy(p_buf+5,"11",2);
	else if(strcmp(buf_temp,"Dec") == 0)
		rt_memcpy(p_buf+5,"12",2);
		
	rt_memcpy(p_buf+7,"-",1);
	rt_memcpy(p_buf+8,p_time+8,3);
	
	rt_memcpy(p_buf+11,p_time+11,9);
	rt_memcpy(p_buf+20,p_time,3);	
	*(p_buf+23) = '\0';
}




void time_show()
{
	static char time_num = 0;
	rt_time_t timetick = time(RT_NULL);
	char buf_time[25]; 

	time_num ++;
	if(time_num == 4)
	{
		time_num = 0;
		Systime_transform_1806time(buf_time,ctime(&timetick));
		LCD_QuitWinMode();
		FontWrite_Position(550,450);//Text written to the position
		NextStep();NextStep();
		Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
		NextStep();NextStep();
		LCD_SetBackgroundColor(color_white);//Set the background color

		String((uint8_t *)buf_time);

		NextStep();NextStep();NextStep();NextStep();
		NextStep();NextStep();NextStep();NextStep();
	}
	
}


void Draw_display_frame()
{
	int i=0,j=0;
	unsigned char str;
	LCD_SetBackgroundColor(color_white);//Set the background color
	
	NextStep();NextStep();NextStep();NextStep();NextStep();NextStep();
	Write_Dir(0x2E,0x40);//16 X16
	NextStep();NextStep();NextStep();NextStep();NextStep();NextStep();
	LCD_SetFontColor(color_red);//Color Settings
	FontWrite_Position(270,8);//Text written to the position
	String("深圳市翌日科技综合分站");
	
	NextStep();NextStep();NextStep();NextStep();NextStep();
	Draw_line_window(start_xrow,795,2,475,color_black);  //外框
	NextStep();NextStep();NextStep();
	NextStep();NextStep();NextStep();NextStep();NextStep();
	LCD_SetFontColor(color_black);//Color Setting
	NextStep();
	
	/************************************前几排的横线*********************************************/
	NextStep();NextStep();NextStep();NextStep();NextStep();
	for(i=0;i<4;i++)
	{
		if(i==0)
		{
			Draw_Line(start_xrow,794,33+i*50-1,33+i*50-1,color_black);
		}
		
		if(i==0 || i==1)
			Draw_Line(start_xrow,794,33+i*50+1,33+i*50+1,color_black);
		else 
			Draw_Line(start_xrow,794,33+i*50+(i-1)*10+1,33+i*50+(i-1)*10+1,color_black);
	}
	/************************************一二排的竖线*********************************************/
	NextStep();NextStep();NextStep();NextStep();NextStep();
	for(i=0;i<=8;i++)
	{
		NextStep();NextStep();NextStep();NextStep();NextStep();
		Draw_Line(8+i*98-1,8+i*98-1,33,143,color_black);
		
		NextStep();NextStep();NextStep();NextStep();NextStep();
		Draw_Line(8+i*98+1,8+i*98+1,33,143,color_black);
	}
	/*********************************百兆光，千兆故竖线*************************************/
	for(i=0;i<=12;i++)
	{
		NextStep();NextStep();NextStep();NextStep();NextStep();
		if(i!=12)
			Draw_Line(8+i*65-1,8+i*65-1,145,205,color_black);
		else
			Draw_Line(8+i*65+3,8+i*65+3,145,205,color_black);
		NextStep();NextStep();NextStep();NextStep();NextStep();
		if(i!=12)
			Draw_Line(8+i*65+1,8+i*65+1,145,205,color_black);
		else
			Draw_Line(8+i*65+5,8+i*65+5,145,205,color_black);
		
	}
		/************************************传感器显示*******************************************/
	NextStep();NextStep();NextStep();NextStep();NextStep();
	draw_monitor(4,220);
/*********************************百兆光，千兆光* 显示***********************************************/
		NextStep();NextStep();NextStep();NextStep();NextStep();
	
	draw_list3_com(10,144);
/*********************************************************************************/	
	NextStep();NextStep();NextStep();NextStep();
	NextStep();NextStep();NextStep();NextStep();NextStep();
	for(j=0;j<2;j++)
	{
		for(i=0;i<8;i++)
		{
			NextStep();NextStep();NextStep();NextStep();
			NextStep();NextStep();NextStep();
			if(j==0)
			{
				Active_Window(8+i*98+2,8+(i+1)*98-2,34+50*j+2,84+50*j-2);//Set the work window size
			}
			else
			{
				Active_Window(8+i*98+2,8+(i+1)*98-2,84+2,84+60-2);//Set the work window size
			}
			NextStep();NextStep();

			Write_Dir(0X8E,0X40);//Set the screen clearing properties window (work window)
			Write_Dir(0X8E,0XC0);//Began to clear the screen
			
			NextStep();NextStep();NextStep();
			if(j==0)
			{
				FontWrite_Position(28+i*98+2,55);//Text written to the position
				NextStep();
				Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
				NextStep();
				if(i==0)
					String("  监测 ");
				else if(i==1)
					String("  广播");
				else if(i==2)
					String("  定位");
				else if(i==3)
					String("  WIFI");
				else 
					String("未添加");
				NextStep();NextStep();
			}
			else if(j==1)
			{
				FontWrite_Position(28+i*98+2,105);//Text written to the position
				NextStep();
				Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
				NextStep();
				String(" 电口");
				int2str(i+1,&str);
				String((uint8_t *)&str);
				NextStep();NextStep();
			}
		}
	}

	FontWrite_Position(430,450);//Text written to the position
	NextStep();
	Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
	NextStep();
	LCD_SetBackgroundColor(color_white);//Set the background color
	//String("服务器已连接      2018-09-17 15:11");
	String("服务器已连接");
//	time_show();
	NextStep();NextStep();NextStep();NextStep();NextStep();NextStep();

	LCD_FUNCTION_STATE_SET(locate_x,locate_y,evacuate_state);
	LCD_FUNCTION_STATE_SET(radio_x,radio_y,broadcast_state);
	LCD_FUNCTION_STATE_SET(minitor_x,minitor_y,broadcast_state);
}


void LCD_Show()
{
	MSG_CENTER_HEADER_T  * pCenterHeader;
	struct nwkhdr *pNwkHdr ;
   	APP_HDR_T *pstAppHdr ;
	SENSOR_DATA *psensor;
	uint8_t rev_buf[512];
	int i;
    	int num=0;
	if(rt_mq_recv(&sensor_mq, rev_buf, sizeof(rev_buf), RT_WAITING_NO) == RT_EOK)
	{
		pCenterHeader = (MSG_CENTER_HEADER_T *)rev_buf;
		pNwkHdr = (struct nwkhdr *)(pCenterHeader+1);
		pstAppHdr = (APP_HDR_T *)(pNwkHdr + 1);
		psensor = (SENSOR_DATA *)(pstAppHdr +1);
       		num = pstAppHdr->len /sizeof(SENSOR_DATA); 
	        for(i=0;i<num;i++)
	        {
			sensor_data[i].port = psensor->port;
			sensor_data[i].subAddr = psensor->subAddr;
			sensor_data[i].type = psensor->type;
			sensor_data[i].subLen = psensor->subLen;
			rt_memcpy(sensor_data[i].payload,psensor->payload,sensor_data[i].subLen);
			psensor++;
	        }    		
	}
	sensor_data_updata(num);
}


struct COORDINATED_CONFIG coordinated_config[20];
static uint8_t flash_buf[2048] ; 
 uint8_t flash_buf[2048];
static uint8_t coordinated_num = 0;
uint8_t radio_type = 0;
uint8_t _3g_type = 0;
static uint8_t flash_read_state = 0;

void coordinated_recv()
{
	uint8_t rev_buf[512];
	uint8_t send_buf[128];
	static uint8_t radio_time = 0;
	static uint8_t _3g_time = 0;
	static uint8_t radio_sta = 0;
	static uint8_t _3g_sta = 0;

	int i=0;
	int offset = 0;
	rt_err_t err = 0;
	struct COORDINATED_CONDITION_CONFIG pcondition_config[5];
	struct COORDINATED_ACTION *paction;
	struct COORDINATED_ACK pack;
	PACKET_HEADER_T PacketTag;
	struct COORDINATED_ACK_INFORMATION ack_information ;

	if(!flash_read_state)
	{
		if( flash_read(OPTION_FLASH_ADDR, (u8*)flash_buf, sizeof(flash_buf)))
		{
			flash_data_config((uint8_t *)flash_buf);
			flash_read_state = 1;
		}
		else 
			flash_read_state = 0;
	}
	
	if(rt_mq_recv(&coordinated_mq, rev_buf, sizeof(rev_buf), RT_WAITING_NO) == RT_EOK)
	{
		struct nwkhdr *pNwkHdr = (struct nwkhdr *)rev_buf;
		APP_HDR_T *pstAppHdr  = (APP_HDR_T *)(pNwkHdr + 1);
		uint8_t recv_state = 0;
		uint8_t coordinated_order = 0;

		if( pstAppHdr ->msgtype == COMM_DOWN_COORDINATED_CONFIG)
		{
			uint8_t j = 0,k = 0;
			struct COORDINATED_CONDITION *pcondition = (struct COORDINATED_CONDITION *)(pstAppHdr+1);
			rt_memcpy(pcondition_config,pcondition+1,pcondition->ConditionLen);
			paction = (struct COORDINATED_ACTION *)((uint8_t *)pcondition + offset);

			for(j=0;j<(coordinated_num+1);j++)
			{
				if(!recv_state)
				{
					if(pcondition->ConditionLen == coordinated_config[j].coordinated_condition.ConditionLen)
					{
						for(k=0;k<(pcondition->ConditionLen/sizeof(struct COORDINATED_CONDITION_CONFIG));k++)
						{	
							if((coordinated_config[j].coordinated_condition_config[k].port ==  pcondition_config[k].port) && 
							(coordinated_config[j].coordinated_condition_config[k].subAddr ==  pcondition_config[k].subAddr))
							{
								if(k == (pcondition->ConditionLen/sizeof(struct COORDINATED_CONDITION_CONFIG )-1))
								{	
									recv_state = 1;
									coordinated_order = j;
									break;
								}
							}
							else
							{
								if(j ==  coordinated_num)
								{
									recv_state = 1;
									coordinated_order = coordinated_num;
									coordinated_num++;
									break;
								}
								
							}
						}
					}	
					else
					{
						if(j ==  coordinated_num)
							coordinated_order = coordinated_num;
						coordinated_num++;
						break;
					}
				}
				else
					break;				
			}
			
			rt_memcpy(&coordinated_config[coordinated_order].coordinated_condition,pcondition,sizeof(struct COORDINATED_CONDITION));
			offset += sizeof(struct COORDINATED_CONDITION);
			
			rt_memcpy(&coordinated_config[coordinated_order].coordinated_condition_config,pcondition_config,pcondition->ConditionLen);
			offset += pcondition->ConditionLen;

			rt_memcpy(&coordinated_config[coordinated_order].coordinated_action,paction,sizeof(struct COORDINATED_ACTION));
			offset += sizeof(struct COORDINATED_ACTION);
				
			if(paction->Evacuate == 1)
			{	
				struct EVACUATE_PACKAGE  *pevacuate_package = (struct EVACUATE_PACKAGE  *)((char *)pcondition + offset);
				struct EVACUATE_INFORMATION *pevacuate =(struct EVACUATE_INFORMATION *)(pevacuate_package+1) ;

				rt_memcpy(&coordinated_config[coordinated_order].evacuate_pack,pevacuate_package,sizeof(struct EVACUATE_PACKAGE));
				rt_memcpy(&coordinated_config[coordinated_order].evacuate_information,pevacuate,pevacuate_package->EvacuateListLen);
				offset += pevacuate_package->EvacuateListLen + sizeof(struct EVACUATE_PACKAGE);
			}
			if(paction->Broadcast == 1)
			{
				struct BROADCAST_PACKAGE  *pbroadcast_package = (struct BROADCAST_PACKAGE  *)((char *)pcondition + offset);  
				struct BROADCAST_INFORMATION *pbroadcast=(struct BROADCAST_INFORMATION *)(pbroadcast_package+1);

				rt_memcpy(&coordinated_config[coordinated_order].broadcast_pack,pbroadcast_package,sizeof(struct BROADCAST_PACKAGE));
				rt_memcpy(&coordinated_config[coordinated_order].broadcast_information,pbroadcast,pbroadcast_package->BroadcastListLen);
				offset += pbroadcast_package->BroadcastListLen + sizeof(struct BROADCAST_PACKAGE);
			}
			
			rt_kprintf("COMM_DOWN_COORDINATED_CONFIG data had received\n");

			//ACK
			pack.state = 1;
			pack.Len =  pcondition->ConditionLen /sizeof(struct COORDINATED_CONDITION_CONFIG) * sizeof(struct COORDINATED_ACK_INFORMATION);
			
			pstAppHdr->len += sizeof(struct COORDINATED_ACK) + pack.Len ;
			pNwkHdr->src = sys_option.u32BsId;
			pNwkHdr->dst = 0xffff;
			pNwkHdr->len = sizeof(APP_HDR_T) + pstAppHdr->len;

			PacketTag.tag = COMM_INTEGRATED_STA_DATA;
			PacketTag.len = sizeof(struct nwkhdr) + pNwkHdr->len;

			rt_memcpy(send_buf, &PacketTag, sizeof(PACKET_HEADER_T));
			offset += sizeof(PACKET_HEADER_T);
			
			rt_memcpy(send_buf+offset, pNwkHdr, sizeof(struct nwkhdr ));
			offset += sizeof(struct nwkhdr);
			rt_memcpy(send_buf+offset, pstAppHdr, sizeof(APP_HDR_T));
			offset += sizeof(APP_HDR_T);

			rt_memcpy(send_buf+offset,&pack,sizeof(struct COORDINATED_ACK));
			offset += sizeof(struct COORDINATED_ACK);

			for(i=0;i<pcondition->ConditionLen /sizeof(struct COORDINATED_CONDITION_CONFIG);i++)
			{
				ack_information.port = coordinated_config[coordinated_order].coordinated_condition_config[i].port;
				ack_information.subAddr = coordinated_config[coordinated_order].coordinated_condition_config[i].subAddr;
				rt_memcpy(send_buf+offset, &ack_information, sizeof(struct COORDINATED_ACK_INFORMATION));
				offset += sizeof(struct COORDINATED_ACK_INFORMATION);
			}
			err = rt_mq_send(&net_mq, send_buf, PacketTag.len + sizeof(PACKET_HEADER_T));	
			if(err == -RT_EFULL)
			{
		   		 rt_kprintf("coordinated config send to net_mq full\n");
			}
			else if(err == -RT_ERROR)
			{
		    		rt_kprintf("coordinated config send to net_mq err\n");
			}
			
			rt_kprintf("  coordinated_num = %d \n",coordinated_num);
			save_coordinateddata_to_flash((uint8_t *)&coordinated_config, (uint32_t )coordinated_num, coordinated_num * sizeof(struct COORDINATED_CONFIG));
		}
		else if( pstAppHdr ->msgtype == COMM_DOWN_COORDINATED_CONFIG_DELETE)
		{
			uint8_t j = 0,k = 0;
			struct COORDINATED_DELECT coordinated_delect_buf[5];
			struct COORDINATED_ACK coordinated_delect_ack;
			struct COORDINATED_DELECT *coordinated_delect = (struct COORDINATED_DELECT *)(pstAppHdr+1);

			uint8_t sensor_num = pstAppHdr->len/sizeof(struct COORDINATED_DELECT);
			
			rt_memcpy(coordinated_delect_buf,coordinated_delect,pstAppHdr->len);
			
			//ACK
			pNwkHdr->len += sizeof(struct COORDINATED_ACK);
			pNwkHdr->src = sys_option.u32BsId;
			pNwkHdr->dst = 0xffff;
			pstAppHdr->len += sizeof(struct COORDINATED_ACK);
			coordinated_delect_ack.state = 1;
			coordinated_delect_ack.Len = pstAppHdr->len;

			rt_memcpy(send_buf, pNwkHdr, sizeof(struct nwkhdr ));
			offset += sizeof(struct nwkhdr);
			rt_memcpy(send_buf+offset, pstAppHdr, sizeof(APP_HDR_T));
			offset += sizeof(APP_HDR_T);

			rt_memcpy(send_buf+offset,&coordinated_delect_ack,sizeof(struct COORDINATED_ACK));
			offset += sizeof(struct COORDINATED_ACK);
			rt_memcpy(send_buf+offset,coordinated_delect,coordinated_delect_ack.Len);
			
			for(j=0;j<coordinated_num;j++)
			{
				if(sensor_num == coordinated_config[j].coordinated_condition.ConditionLen/sizeof(struct COORDINATED_CONDITION_CONFIG))
				{
					for(k=0;k<sensor_num;k++)
					{	
						if((coordinated_config[j].coordinated_condition_config[k].port ==  coordinated_delect_buf[k].port) && 
						(coordinated_config[j].coordinated_condition_config[k].subAddr ==  coordinated_delect_buf[k].subAddr))
						{
							if(k == sensor_num-1)
							{
								int l = 0;
								for(l = j;l < coordinated_num-1 ;l++)
								{
									rt_memcpy(&coordinated_config[l],&coordinated_config[l+1],sizeof(struct COORDINATED_CONFIG));
								}
								rt_memset(&coordinated_config[coordinated_num-1], 0, sizeof(struct COORDINATED_CONFIG));
								coordinated_num --;
							}
							
						}
						else
							break;
					}
				}	
			}
			
			save_coordinateddata_to_flash((uint8_t *)&coordinated_config, (uint32_t )(coordinated_num-1), coordinated_num * sizeof(struct COORDINATED_CONFIG));

			err = rt_mq_send(&net_mq, send_buf, PacketTag.len + sizeof(PACKET_HEADER_T));	
			if(err == -RT_EFULL)
			{
		   		rt_kprintf("coordinated config delect send to net_mq full\n");
			}
			else if(err == -RT_ERROR)
			{
		    		rt_kprintf("coordinated config delect send to net_mq err\n");
			}		
            		rt_kprintf("COMM_DOWN_COORDINATED_CONFIG_DELECT data had received\n");			
		}
		
	}
	else
	{
		for(i=0;i<20;i++)
		{
			uint8_t num_condition = coordinated_config[i].coordinated_condition.ConditionLen/sizeof(struct COORDINATED_CONDITION_CONFIG);
			if(num_condition != 0)
			{
				if((num_condition > 0)&&(num_condition < 5))
				{
					int j=0;
					for(j = 0;j<num_condition;j++)
					{	
						if(!Is_coordinated_condition(coordinated_config[i].coordinated_condition_config[j].port,coordinated_config[i].coordinated_condition_config[j].subAddr,coordinated_config[i].coordinated_condition_config[j].symbol,coordinated_config[i].coordinated_condition_config[j].value))
						{
							radio_type = RADIO_EVACUATE_CANCLE_ALARM;
							_3g_type = 0;
							break;
						}
		
						if(j == num_condition-1)
						{
							rt_kprintf("The condition is satisfying.\n" );
							if(coordinated_config[i].coordinated_action.SensorWarn == 1)
							{
								
							}
							if(coordinated_config[i].coordinated_action.Evacuate == 1)
							{
								uint8_t Evacuate_num = coordinated_config[i].evacuate_pack.EvacuateListLen /sizeof(struct EVACUATE_INFORMATION);
								char k = 0;
								rt_kprintf("Evacuate_num = %d \n",Evacuate_num);
								_3g_time = 0;
								_3g_type = 1;
								_3g_sta = 1;
								for(k=0;k<Evacuate_num;k++)
								{
									rt_kprintf(" data will send evacuation order to 3g station \n");
									rt_kprintf("  EvacuateSiteNum = %d    EvacuateType = %d \n ",coordinated_config[i].evacuate_information[k].EvacuateSiteNum,coordinated_config[i].evacuate_information[k].EvacuateType);
			
									coordinated_build_packet(coordinated_config[i].evacuate_information[k].EvacuateSiteNum+20000,APP_PROTOCOL_TYPE_CARD,APP_TOF_SENSOR_ALARM,0);
								}
							}
							if(coordinated_config[i].coordinated_action.Broadcast == 1)
							{
								uint8_t Broadcast_num = coordinated_config[i].broadcast_pack.BroadcastListLen /sizeof(struct BROADCAST_INFORMATION);
								char k = 0;
								radio_type = RADIO_EVACUATE_ALARM;
								radio_time = 0;
								radio_sta = 1;
								rt_kprintf(" data will send evacuation order to radio station \n");
								rt_kprintf("  BroadcastSiteNum = %d    BroadcastType = %d \n ",coordinated_config[i].broadcast_information[k].BroadcastSiteNum,coordinated_config[i].broadcast_information[k].BroadcastType);
								for(k=0;k<Broadcast_num;k++)
								{
									coordinated_build_packet(coordinated_config[i].broadcast_information[k].BroadcastSiteNum,APP_PROTOCOL_TYPE_RADIO,APP_RADIO_ALARM,radio_type);
								}
							}
						}
					}
					if((_3g_type == 0)&&(_3g_sta == 1))
					{
						_3g_time ++;
						if(_3g_time == 5)
						{	
							uint8_t Evacuate_num = coordinated_config[i].evacuate_pack.EvacuateListLen /sizeof(struct EVACUATE_INFORMATION);
							uint8_t k = 0; 
							for(k=0;k<Evacuate_num;k++)
							{
								coordinated_build_packet(coordinated_config[i].evacuate_information[k].EvacuateSiteNum,APP_PROTOCOL_TYPE_CARD,APP_TOF_SENSOR_ALARM,0);
							}
							coordinated_build_packet(coordinated_config[i].evacuate_information[k].EvacuateSiteNum,APP_PROTOCOL_TYPE_CARD,APP_TOF_SENSOR_CANCEL_ALARM,0);
							rt_kprintf(" APP_TOF_SENSOR_CANCEL_ALARM coming\n ");
							_3g_time = 0;
							_3g_sta = 0;
						}				
					} 
					if((radio_type == RADIO_EVACUATE_CANCLE_ALARM)&&(radio_sta == 1))
					{
						radio_time ++;
						if(radio_time == 5)
						{	
							uint8_t Broadcast_num = coordinated_config[i].broadcast_pack.BroadcastListLen /sizeof(struct BROADCAST_INFORMATION);
							uint8_t k = 0; 
							for(k=0;k<Broadcast_num;k++)
							{
								coordinated_build_packet(coordinated_config[i].broadcast_information[k].BroadcastSiteNum,APP_PROTOCOL_TYPE_RADIO,APP_RADIO_CANCEL_ALARM,RADIO_EVACUATE_CANCLE_ALARM);
							}
							radio_time = 0;
							radio_sta = 0;
						}				
					} 
					
					
				}
			}

		}

		
	}
}


void Dispic(uint16_t X, uint16_t Y, uint16_t Height, uint16_t Width, uint16_t *ptr)
{
	const uint16_t *p;
	uint16_t color;
	uint16_t x, y;		

	p = ptr;
	for (y = 0; y < Height; y++)
	{
		for (x = 0; x < Width; x++)
		{
			color = *p++;	   
			Draw_point(x + X, y + Y, color);
		}
	}
}


/* 保存当前显示窗口的位置和大小，这几个变量由 RA8875_SetDispWin() 进行设置 */
static __IO uint16_t s_WinX = 0;
static __IO uint16_t s_WinY = 0;
static __IO uint16_t s_WinHeight = 480;
static __IO uint16_t s_WinWidth = 800;


void LCD_SetDispWin(uint16_t X, uint16_t Y, uint16_t Height, uint16_t Width)
{

	uint16_t Temp;

	Write_Dir(0x30, X);
	Write_Dir(0x31, X >> 8);

	Write_Dir(0x32, Y);
	Write_Dir(0x33, Y >> 8);

	Temp = Width + X - 1;
	Write_Dir(0x34, Temp);
	Write_Dir(0x35, Temp >> 8);

	Temp = Height + Y - 1;
	Write_Dir(0x36, Temp);
	Write_Dir(0x37, Temp >> 8);

	Write_Dir(0x40,0x40);

	MemoryWrite_Position(X, Y);

}


void LCD_QuitWinMode()
{
	LCD_SetDispWin(0, 0,480, 800);
}





extern const unsigned char gImage_pic[];

void LCD_DrawPic(uint16_t X, uint16_t Y, uint16_t Height, uint16_t Width, uint16_t *ptr)
{
	uint32_t index = 0;
	uint16_t *p;
    	uint16_t color;
	LCD_SetDispWin(X, Y, Height, Width);
    	LCD_CmdWrite(0x02); 		  
    	p =ptr;
	for (index = 0; index < Height * Width; index++)
	{
        color = *p;
        LCD_DataWrite16(color);
        p++;     
          
	}
    
	NextStep();NextStep();NextStep();NextStep();NextStep();NextStep();
	LCD_QuitWinMode();

}








void LCD_DrawPic1(uint16_t X, uint16_t Y, uint16_t Height, uint16_t Width, uint16_t *ptr)
{
	uint16_t x,y;
	uint16_t *p;
	uint16_t color;
	
	p = ptr;
	for (y = Y; y < Y+Height ; y++)
	{
	        for (x = X; x < X+Width ; x++)
	        {
	            color = *p;
	            Draw_point(x,y,color);
	            p++;   
	        }
	}
}



void LCD_FUNCTION_STATE_SET(uint8_t x_order,uint8_t y_order,uint8_t state)
{
	unsigned char str;
	if( y_order == 0 )
	{		
		NextStep();NextStep();NextStep();NextStep();NextStep();
		NextStep();NextStep();NextStep();NextStep();NextStep();
		Active_Window(8+x_order*98+2,8+(x_order+1)*98-2,34+2,84-2);//Set the work window size
//		NextStep();NextStep();NextStep();NextStep();
		if(state == 1)
			LCD_SetBackgroundColor(color_green);//Set the background color
		else 
			LCD_SetBackgroundColor(color_red);//Set the background color

		Write_Dir(0X8E,0X40);//Set the screen clearing properties window (work window)
		Write_Dir(0X8E,0XC0);//Began to clear the screen
			
		NextStep();
			
		FontWrite_Position(28+x_order*98+2,55);//Text written to the position	
	        NextStep();NextStep();NextStep();
		Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
		NextStep();NextStep();NextStep();
				
		if(x_order==0)
			String("  监测 ");
		else if(x_order==1)
			String("  广播");
		else if(x_order==2)
			String("  定位");
		else if(x_order==3)
			String("  WIFI");
		else 
			String("未添加");
		NextStep();
		        		
	}
	else if( y_order == 1 )
	{
		NextStep();NextStep();NextStep();NextStep();NextStep();	
		Active_Window(8+x_order*98+2,8+(x_order+1)*98-2,84+2,84+60-2);//Set the work window size

		NextStep();NextStep();

		if(state == 1)
			LCD_SetBackgroundColor(color_green);//Set the background color
		else 
			LCD_SetBackgroundColor(color_red);//Set the background color

		Write_Dir(0X8E,0X40);//Set the screen clearing properties window (work window)
		Write_Dir(0X8E,0XC0);//Began to clear the screen
			
		NextStep();NextStep();NextStep();NextStep();NextStep();

		FontWrite_Position(28+x_order*98+2,105);//Text written to the position
		NextStep();NextStep();NextStep();
		Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
		NextStep();NextStep();NextStep();
		String(" 电口");
		int2str(x_order+1,&str);
		String((uint8_t *)&str);
		NextStep();NextStep();
	}
	else if( y_order == 2 )
	{	
		NextStep();NextStep();NextStep();NextStep();NextStep();
		
		if(x_order!=0)
			Active_Window(10+x_order*65+2,10+(x_order+1)*65-2,144+2,144+60-2);//Set the work window size
		else
			Active_Window(10,10+(x_order+1)*65-2,144+2,144+60-2);//Set the work window size

		NextStep();NextStep();
		
		if(state == 1)
			LCD_SetBackgroundColor(color_green);//Set the background color
		else 
			LCD_SetBackgroundColor(color_red);//Set the background color
		
		Write_Dir(0X8E,0X40);//Set the screen clearing properties window (work window)
		Write_Dir(0X8E,0XC0);//Began to clear the screen
			
		NextStep();NextStep();NextStep();NextStep();NextStep();
		
		if(x_order>=8)
			FontWrite_Position(10+4+x_order*65+2,105+60);//Text written to the position
		else
			FontWrite_Position(10+4+x_order*65+2,105+50);//Text written to the position
		NextStep();NextStep();NextStep();
		Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
		NextStep();NextStep();NextStep();
		
		String(list3_word[x_order]);
		NextStep();NextStep();
		NextStep();NextStep();
	
	}
}



void station_status_monitoring(void )
{
	static rt_uint8_t time = 0;
	static rt_uint8_t evacuate_sta = 0 ;
	static rt_uint8_t broadcast_sta = 0 ;
	static rt_uint8_t minitor_sta = 0 ;

	time ++;

	if((minitor_state == 1) && (minitor_sta < 40))
	{
		minitor_sta ++ ;
		if(minitor_sta % 10 == 0)
			LCD_FUNCTION_STATE_SET(minitor_x,minitor_y,minitor_state);
		
	}
	if((broadcast_state == 1) && (broadcast_sta <40))
	{
		broadcast_sta ++ ;
		if(broadcast_sta % 10 == 0)
			LCD_FUNCTION_STATE_SET(radio_x,radio_y,broadcast_state);
		
	}
	if((evacuate_state == 1) && (evacuate_sta <40))
	{
		evacuate_sta ++ ;
		if(evacuate_sta % 10 == 0)
			LCD_FUNCTION_STATE_SET(locate_x,locate_y,evacuate_state);
		
	}
	
	if(time == 70)
	{
		time = 0;
		if((evacuate_state == 0) && ( evacuate_sta == 40))
		{			
			LCD_FUNCTION_STATE_SET(locate_x,locate_y,evacuate_state);
			evacuate_sta = 0;
		}
		if((broadcast_state == 0) && (broadcast_sta == 40 ))
		{
			LCD_FUNCTION_STATE_SET(radio_x,radio_y,broadcast_state);
			broadcast_sta = 0;
		}
		if((minitor_state == 0) && (minitor_sta == 40 ))
		{
			LCD_FUNCTION_STATE_SET(minitor_x,minitor_y,minitor_state);
			minitor_sta = 0;
		}
		
		evacuate_state = 0;
		broadcast_state = 0;
		minitor_state = 0 ;
		
	}
    	return;
}



//full display test
void Test(uint16_t count)
{	
	Write_Dir(0X01,0X80);//display on
	LCD_SetBackgroundColor(color_white);//Background color setting
	Write_Dir(0X8E,0X80);//Began to clear the screen (display window)
	NextStep();
	NextStep();NextStep();NextStep();NextStep();NextStep();

	//LCD_SetFontColor(color_black);//Set the foreground color
	LCD_SetBackgroundColor(color_white);//Set the background color		
	Active_Window(0,799,0,479);;//Set the work window size
	//Active_Window(0,500,0,200);;//Set the work window size
	Write_Dir(0X8E,0X80);//Start screen clearing (display window)
	Chk_Busy();
	NextStep();
	//Write_Dir(0X8E,0X80);//Start screen clearing (display window)
	LCD_SetFontColor(color_black);//Set the foreground color
	Write_Dir(0x21,0x20);//Select the external character
	Write_Dir(0x06,0x03);//Set the frequency
	Write_Dir(0x2E,0x01);//Font Write Type Setting Register Set up 16 x16 character mode     spacing   0 
	Write_Dir(0x2F,0x81);//Serial Font ROM Setting GT23L32S4W
	Write_Dir(0x05,0x28);// The waveform 3   2 byte dummy Cycle) 
	Write_Dir(0x22,0x80);//Full alignment is enable.The text background color . Text don't rotation. 0x zoom		
	Write_Dir(0x29,0x05);//Font Line Distance Setting

	//Write_Dir(0x2E,0x80);//Font Write Type Setting Register Set up 32 x32 character mode     spacing   0 

	Write_Dir(0x40,0x80);//Set the character mode
	LCD_CmdWrite(0x02);//start write data
	NextStep();NextStep();NextStep();
	Draw_display_frame();
	Delay100ms(2);
	NextStep();
}



 
void data_show(int x,int y,unsigned char type_num,unsigned char *type,char addr,float data, unsigned char *unit,char state)
{
    unsigned char buf[10];
    unsigned char buff[3] = {0xff,0xff,0xff};
    if(state)
    {
        FontWrite_Position(x,y);//Text written to the position
        NextStep();NextStep();NextStep();
        Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
        NextStep();NextStep();NextStep();
        String(type);
    
        sprintf((char *)buf,"(%d)",addr);
        String(buf);
    }
    
    FontWrite_Position(x+15,y+25);//Text written to the position
    NextStep();NextStep();NextStep();
    Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
    NextStep();NextStep();NextStep();
	
    rt_memset(buf,0,10);
    if(data != ConverToByeFloat(buff))
	if((type_num == 15) || (type_num == 14)||(type_num == 18))
		sprintf((char *)buf,"      %d%s",(char)data,unit);
	else		
		sprintf((char *)buf," %0.2lf%s",data,unit);
	
    String(buf);
}


void sensor_type_addr_show(int x,int y,char type,char addr,float data,char state)
{        
    switch(type)
    {
        case 1:data_show(x,y,type,"4%甲烷",addr,data,"%vol",state);break;
        case 2:data_show(x,y,type,"40%甲烷",addr,data,"%vol",state);break;
        case 3:data_show(x,y,type,"100%甲烷",addr,data,"%vol",state);break;
        case 4:data_show(x,y,type,"管道甲烷",addr,data,"%vol",state);break;
        case 5:data_show(x,y,type,"CO",addr,data,"ppml",state);break;
        case 6:data_show(x,y,type,"绝缘装置",addr,data,"ppml",state);break;
        case 7:data_show(x,y,type,"负压",addr,data,"kpa",state);break;
        case 8:data_show(x,y,type,"水位",addr,data,"m",state);break;
        case 9:data_show(x,y,type,"温度",addr,data,"℃",state);break;
        case 10:data_show(x,y,type,"粉尘",addr,data,"mg/m3",state);break;
        case 11:data_show(x,y,type,"5%二氧",addr,data,"ppml",state);break;
        case 12:data_show(x,y,type,"25%氧气",addr,data,"ppml",state);break;
        case 13:data_show(x,y,type,"开停",addr,data," ",state);break;
        case 14:data_show(x,y,type,"风门",addr,data,"",state);break;
        case 15:data_show(x,y,type,"断电馈电",addr,data," ",state);break;
        case 16:data_show(x,y,type,"风速",addr,data,"m/s",state);break;
        case 17:data_show(x,y,type,"烟雾",addr,data,"ppml",state);break;
        case 18:data_show(x,y,type,"风简风量",addr,data," ",state);break;
        case 19:data_show(x,y,type,"硫化氢",addr,data,"ppml",state);break;
        case 20:data_show(x,y,type,"二氧化硫",addr,data,"ppml",state);break;
        case 21:data_show(x,y,type,"语音报警",addr,data,"ppml",state);break;
        case 22:data_show(x,y,type,"管道流量",addr,data,"ppml",state);break;
        case 23:data_show(x,y,type,"管道温度",addr,data,"℃",state);break;
        case 24:data_show(x,y,type,"管道压力",addr,data,"ppml",state);break;
        case 25:data_show(x,y,type,"粉尘500mg",addr,data,"ppml",state);break;
        case 26:data_show(x,y,type,"顶板应力",addr,data,"ppml",state);break;
        case 27:data_show(x,y,type,"顶板压力",addr,data,"ppml",state);break;
        case 28:data_show(x,y,type,"顶板位移",addr,data,"ppml",state);break;
        case 29:data_show(x,y,type,"测距",addr,data,"ppml",state);break;
        case 30:data_show(x,y,type,"声发射",addr,data,"ppml",state);break;
        case 31:data_show(x,y,type,"深部压力",addr,data,"ppml",state);break;
        case 32:data_show(x,y,type,"多点位移",addr,data,"ppml",state);break;
        case 33:data_show(x,y,type,"加速度",addr,data,"ppml",state);break;
        case 34:data_show(x,y,type,"超声物位仪",addr,data,"ppml",state);break;
        case 35:data_show(x,y,type,"NO2",addr,data,"ppml",state);break;
        case 36:data_show(x,y,type,"NO",addr,data,"ppml",state);break;
        case 37:data_show(x,y,type,"Cl2",addr,data,"ppml",state);break;
        case 38:data_show(x,y,type,"F2",addr,data,"ppml",state);break;
        case 39:data_show(x,y,type,"ASH3",addr,data,"ppml",state);break;
        case 40:data_show(x,y,type,"电源箱",addr,data,"ppml",state);break;
        case 41:data_show(x,y,type,"风向",addr,data,"ppml",state);break;
        case 42:data_show(x,y,type,"10%激光甲烷",addr,data,"%vol",state);break;
        case 43:data_show(x,y,type,"100%激光甲烷",addr,data,"%vol",state);break;
        case 44:data_show(x,y,type,"5A电流",addr,data,"ppml",state);break;
        case 45:data_show(x,y,type,"5V电压",addr,data,"ppml",state);break;
        case 46:data_show(x,y,type,"断电器",addr,data,"ppml",state);break;
        case 47:data_show(x,y,type,"馈电器",addr,data,"ppml",state);break;
        case 48:data_show(x,y,type,"温湿度T",addr,data,"℃",state);break;
        case 49:data_show(x,y,type,"温湿度H",addr,data,"%RH",state);break;
        case 50:data_show(x,y,type,"无线激光甲烷",addr,data,"%vol",state);break;
        case 51:data_show(x,y,type,"缺水",addr,data,"ppml",state);break;
        case 52:data_show(x,y,type,"箕斗",addr,data,"ppml",state);break;
        case 53:data_show(x,y,type,"速度",addr,data,"ppml",state);break;
    }
}    


uint8_t Is_coordinated_condition(uint8_t port,uint8_t addr,uint8_t symbol,uint32_t data)
{
	uint8_t i;
	float sensor_value_set =  int_to_float(data);
	uint8_t sensor_condition = RT_FALSE;
	rt_kprintf(" port = %d   addr = %d \n ",port,addr);
	for(i=0;i<3;i++)
	{
		if(addr == sensor_data_save[port-1][i].addr)
		{
			float sensor_value_read = sensor_data_save[port-1][i].data_now;

			switch(symbol)
			{
				case 0:
				{
					if(sensor_value_read > sensor_value_set)
					{
						rt_kprintf(" symbol == 1 true\n  ");
						sensor_condition = RT_TRUE;
					}
						
					else 
					{
						sensor_condition = RT_FALSE;
						rt_kprintf(" symbol == 1 false\n  ");
					}
					break;
				}
				case 1:
				{
					if(sensor_value_read < sensor_value_set)
						sensor_condition = RT_TRUE;
					else 
						sensor_condition = RT_FALSE;
					break;
				}
				case 2:
				{
					if(sensor_value_read >= sensor_value_set)
						sensor_condition = RT_TRUE;
					else 
						sensor_condition = RT_FALSE;
					break;
				}
				case 3:
				{
					if(sensor_value_read <= sensor_value_set)
						sensor_condition = RT_TRUE;
					else 
						sensor_condition = RT_FALSE;
					break;
				}
				case 4:
				{	
					if(sensor_value_read == sensor_value_set)
					{

						sensor_condition = RT_TRUE;
						rt_kprintf(" symbol == 4 true\n  ");
					}	
					else 
					{
						sensor_condition = RT_FALSE;
						rt_kprintf(" symbol == 4 false\n  ");
					}
						
					break;
				}
				case 5:
				{
					if(sensor_value_read != sensor_value_set)
						sensor_condition = RT_TRUE;
					else 
						sensor_condition = RT_FALSE;
					break;
				}
			}
			break;
		}
	}

	return sensor_condition;
}


void coordinated_build_packet(uint16_t station_id,uint8_t protocoltype ,uint8_t msgtype,uint8_t voice_type)
{
	rt_uint8_t send_buf[MSG_COM_PKT_SIZE];
    	struct nwkhdr *pNwkHdr = (struct nwkhdr *)send_buf;
    	APP_HDR_T *pstAppHdr = (APP_HDR_T *)(pNwkHdr + 1);

	pNwkHdr->type = NWK_DATA;
	pNwkHdr->ttl = 1;
	pNwkHdr->src = sys_option.u32BsId;
	pNwkHdr->dst = station_id;
	
	pstAppHdr->protocoltype = protocoltype;
	pstAppHdr->msgtype = msgtype;

	if(protocoltype == APP_PROTOCOL_TYPE_CARD)
	{	
		pNwkHdr->len = sizeof(APP_HDR_T);
		pstAppHdr->len = 0;
		bsmac_send_packet(send_buf, sizeof(struct nwkhdr) + pNwkHdr->len, evacuate_port);
	}
	else if(protocoltype == APP_PROTOCOL_TYPE_RADIO)
	{
		pNwkHdr->len = sizeof(APP_HDR_T)+sizeof(voice_type);
		pstAppHdr->len = sizeof(voice_type);
		rt_memcpy(send_buf+sizeof(struct nwkhdr)+sizeof(APP_HDR_T),&voice_type,sizeof(voice_type));
		rt_kprintf(" station_id = %d  broadcast_id = %d\n  ",station_id,broadcast_id);
		if(station_id != broadcast_id)
		{
			rt_kprintf(" radio data send to 3g station\n");
			bsmac_send_packet(send_buf, sizeof(struct nwkhdr) + pNwkHdr->len, 1);
		}	
		else
		{
			rt_kprintf(" radio data send to radio station\n");
			bsmac_send_packet(send_buf, sizeof(struct nwkhdr) + pNwkHdr->len, 2);
		}
			
	}
	
}




void save_coordinateddata_to_flash(uint8_t *pdata,uint32_t num,uint16_t len)
{
	int offset = 0;
	uint8_t buff[1024];   
	unsigned int save_num = num;
	rt_kprintf(" save flash num = %d \n",save_num);
	rt_memcpy(buff,&save_option,sizeof(CFG_OPTION_T));
	offset += sizeof(CFG_OPTION_T);
	
	rt_memcpy(buff+offset,&save_num,sizeof(save_num));
	offset += sizeof(num);
	rt_memcpy(buff + offset ,&coordinated_config,len);
	offset += len;
	if(flash_save(OPTION_FLASH_ADDR,buff,offset))
		rt_kprintf("flash save OK--------------------\n");
	else 
		rt_kprintf("flash save ERROR--------------------\n");
}


void flashdata_build_packet(uint8_t * pbuf, uint8_t *pdata, uint8_t type,uint8_t len)
{
	struct nwkhdr *pNwkHdr = (struct nwkhdr *)pbuf;
        APP_HDR_T *pstAppHdr = (APP_HDR_T *)(pNwkHdr + 1);
	rt_err_t err = 0;
	
	pNwkHdr->type = NWK_DATA ;
	pNwkHdr->ttl = 1;
	pNwkHdr->src = sys_option.u32BsId;
	pNwkHdr->dst = 0xff;
	pNwkHdr->len = sizeof(APP_HDR_T)+len;

	pstAppHdr->protocoltype = APP_PROTOCOL_TYPE_SENSOR_STATION ;
	pstAppHdr->msgtype = type;
	pstAppHdr->len = len;

	rt_memcpy(pstAppHdr+1, pdata, len);

	err = rt_mq_send(&bsmac_mq,(void*)pbuf,pNwkHdr->len+sizeof(struct nwkhdr) );
				
	if (err != RT_EOK)
	{
		if(type == COMM_COORDINATED_EVACUATE)
			rt_kprintf(" Evacuate packet send to bsmac_mq err\n ");
		else if(type == COMM_COORDINATED_BROADCAST)
			rt_kprintf(" Broadcast packet send to bsmac_mq err\n ");
	}
	else 
	{
		if(type == COMM_COORDINATED_EVACUATE)
			rt_kprintf(" Evacuate packet send to bsmac_mq ok\n ");
		else if(type == COMM_COORDINATED_BROADCAST)
			rt_kprintf(" Broadcast packet send to bsmac_mq ok\n ");
	}
}


void flash_data_config(uint8_t *pdata)
{
	int offset = 0;
	uint16_t i = 0;
	uint32_t num = 0;
	
	save_option = *(CFG_OPTION_T *)pdata; 
	offset += sizeof(CFG_OPTION_T);
	rt_kprintf(" Bsid = %d   Bsip = %d \n ",save_option.u32BsId,save_option.u32BsIp);

	num = *(uint32_t *)(pdata+offset);
	offset += sizeof(num);
	coordinated_num = (uint16_t)num;
	

	if((coordinated_num>0)&&(coordinated_num<=20))
	{
		for(i=0;i<coordinated_num;i++)
		{
			rt_memcpy(&coordinated_config[i],pdata+offset,sizeof(struct COORDINATED_CONFIG));
			offset += sizeof(struct COORDINATED_CONFIG);
			Delay1ms(1);
		}
	}
	else
		coordinated_num = 0;
		
	
	rt_kprintf(" coordinated_num = %d \n",coordinated_num);
				
}

