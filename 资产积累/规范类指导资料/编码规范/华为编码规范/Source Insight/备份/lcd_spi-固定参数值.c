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
/*
sbit  	MCU_RST=P3^4;
sbit  	SCLK=P1^1;
sbit  	SDI=P1^0;
sbit  	SDO=P1^2;
sbit  	SCS=P1^3;

*/
#define write_data_addr  0x0c  //slave addresses with write data
#define read_data_addr  0x0d  //slave addresses with write data
#define write_cmd_addr  0x0e  //slave addresses with write command
#define read_cmd_addr  0x0f  //slave addresses with read status


#define uchar      unsigned char
#define uint       unsigned int
#define ulong      unsigned long

unsigned int X1,Y1,X2,Y2,X3,Y3,X4,Y4;
uchar taby[4];
uchar tabx[4];
uint x[6],y[6],xmin,ymin,xmax,ymax;

uchar  pic1[];
unsigned char   pic[1];

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

//int color_list[6]={color_blue,color_yellow,color_green,color_brown,color_1,color_cyan};
int color_list[6]={color_green,color_green,color_green,color_green,color_green,color_green};

unsigned char *monitor_list[]={"甲烷传感器   0.1%vol","CO传感器     10ppml","氧气传感器    15%vol",
                                "CO传感器    10ppml","差压传感器   1.2kPa","温度传感器   0.5%vol",
                                "氧气传感器   15%vol","甲烷传感器   0.5%vol","温度传感器   0.5%vol",
	
	                              "甲烷传感器   0.1%vol","氧气传感器    15%vol","CO传感器      10ppml",
                                "CO传感器     10ppml","差压传感器   1.2kPa","风速传感器   1.3m/s",
                                "甲烷传感器   0.4%vol","氧气传感器   15%vol","温度传感器   0.5%vol",};

void Delay1ms(uint i)
{uchar j;
	while(i--)
  	for(j=0;j<125;j++);
}


void Delay10ms(uint i)
{	while(i--)
	Delay1ms(10);
}

void Delay100ms(uint i)
{	while(i--)
	Delay1ms(100);
}

void NextStep(void)
{ 
	int j=0;
 for(j=0;j<50000;j++)
 	__NOP();
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
/*
void SPI_Init(void)
{
	//SCLK = 1;	
	//SDI = 1;
	//SDO = 1;	
	
	select_port(DISABLE);//SCS = 1;
}*/
unsigned char SPI_Write(unsigned char dat)
{
	while (SPI_GetFlagStatus(SPI3, SPI_FLAG_TXE) == RESET);
	SPI_SendData(SPI3,dat);
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_ReceiveData(SPI3); //????SPIx???????	
}

unsigned char SPI_Read(unsigned char dat)
{
	while (SPI_GetFlagStatus(SPI3, SPI_FLAG_TXE) == RESET);
	SPI_SendData(SPI3,dat);
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_ReceiveData(SPI3); //????SPIx???????	
}

/*
/////////////4W_SPI_Write()
void SPI_Write(unsigned char dat)
{
	unsigned char t = 8;
	do
	{
		SDI = (bit)(dat & 0x80);
		dat <<= 1;
		SCLK = 0;	
		//SPI_Delay();
		SCLK = 1;
		//SPI_Delay();
	
	} while ( --t != 0 );
	//SCLK = 1;
	//SDI = 1;
}

//////////////4W_SPI_Read()
unsigned char SPI_Read()
{
   unsigned char dat;
   unsigned char t = 8;
	SDO = 1;
	do
	{
		SCLK = 0;
        //SPI_Delay();
		dat <<= 1;
		if ( SDO ) dat++;
		SCLK = 1;	
        //SPI_Delay();
	} while ( --t != 0 );
	return dat;
}
*/
//////////////SPI Write command
void LCD_CmdWrite(uchar cmd)
{	
	//SCLK = 1;	
	//SDI = 1;	
	select_cs_lcd(ENABLE);//SCS = 0;
	//SPI_Delay();
	SPI_Write(0x80); 
	SPI_Write(cmd);
	select_cs_lcd(DISABLE);//SCS = 1;
	//SPI_Delay();
}

//////////////SPI Write data or  parameter
void LCD_DataWrite(uchar Data)
{
	//SCLK = 1;	
	//SDI = 1;		
	select_cs_lcd(ENABLE);//SCS = 0;
	SPI_Write(0x00); 
	SPI_Write(Data);
	//SPI_Delay();
	select_cs_lcd(DISABLE);//SCS = 1;
}

///////////////Read data or  parameter
uchar LCD_DataRead(void)
{
	uchar Data;	
	//SCLK = 1;	
	//SDO = 1;	
	select_cs_lcd(ENABLE);//SCS = 0;
	//SPI_Write(0x40);  
	Data = SPI_Read(0x40);
	//SPI_Delay();
	select_cs_lcd(DISABLE);//SCS = 1;
	return Data;
}  

////////////////Write command and parameter
void Write_Dir(uchar Cmd,uchar Data)
{
  LCD_CmdWrite(Cmd);
  LCD_DataWrite(Data);
}

///////////SPI Read  status
uchar LCD_StatusRead(void)
{
	uchar Data;	
	//SCLK = 1;	
	//SDO = 1;	
	select_cs_lcd(ENABLE);//SCS = 0;
	//SPI_Delay();
	//SPI_Write(0xc0); 	
	Data = SPI_Read(0xc0);
	//SPI_Delay();
	select_cs_lcd(DISABLE);//SCS = 1;
	//rt_kprintf("LCD_DataRead = %d---",Data);
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
	uchar temp; 	
	//do
	//{
		temp=LCD_StatusRead();
	//}while((temp&0x80)==0x80);
	Delay10ms(1);	
	NextStep();
}
///////////////check bte busy
void Chk_BTE_Busy(void)
{
	uchar temp; 	
	//do
	//{
	temp=LCD_StatusRead();
//	}while((temp&0x40)==0x40);		   
}
///////////////check dma busy
void Chk_DMA_Busy(void)
{
	uchar temp; 	
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
void Active_Window(uint XL,uint XR ,uint YT ,uint YB)
{
	uchar temp;
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
	
	
	Active_Window(0,799,0,479);

	LCD_CmdWrite(0x8a);//PWM setting
	LCD_DataWrite(0x80);
	LCD_CmdWrite(0x8a);//PWM setting
	LCD_DataWrite(0x81);//open PWM
	LCD_CmdWrite(0x8b);//Backlight brightness setting
	LCD_DataWrite(0xff);//Brightness parameter 0xff-0x00
}


///////////////Background color settings
void Text_Background_Color1(uint b_color)
{
	
	LCD_CmdWrite(0x60);//BGCR0
	LCD_DataWrite((uchar)(b_color>>11));
	
	LCD_CmdWrite(0x61);//BGCR0
	LCD_DataWrite((uchar)(b_color>>5));
	
	LCD_CmdWrite(0x62);//BGCR0
	LCD_DataWrite((uchar)(b_color));
} 

///////////////Background color settings
/*void Text_Background_Color(uchar setR, setG, setB)
{
    LCD_CmdWrite(0x60);//BGCR0
	LCD_DataWrite(setR);
   
    LCD_CmdWrite(0x61);//BGCR0
	LCD_DataWrite(setG);

    LCD_CmdWrite(0x62);//BGCR0
	LCD_DataWrite(setB);
} 
*/
////////////////Foreground color settings
void Text_Foreground_Color1(uint b_color)
{
	
	LCD_CmdWrite(0x63);//BGCR0
	LCD_DataWrite((uchar)(b_color>>11));
	
	LCD_CmdWrite(0x64);//BGCR0
	LCD_DataWrite((uchar)(b_color>>5));
	
	LCD_CmdWrite(0x65);//BGCR0
	LCD_DataWrite((uchar)(b_color));
} 


////////////////Foreground color settings
void Text_Foreground_Color(uchar setR,uchar setG,uchar setB)
{	    
    LCD_CmdWrite(0x63);//BGCR0
	LCD_DataWrite(setR);
   
    LCD_CmdWrite(0x64);//BGCR0
	LCD_DataWrite(setG);

    LCD_CmdWrite(0x65);//BGCR0·
	LCD_DataWrite(setB);
}
//////////////////BTE area size settings
void BTE_Size(uint width,uint height)
{
    uchar temp;
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
void BTE_Source(uint SX,uint DX ,uint SY ,uint DY)
{
	uchar temp,temp1;
    
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
void MemoryWrite_Position(uint X,uint Y)
{
	uchar temp;

	temp=X;   
    LCD_CmdWrite(0x46);
	LCD_DataWrite(temp);
	temp=X>>8;   
    LCD_CmdWrite(0x47);	   
	LCD_DataWrite(temp);

	temp=Y;   
    LCD_CmdWrite(0x48);
	LCD_DataWrite(temp);
	temp=Y>>8;   
    LCD_CmdWrite(0x49);	   
	LCD_DataWrite(temp);
}

////////////////Text write position
void FontWrite_Position(uint X,uint Y)
{
	uchar temp;
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
void String(uchar *str)
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

		

/////////////////Scroll window size
void Scroll_Window(uint XL,uint XR ,uint YT ,uint YB)
{
	uchar temp;    
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
void Scroll(uint X,uint Y)
{
	uchar temp;
    
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
void DMA_block_mode_size_setting(uint BWR,uint BHR,uint SPWR)
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
void DMA_Start_address_setting(ulong set_address)
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
void  Draw_Circle(uint X,uint Y,uint R)
{
	uchar temp;
    
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
void  Draw_Ellipse(uint X,uint Y,uint R1,uint R2)
{
	uchar temp;    
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
void Draw_Line(uint XS,uint XE ,uint YS,uint YE)
{	
    uchar temp;    
	temp=XS;   
    LCD_CmdWrite(0x91);
	LCD_DataWrite(temp);
	temp=XS>>8;   
    LCD_CmdWrite(0x92);	   
	LCD_DataWrite(temp);

	temp=XE;
    LCD_CmdWrite(0x95);
	LCD_DataWrite(temp);
	temp=XE>>8;   
    LCD_CmdWrite(0x96);	   
	LCD_DataWrite(temp);

	temp=YS;   
    LCD_CmdWrite(0x93);
	LCD_DataWrite(temp);
	temp=YS>>8;   
    LCD_CmdWrite(0x94);	   
	LCD_DataWrite(temp);

	temp=YE;   
    LCD_CmdWrite(0x97);
	LCD_DataWrite(temp);
	temp=YE>>8;   
    LCD_CmdWrite(0x98);	   
	LCD_DataWrite(temp);
}

////////////draw a triangle of three point 
void Draw_Triangle(uint X3,uint Y3)
{
    uchar temp;    
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

void Draw_line_window(uint XS,uint XE ,uint YS,uint YE)
{
	Draw_Line(XS,XE,YS,YS);
	Text_Foreground_Color1(color_black);//Color Settings
	Write_Dir(0X90,0X10);//Setting parameters
	Write_Dir(0X90,0X90);
	NextStep();NextStep();NextStep();NextStep();
	Draw_Line(XS,XE,YE,YE);
	Text_Foreground_Color1(color_black);//Color Settings
	Write_Dir(0X90,0X10);//Setting parameters
	Write_Dir(0X90,0X90);
	NextStep();NextStep();NextStep();NextStep();
	Draw_Line(XS,XS,YS,YE);
	Text_Foreground_Color1(color_black);//Color Settings
	Write_Dir(0X90,0X10);//Setting parameters
	Write_Dir(0X90,0X90);
	NextStep();NextStep();NextStep();NextStep();
	Draw_Line(XE,XE,YS,YE);
	Text_Foreground_Color1(color_black);//Color Settings
	Write_Dir(0X90,0X10);//Setting parameters
	Write_Dir(0X90,0X90);
	NextStep();
}
/////////////Touch the interrupt judgment
uchar Touch_Status(void)
{	
    uchar temp;
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
uchar Chk_INT(void)
{
	uchar temp; 	
	temp=LCD_StatusRead();
    if ((temp&0x20)==0x20)
	return 1;
	else 
	return 0;	   
}

uchar Chk_INT2(void)
{
	uchar temp; 	
    LCD_CmdWrite(0x74);//INTC
	temp =LCD_DataRead();
    if ((temp&0x80)==0x80)
	return 1;
	else 
	return 0;	   
}

/////////Read TP the X coordinate 
uchar ADC_X(void)
{
    uchar temp;
	LCD_CmdWrite(0x72);//TPXH	 X_coordinate high byte
	//Chk_Busy();
	temp=LCD_DataRead();
	return temp;
}

/////////Read TP the Y coordinate 
uchar ADC_Y(void)
{
    uchar temp;
	LCD_CmdWrite(0x73);//TPYH	  Y_coordinate high byte
    //Chk_Busy();
	temp=LCD_DataRead();
	return temp;
}

////////////Read TP the XY coordinates, the coordinates (high)
uchar ADC_XY(void)
{	
    uchar temp;
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
void Displaypicture(uchar picnum)
{  
   uchar picnumtemp;
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
void CutPictrue(uchar picnum,uint x1,uint y1,uint x2,uint y2,unsigned long x,unsigned long y)
{unsigned long cutaddress;uchar picnumtemp;
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

/*	Draw_Line(137,137,33,133);
	Text_Foreground_Color1(color_black);//Color Settings
	Write_Dir(0X90,0X10);//Setting parameters
	Write_Dir(0X90,0X90);//Start drawing
	
	NextStep();NextStep();
	Draw_Line(268,268,33,133);
	Text_Foreground_Color1(color_black);//Color Settings
	Write_Dir(0X90,0X10);//Setting parameters
	Write_Dir(0X90,0X90);//Start drawing
	
	NextStep();NextStep();
	Draw_Line(400,400,33,133);
	Text_Foreground_Color1(color_black);//Color Settings
	Write_Dir(0X90,0X10);//Setting parameters
	Write_Dir(0X90,0X90);//Start drawing
	
	NextStep();NextStep();
	Draw_Line(531,531,33,133);
	Text_Foreground_Color1(color_black);//Color Settings
	Write_Dir(0X90,0X10);//Setting parameters
	Write_Dir(0X90,0X90);//Start drawing
	
	NextStep();NextStep();
	Draw_Line(662,662,33,133);
	Text_Foreground_Color1(color_black);//Color Settings
	Write_Dir(0X90,0X10);//Setting parameters
	Write_Dir(0X90,0X90);//Start drawing
	*/
void int2str(int n, char *str)
{
		char buf[10] = "";
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
	 {           i--;
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

void draw_tcp_com(unsigned int xs,unsigned int ys)
{
	int i=0,j=19;
	int hig=0,low=0;
	unsigned char str;
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
		if(i<4)
			Text_Background_Color1(color_white);//Set the background color
		else if(i>=4&&i<7)
			Text_Background_Color1(color_green);//Set the background color
		else if(i>=7)
			Text_Background_Color1(color_white);//Set the background color
		//Text_Background_Color1(color_list[(i+j)%6]);//Set the background color
		Write_Dir(0X8E,0X40);//Set the screen clearing properties window (work window)
		Write_Dir(0X8E,0XC0);//Began to clear the screen
		
		if(i>=8)
			FontWrite_Position(12+i*65+2,105+60);//Text written to the position
		else
			FontWrite_Position(12+i*65+2,105+50);//Text written to the position
		NextStep();
		Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
		NextStep();
		if(i<4)
		{
			String("百兆光     ");
			j=17+i;
			hig=j/10;low=j%10;
			int2str(hig,&str);
			String(&str);
			NextStep();NextStep();
			int2str(low,&str);
			String(&str);
		}
		else if(i<=7)
		{
			String("千兆光     ");
			j=17+i;
			hig=j/10;low=j%10;
			int2str(hig,&str);
			String(&str);
			NextStep();NextStep();
			int2str(low,&str);
			String(&str);
		}
		else
		{
			String("COM");
			j=i-7;
			int2str(j,&str);
			String(&str);
		}
		NextStep();NextStep();
	}

}

void draw_monitor(unsigned int xs,unsigned int ys)
{
		int i=0,j=0,start=0;
	char str;
	for(i=0;i<6;i++)
	{
		start=xs+4+130*i;
		
		NextStep();NextStep();NextStep();NextStep();
		NextStep();NextStep();NextStep();
		//Active_Window(start+6,start+125-6,ys+30,399);//Set the work window size
		NextStep();NextStep();

		NextStep();NextStep();NextStep();
		Text_Foreground_Color1(color_black);//Color Settings
		Draw_line_window(start+5,start+130-5,ys,430);  //外框
		NextStep();NextStep();NextStep();NextStep();
		NextStep();NextStep();NextStep();
		for(j=0;j<3;j++)
		{
			NextStep();NextStep();NextStep();NextStep();NextStep();
			Draw_Line(start+5,start+130-5,ys+30+j*60,ys+30+j*60);
			NextStep();NextStep();
			Text_Foreground_Color1(color_black);//Color Settings
			Write_Dir(0X90,0X10);//Setting parameters
			Write_Dir(0X90,0X90);//Start drawing
		}
	}
	for(i=0;i<6;i++)
	{
		start=xs+4+130*i;

		for(j=0;j<3;j++)
		{
			
			NextStep();NextStep();NextStep();NextStep();NextStep();
			NextStep();NextStep();NextStep();NextStep();NextStep();
			Active_Window(start+5+1,start+130-6,ys+30+j*60+1,ys+30+(j+1)*60-1);
			NextStep();NextStep();
			Text_Background_Color1(color_list[(i+j)%6]);//Set the background color
			Write_Dir(0X8E,0X40);//Set the screen clearing properties window (work window)
			Write_Dir(0X8E,0XC0);//Began to clear the screen
			
			NextStep();NextStep();NextStep();NextStep();NextStep();
			FontWrite_Position(start+10,ys+30+j*60+10);//Text written to the position
			NextStep();NextStep();NextStep();
			Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
			NextStep();NextStep();NextStep();
			String(monitor_list[i*3+j]);
		}
		FontWrite_Position(start+10,ys+5);//Text written to the position
		NextStep();
		Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
		NextStep();
		Text_Background_Color1(color_white);//Set the background color
		String("端口 ");
		int2str(i+1,&str);
		String(&str);
	}

}
void Draw_display_frame()
{
	int i=0,j=0;
	unsigned char str;
	int start=0, xs=0,ys=0;
	Text_Background_Color1(color_white);//Set the background color
	Draw_line_window(start_xrow,795,2,475);  //外框
	NextStep();NextStep();NextStep();
	
	//Active_Window(start_xrow+1,794,2,474);//Set the work window size
	//Text_Background_Color1(color_white);//Set the background color
	//Write_Dir(0X8E,0X40);//Set the screen clearing properties window (work window)
	//Write_Dir(0X8E,0XC0);//Began to clear the screen
	NextStep();NextStep();NextStep();NextStep();NextStep();
	NextStep();
	//Draw_Line(6,794,150,150);
	//NextStep();
	Text_Foreground_Color1(color_red);//Color Settings
	//Write_Dir(0X90,0X10);//Setting parameters
	//Write_Dir(0X90,0X90);//Start drawing
	
	NextStep();
	Write_Dir(0x2E,0x40);//16 X16
	NextStep();
	FontWrite_Position(260,8);//Text written to the position
	//String("深 圳 市 翌 日 科 技 有 限 公 司 环 境 监 控 系 统 ");
	String("深圳市翌日科技综合分站");
	/************************************前几排的横线*********************************************/
	NextStep();NextStep();NextStep();NextStep();NextStep();
	for(i=0;i<4;i++)
	{
		if(i==0)
		{
				Draw_Line(start_xrow,794,33+i*50-1,33+i*50-1);
			//else 
			//	Draw_Line(start_xrow,794,33+i*50+(i-1)*10-1,33+i*50+(i-1)*10-1);
			Text_Foreground_Color1(color_black);//Color Settings
			Write_Dir(0X90,0X10);//Setting parameters
			Write_Dir(0X90,0X90);//Start drawing
			NextStep();NextStep();NextStep();NextStep();NextStep();NextStep();
		}
		
		if(i==0 || i==1)
			Draw_Line(start_xrow,794,33+i*50+1,33+i*50+1);
		else 
			Draw_Line(start_xrow,794,33+i*50+(i-1)*10+1,33+i*50+(i-1)*10+1);
		Text_Foreground_Color1(color_black);//Color Settings
		Write_Dir(0X90,0X10);//Setting parameters
		Write_Dir(0X90,0X90);//Start drawing
		NextStep();NextStep();NextStep();NextStep();NextStep();NextStep();
	}
	/************************************一二排的竖线*********************************************/
	NextStep();NextStep();NextStep();NextStep();NextStep();
	for(i=0;i<=8;i++)
	{
		NextStep();NextStep();NextStep();NextStep();NextStep();
		Draw_Line(8+i*98-1,8+i*98-1,33,143);
		NextStep();NextStep();
		Text_Foreground_Color1(color_black);//Color Settings
		Write_Dir(0X90,0X10);//Setting parameters
		Write_Dir(0X90,0X90);//Start drawing
		
		NextStep();NextStep();NextStep();NextStep();NextStep();
		Draw_Line(8+i*98+1,8+i*98+1,33,143);
		NextStep();NextStep();
		Text_Foreground_Color1(color_black);//Color Settings
		Write_Dir(0X90,0X10);//Setting parameters
		Write_Dir(0X90,0X90);//Start drawing
	}
	/*********************************百兆光，千兆故竖线*************************************/
	for(i=0;i<=12;i++)
	{
		NextStep();NextStep();NextStep();NextStep();NextStep();
		if(i!=12)
			Draw_Line(8+i*65-1,8+i*65-1,145,205);
		else
			Draw_Line(8+i*65+3,8+i*65+3,145,205);
		NextStep();NextStep();
		Text_Foreground_Color1(color_black);//Color Settings
		Write_Dir(0X90,0X10);//Setting parameters
		Write_Dir(0X90,0X90);//Start drawing
		
		NextStep();NextStep();NextStep();NextStep();NextStep();
		if(i!=12)
			Draw_Line(8+i*65+1,8+i*65+1,145,205);
		else
			Draw_Line(8+i*65+5,8+i*65+5,145,205);
		NextStep();NextStep();
		Text_Foreground_Color1(color_black);//Color Settings
		Write_Dir(0X90,0X10);//Setting parameters
		Write_Dir(0X90,0X90);//Start drawing
		
	}
		/************************************传感器显示*******************************************/
	NextStep();NextStep();NextStep();NextStep();NextStep();
	draw_monitor(4,220);
/*********************************百兆光，千兆光* 显示***********************************************/
		NextStep();NextStep();NextStep();NextStep();NextStep();
	
	draw_tcp_com(10,144);
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
				//if(i!=0)
					Active_Window(8+i*98+2,8+(i+1)*98-2,34+50*j+2,84+50*j-2);//Set the work window size
				//else
				//	Active_Window(8,8+(i+1)*98-2,34+50*j+2,84+50*j-2);//Set the work window size
			}
			else
			{
				//if(i!=0)
					Active_Window(8+i*98+2,8+(i+1)*98-2,84+2,84+60-2);//Set the work window size
				//else
				//	Active_Window(8,8+(i+1)*98-2,84+2,84+60-2);//Set the work window size
			}
			NextStep();NextStep();
			if(j==0)
			{
				if(i>=4)
					Text_Background_Color1(color_white);//Set the background color
				else
					Text_Background_Color1(color_list[(i+j)%6]);//Set the background color
			}
			else 
				Text_Background_Color1(color_white);//Set the background color
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
				String(&str);
				NextStep();NextStep();
			}
		}
	}

		FontWrite_Position(500,450);//Text written to the position
		NextStep();
		Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
		NextStep();
		Text_Background_Color1(color_white);//Set the background color
	   String("服务器已连接      2018-03-14 15:11");	
	NextStep();NextStep();NextStep();NextStep();NextStep();NextStep();
}


//full display test
void Test(int count)
{	///display red
	int i=0;
	unsigned char result=0;
	Write_Dir(0X01,0X80);//display on
	if(count ==1||count ==0)
	{
		Text_Background_Color1(color_white);//Background color setting
		Write_Dir(0X8E,0X80);//Began to clear the screen (display window)
		NextStep();
	}

	NextStep();NextStep();NextStep();NextStep();NextStep();

	Text_Foreground_Color1(color_white);//Set the foreground color
	//Text_Background_Color1(color_black);//Set the background color		
	rt_kprintf("2---");
	Active_Window(0,899,0,479);;//Set the work window size
	//Active_Window(0,500,0,200);;//Set the work window size
	Write_Dir(0X8E,0X80);//Start screen clearing (display window)
	// Chk_Busy();
	NextStep();
	//Write_Dir(0X8E,0X80);//Start screen clearing (display window)
	Text_Foreground_Color1(color_black);//Set the foreground color
	rt_kprintf("3---");
	Write_Dir(0x21,0x20);//Select the external character
	Write_Dir(0x06,0x03);//Set the frequency
	Write_Dir(0x2E,0x01);//Font Write Type Setting Register Set up 16 x16 character mode     spacing   0 
	Write_Dir(0x2F,0x81);//Serial Font ROM Setting GT23L32S4W
	Write_Dir(0x05,0x28);// The waveform 3   2 byte dummy Cycle) 
	Write_Dir(0x22,0x80);//Full alignment is enable.The text background color . Text don't rotation. 0x zoom		
	Write_Dir(0x29,0x05);//Font Line Distance Setting
	rt_kprintf("4---");

	//Write_Dir(0x2E,0x80);//Font Write Type Setting Register Set up 32 x32 character mode     spacing   0 

	Write_Dir(0x40,0x80);//Set the character mode
	LCD_CmdWrite(0x02);//start write data
	rt_kprintf("5---");
	NextStep();NextStep();NextStep();
	
	Draw_display_frame();
	
	
	//Scroll_Window(6,999,34,474);	//Specifies scrolling activity area
	i=0; 

	Delay100ms(2);
	NextStep();
}



/////////////////////main////////////////////
void Test1(void)
{
	uint i;
//	P0=0xff;
//	P1=0xff;
//	P2=0xff;
//	P3=0xff;
    Delay100ms(5);

	LCD_Reset(); //RC Reset on board
	//LCD_Initial();
	rt_kprintf("0---");
	Write_Dir(0X01,0X80);//display on

//	while(1)
		{

			//full display test
			//Test();
			rt_kprintf("1---");

			/////External characters of the functional test
		    Text_Foreground_Color1(color_white);//Set the foreground color
			Text_Background_Color1(color_black);//Set the background color		
			rt_kprintf("2---");
			Active_Window(0,799,0,479);;//Set the work window size
			Write_Dir(0X8E,0X80);//Start screen clearing (display window)
		   // Chk_Busy();
			NextStep();
			rt_kprintf("3---");
			Write_Dir(0x21,0x20);//Select the external character
			Write_Dir(0x06,0x03);//Set the frequency
			Write_Dir(0x2E,0x80);//Font Write Type Setting Register Set up 32 x32 character mode     spacing   0 
			Write_Dir(0x2F,0x81);//Serial Font ROM Setting GT23L32S4W
			Write_Dir(0x05,0x28);// The waveform 3   2 byte dummy Cycle) 
		    Write_Dir(0x22,0x80);//Full alignment is enable.The text background color . Text don't rotation. 0x zoom		
		    Write_Dir(0x29,0x05);//Font Line Distance Setting
	rt_kprintf("4---");
			FontWrite_Position(208,45);//Text written to the position
		    Write_Dir(0x40,0x80);//Set the character mode
		    LCD_CmdWrite(0x02);//start write data
		    rt_kprintf("5---");
			NextStep();
		    String("深圳市亚斌电子有限公司");
		rt_kprintf("6---");
			Text_Foreground_Color1(color_red);//Set the foreground color
			Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
		    FontWrite_Position(100,90);//Text written to the position
			String("TEL:0755-29517345  FAX:0755-29517347");
			FontWrite_Position(100,120);//Text written to the position
			String("http：//www.yab-lcm.com");
			FontWrite_Position(100,150);//Text written to the position
			String("E-mail：yablcm@126.com");
			FontWrite_Position(100,180);//Text written to the position
			String("Adress:F2,A building,YiLai Industrial Park,ShiYan Town,ShenZhen,China.");
		    Write_Dir(0x29,0x00);//Font Line Distance Setting
		    Write_Dir(0x22,0x05);//Full alignment is disable.The text background color . Text don't rotation. 2x zoom		
			Text_Foreground_Color1(color_green);//Set the foreground color
			Write_Dir(0x2E,0x00);//Set the characters mode 16 x16 / spacing 0
			FontWrite_Position(0x00,250);//Text written to the position
			String("YBT7000A-8875，Optional Chinese / English character library,  MicroSD cord,Falsh.Font Support 2/3/4 times zoom."
		    "     Support8/16-bit 8080/6800 Series bus,Support serial 3/4wire SPI interface,I2C interface.Block Transfer Engine (BTE) Supports  with 2D，Geometry Accelerated Graphics Engine,Support DMA Direct Access FLASH。");
			Write_Dir(0x21,0x00);//Recovery of register
			Write_Dir(0x2F,0x00);//Recovery of register

			rt_kprintf("7---");

			////////////RA8875 internal input character test
		    Text_Foreground_Color1(color_yellow);//Set the foreground color
		    Write_Dir(0x2E,0x01);//Set the characters mode 16 x16 / spacing 1
		    Write_Dir(0x40,0x80);//Set the character mode
		    Write_Dir(0x21,0x10);//Select the internal CGROM  ISO/IEC 8859-1.
		    FontWrite_Position(80,5);//Text written to the position
	

		    String("ShenZhen YABIN Electronics .;LTD");
	
			Delay100ms(2);
			NextStep();
			rt_kprintf("8---");


			//////////The cursor function test
		    Write_Dir(0x40,0xE0);//Set the text mode cursor
		    Write_Dir(0x41,0x00);//Close the graphics cursor
		    Write_Dir(0x44,0x1f);//The cursor flashing cycle
		    Write_Dir(0x4e,0x1f);//The cursor size
		    Write_Dir(0x4f,0x1f);//The cursor size
			Delay100ms(10);
			NextStep();
		
		    Write_Dir(0x21,0x00);//Recovery of register
		    Write_Dir(0x40,0x00);//Recovery of register

  
	/*		////////PWM backlight control test  
   			Write_Dir(0x8b,0x0f);//Backlight brightness adjustment
			Delay100ms(3);
			NextStep();
   			Write_Dir(0x8b,0x3f);//Backlight brightness adjustment
			Delay100ms(3);
			NextStep();
   			Write_Dir(0x8b,0xff);//Backlight brightness adjustment
			Delay100ms(3);
			NextStep();
	  */

			//clear screen test:   part of the window 
	rt_kprintf("9---");

		   	Write_Dir(0X8E,0X80);//Began to clear the screen (display window)
		   	Chk_Busy();		
		   	Active_Window(40,300,100,300);//Set the work window size
		   	Text_Background_Color1(color_green);//Set the background color
		   	Write_Dir(0X8E,0X40);//Set the screen clearing properties window (work window)
		   	Write_Dir(0X8E,0XC0);//Began to clear the screen
		   	Chk_Busy();
		     
		   	Active_Window(300,799,200,479);//Set the work window size
		   	Text_Background_Color1(color_cyan);//Set the background color
		   	Write_Dir(0X8E,0X40);//Set the screen clearing properties window (work window)
		   	Write_Dir(0X8E,0XC0);//Began to clear the screen
		   	Chk_Busy();

			/////////Memory write test
		   	Write_Dir(0x40,0x00);
			Active_Window(0,111,0,139);//Set the work window size	
		   	MemoryWrite_Position(0,0);//Memory write position
		   	LCD_CmdWrite(0x02);//start data write
  			//112X140 dot
  			for(i=0;i<31360;i++)
   			{
			    LCD_DataWrite(pic[i]);
				Chk_Busy();
		    }
			Delay100ms(3);
			NextStep();	
	rt_kprintf("10---");
			/////// Geometric pattern drawing test
			Text_Background_Color1(color_black);//Set the background color
			Active_Window(0,799,0,479);;//Set the work window size
	 		Write_Dir(0X8E,0X40);//Set clear screen nature ( working window )
   			Write_Dir(0X8E,0XC0);//Began to clear the screen
			Chk_Busy();

			///////////Drawing curves
			Draw_Ellipse(210,120,205,105);
		    Text_Foreground_Color1(color_cyan);//Color Settings
		    Write_Dir(0XA0,0X10);//Setting parameters
		    Write_Dir(0XA0,0X90);//Start drawing
			Delay10ms(5);
		    Write_Dir(0XA0,0X91);//Start drawing
			Delay10ms(5);
		    Write_Dir(0XA0,0X92);//Start drawing
			Delay10ms(5);
		    Write_Dir(0XA0,0X93);//Start drawing
			Delay10ms(5);

			////////////drawing oval
			Draw_Ellipse(210,120,200,100);
			Text_Foreground_Color1(color_red);//Color Settings
			Write_Dir(0XA0,0X00);//Setting parameters
		    Write_Dir(0XA0,0X80);//Start drawing
			Delay10ms(5);
			Write_Dir(0XA0,0X40);//Set whether filling
		    Write_Dir(0XA0,0XC0);//Start drawing
			Delay10ms(5);
			/////////////drawing circle
			Draw_Circle(600,110,100);
			Text_Foreground_Color1(color_green);//Color Settings
			Write_Dir(0X90,0X00);//Setting parameters
		    Write_Dir(0X90,0X40);//Start drawing
			Delay10ms(10);		
			Write_Dir(0X90,0X20);//Setting parameters
		    Write_Dir(0X90,0X60);//Start drawing
			Delay10ms(10);
		 	/////////////drawing rectangle
		    Draw_Line(15,225,270,460);
		    Text_Foreground_Color1(color_blue);//Color Settings
			Write_Dir(0X90,0X10);//Setting parameters
		    Write_Dir(0X90,0X90);//Start drawing
		    Delay10ms(5);
			Write_Dir(0X90,0X30);//Setting parameters
		    Write_Dir(0X90,0XB0);//Start drawing
		    Delay10ms(5);
			///////////drawing triangle
			Draw_Line(300,420,460,270);
		    Draw_Triangle(540,460);//draw a triangle of three point
			Text_Foreground_Color1(color_purple);//Color Settings
		    Write_Dir(0X90,0X01);//Setting parameters
		    Write_Dir(0X90,0X81);//Start drawing
		    Delay10ms(5);
		    Write_Dir(0X90,0X21);//Setting parameters
		    Write_Dir(0X90,0XA1);//Start drawing
		    Delay10ms(5);
			///////////drawing rounded rectangle
		    Draw_Line(570,780,270,460);
		    Draw_Ellipse(0,0,20,30);//Set Radius
		    Text_Foreground_Color1(color_yellow);//Color Settings
		 	Write_Dir(0XA0,0X20);//Set whether filling
		    Write_Dir(0XA0,0XA0);//Start drawing
			Delay10ms(5);
		 	Write_Dir(0XA0,0X60);//Set whether filling
		    Write_Dir(0XA0,0XE0);//Start drawing
			Delay10ms(5);
			///////////drawing line
			Draw_Line(0,799,0,0);
		    Text_Foreground_Color1(color_red);//Color Settings
			Write_Dir(0X90,0X00);//Setting parameters
		    Write_Dir(0X90,0X80);//Start drawing
			Delay10ms(2);
			Draw_Line(799,799,0,479);//drawing line
		    Write_Dir(0X90,0X80);//Start drawing
			Delay10ms(2);
			Draw_Line(0,799,479,479);//drawing line
		    Write_Dir(0X90,0X80);//Start drawing
			Delay10ms(2);
			Draw_Line(0,0,0,479);//drawing line
		    Write_Dir(0X90,0X80);//Start drawing
			Delay10ms(2);
			NextStep();
			rt_kprintf("9---");


			////////////BTE Color Fill
			BTE_Size(25,120);
    		Write_Dir(0x51,0xcc);//Raster Settings
			for(i=0;i<32;i++)
			{
				Text_Foreground_Color(i,0,0);
				BTE_Source(0,i*25,0,0);//BTE starting position			 				  
			    Write_Dir(0x50,0x80);//BET open
			    Chk_BTE_Busy();
			
				Text_Foreground_Color(0,i*2,0);
				BTE_Source(0,i*25,0,120);//BTE starting position		  
			    Write_Dir(0x50,0x80);//BET open
			    Chk_BTE_Busy();
			
				Text_Foreground_Color(0,0,i);
				BTE_Source(0,i*25,0,240);//BTE starting position			 				  
			    Write_Dir(0x50,0x80);//BET open
			    Chk_BTE_Busy();
			
				Text_Foreground_Color(i,i*2,i);
			    BTE_Source(0,i*25,0,360);//BTE starting position			 				  
			    Write_Dir(0x50,0x80);//BET open
			    Chk_BTE_Busy();
			}
			Delay100ms(2);
			NextStep();
			rt_kprintf("11---");

			//////////BTE Color Expansion
		    Text_Background_Color1(color_purple);//Set the background color 
		    Text_Foreground_Color1(color_yellow);//Set the foreground color
		    BTE_Source(0,0,0,0);//BTE starting position
		    BTE_Size(120,100);//BTE size setting
		    Write_Dir(0x51,0x78);//Raster setting
		    Write_Dir(0x50,0x80);//BET open
			Chk_Busy();
		    LCD_CmdWrite(0x02);//start write data
  			for(i=0;i<1500;i++)
   			{
			    LCD_DataWrite(pic1[i]);
				Chk_Busy();
    		}
   			Chk_BTE_Busy();
			Delay100ms(2);
			NextStep();

			////////////BTE color expansion moves
  			BTE_Source(0,200,0,0);//BTE starting position
  			BTE_Size(120,100);//BBTE size setting
  			Text_Foreground_Color1(color_purple);//Set the foreground color (background color filter)
  			Write_Dir(0x51,0xc5);//start write data
  			Write_Dir(0x50,0x80);//BET open
  			Delay100ms(5); 
			NextStep();

			///////////Scroll function test
    		Scroll_Window(0,119,0,99);	//Specifies scrolling activity area
   			i=0; 
	 		while(i++<99){Delay10ms(10); Scroll(i,i);} //Note:  scroll offset value must be less than  scroll setting range
		    while(i-->0){Delay10ms(10); Scroll(i,i);}       
			while(i++<99){Delay10ms(10); Scroll(i,i);}
		    while(i-->0){Delay10ms(10); Scroll(i,i);}
			Delay100ms(5);
			NextStep();

			rt_kprintf("12---");


			//////////Touch function test
			//LCD_Reset();
			//LCD_Initial();
		
			Active_Window(0,799,0,479);//Set the working window size
		    Text_Foreground_Color1(color_white);//Set the foreground color
			Text_Background_Color1(color_blue);//Set the background color
			Write_Dir(0X8E,0X80);//Began to clear the screen (display window)
		    Chk_Busy();
		  	Write_Dir(0x21,0x10);//Select the internal CGROM  ISO/IEC 8859-1.
			Write_Dir(0x22,0x00);//Full alignment is disable.The text background color . Text don't rotation. 2x zoom
		  	FontWrite_Position(40,200);
		  	String("Touch to display the coordinate");

		  	Write_Dir(0xf0,0x04);//open interruption
		  	Write_Dir(0x71,0x00);//set to 4-wire touch screen
		  	Write_Dir(0x70,0xB2);//open the touch function, touch the parameter settings
			rt_kprintf("13---");

			Delay100ms(5000);
  		/*	while(1)
			{
		       Write_Dir(0xf1,0x04);//clear INT state      Must be clean TP_interrupt 
		       Delay10ms(5);
		       if(Touch_Status())
		       	{
			  		TP();
		       	}
			   else
			   	{
				  	FontWrite_Position(100,60); 
			      	LCD_CmdWrite(0x02);
				  	String("X = 0000");
				  	FontWrite_Position(100, 140); 
				  	LCD_CmdWrite(0x02);
				  	String("Y = 0000");	 
				}
	  		}*/

			NextStep();
 		}

}

 unsigned char  pic1[] = {
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X60,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X01,0XFC,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X03,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X03,0XFF,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X03,
0XEF,0XE0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X03,0XC1,
0XF8,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X03,0XC0,0X7C,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X03,0XC0,0X3E,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X01,0XE1,0XC0,0X1F,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0F,0XFF,0XE0,0X07,0X80,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0F,0XFF,0XE0,0X03,0XC0,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0E,0X7F,0XF0,0X01,0XE0,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X0E,0X03,0XF0,0X00,0XE0,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X0E,0X00,0X78,0X00,0X70,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X0E,0X00,0X1E,0X00,0X38,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X07,0X02,0X07,0X00,0X1C,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X07,0X07,0XC1,0XC0,0X1E,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X07,0X80,0XF8,0X60,0X0F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X3F,
0XE3,0X80,0X0E,0X38,0X07,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X03,0XFF,0XFF,
0XC0,0X03,0X9C,0X03,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0F,0XFF,0XFF,0XC0,
0X01,0XC6,0X03,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X7F,0XFF,0XFF,0XE0,0X00,
0X73,0X01,0XE0,0X00,0X00,0X00,0X00,0X00,0X00,0X01,0XFF,0XE0,0X1F,0XE0,0X00,0X39,
0X81,0XE0,0X00,0X00,0X00,0X00,0X00,0X00,0X07,0XFE,0X00,0X03,0XF0,0X03,0X9C,0XC0,
0XF0,0X00,0X00,0X00,0X00,0X00,0X00,0X0F,0XF0,0X00,0X00,0XF8,0X01,0XFE,0X60,0X70,
0X00,0X00,0X00,0X00,0X00,0X00,0X1F,0XC0,0X00,0X00,0X3C,0X00,0X7F,0X30,0X78,0X00,
0X00,0X00,0X00,0X00,0X00,0X7F,0X00,0X00,0X00,0X0E,0X00,0X01,0X98,0X3C,0X00,0X00,
0X00,0X00,0X00,0X00,0XFE,0X00,0X00,0X00,0X07,0X00,0X00,0X8C,0X3C,0X00,0X00,0X00,
0X00,0X00,0X01,0XF8,0X00,0X00,0X00,0X01,0XC1,0XC0,0XC6,0X1E,0X00,0X00,0X00,0X00,
0X00,0X03,0XF0,0X00,0X00,0X00,0X00,0XF0,0XFF,0XC7,0X0E,0X00,0X00,0X00,0X00,0X00,
0X07,0XE0,0X00,0X00,0X00,0X00,0X3C,0X1F,0XE3,0XCF,0X80,0X00,0X00,0X00,0X00,0X0F,
0X80,0X00,0X00,0X00,0X00,0X0F,0X80,0X61,0XFF,0XC0,0X00,0X00,0X00,0X00,0X0F,0X00,
0X00,0X00,0X00,0X00,0X01,0XF0,0X40,0X3F,0XF0,0X00,0X00,0X00,0X00,0X1E,0X00,0X00,
0X00,0X00,0X00,0X00,0X3C,0X00,0X01,0XFC,0X00,0X00,0X00,0X00,0X3C,0X00,0X00,0X00,
0X00,0X00,0X00,0X07,0X80,0X00,0X7F,0X00,0X00,0X00,0X00,0X78,0X00,0X00,0X00,0X00,
0X00,0X00,0X01,0XF8,0X00,0X1F,0X80,0X00,0X00,0X00,0X70,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X3C,0X00,0X07,0XE0,0X00,0X00,0X00,0XF0,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X01,0XF0,0X00,0X00,0X01,0XE0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X78,0X00,0X00,0X03,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X3C,0X00,0X00,0X03,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X40,0X1E,0X00,0X00,0X07,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X02,0XF0,
0X0E,0X00,0X00,0X07,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X02,0XF8,0X0F,
0X00,0X00,0X0E,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XF8,0X07,0X00,
0X00,0X0E,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X07,0X80,0X00,
0X1E,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X07,0X80,0X00,0X1C,
0X00,0X00,0X00,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X03,0X80,0X00,0X1C,0X00,
0X00,0X00,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X03,0X80,0X00,0X1C,0X00,0X00,
0X00,0X60,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X03,0X80,0X00,0X38,0X00,0X00,0X00,
0X30,0X00,0X00,0X00,0X04,0X40,0X00,0X00,0X03,0X80,0X00,0X38,0X00,0X00,0X00,0X30,
0X00,0X00,0X00,0X0E,0X40,0X00,0X00,0X03,0X80,0X00,0X38,0X00,0X00,0X00,0X10,0X00,
0X00,0X00,0X07,0X60,0X00,0X00,0X03,0X80,0X00,0X70,0X00,0X00,0X00,0X18,0X00,0X00,
0X00,0X03,0X20,0X00,0X00,0X13,0X80,0X00,0X70,0X00,0X00,0X00,0X08,0X00,0X00,0X00,
0X01,0XB0,0X00,0X00,0X1F,0X00,0X00,0X70,0X00,0X00,0X00,0X08,0X00,0X00,0X00,0X01,
0X98,0X00,0X00,0X07,0X80,0X00,0XF0,0X00,0X00,0X00,0X0C,0X00,0X00,0X00,0X00,0X8C,
0X00,0X00,0X8F,0X80,0X00,0XE0,0X00,0X00,0X00,0X0C,0X00,0X00,0X00,0X00,0X86,0X00,
0X00,0XFF,0X00,0X01,0XE1,0X80,0X00,0X00,0X0C,0X00,0X00,0X00,0X00,0XC3,0X80,0X00,
0X7E,0X00,0X01,0XC1,0X80,0X00,0X00,0X0C,0X00,0X00,0X00,0X00,0XC1,0XF0,0XFC,0XFC,
0X00,0X03,0XC1,0X00,0X00,0X00,0X08,0X00,0X00,0X00,0X00,0XC0,0X7F,0XFF,0XF0,0X00,
0X03,0X83,0X00,0X00,0X00,0X18,0X00,0X00,0X00,0X00,0X40,0X07,0XFF,0XE0,0X00,0X07,
0X83,0X00,0X00,0X00,0X18,0X00,0X00,0X00,0X00,0X44,0X07,0XE0,0X00,0X00,0X0F,0X03,
0X00,0X00,0X00,0X30,0X00,0X00,0X00,0X00,0X43,0XFF,0XC0,0X00,0X00,0X0F,0X03,0X80,
0X00,0X00,0X30,0X00,0X00,0X00,0X00,0XC0,0XFF,0X00,0X00,0X00,0X1F,0X8F,0X98,0X00,
0X00,0X60,0X00,0X00,0X00,0X00,0XC0,0X1E,0X00,0X00,0X00,0X3F,0XFF,0X98,0X00,0X00,
0X60,0X00,0X00,0X00,0X00,0X80,0X3E,0X00,0X00,0X00,0X3F,0XFF,0XD8,0X00,0X00,0XC0,
0X00,0X00,0X00,0X00,0X80,0X7C,0X00,0X00,0X00,0X0F,0XFF,0XF0,0X00,0X01,0X80,0X00,
0X03,0X00,0X01,0X80,0XF8,0X00,0X00,0X00,0X00,0X01,0XE0,0X00,0X03,0X00,0X00,0X03,
0X00,0X01,0X07,0XF0,0X00,0X00,0X00,0X00,0X03,0XC0,0X00,0X07,0X00,0X00,0X03,0X00,
0X03,0XFF,0XF0,0X00,0X00,0X00,0X00,0X03,0XC0,0X00,0X1F,0X00,0X00,0X03,0X00,0X07,
0XFF,0XF0,0X00,0X00,0X00,0X00,0X07,0X80,0X00,0X3F,0X80,0X00,0X07,0X80,0X0F,0X83,
0XF8,0X00,0X00,0X00,0X00,0X07,0X80,0X01,0XFF,0XF0,0X00,0X0F,0X80,0X0F,0XE0,0X7C,
0X00,0X00,0X00,0X00,0X03,0X80,0X0F,0XFF,0XFF,0X00,0XFF,0X80,0X00,0XFC,0X1C,0X00,
0X00,0X00,0X00,0X03,0XC0,0X03,0XFF,0XFF,0XFF,0XFF,0XC0,0X00,0X0F,0X0E,0X00,0X00,
0X00,0X00,0X03,0XE0,0X00,0X1F,0XFF,0XFF,0XFF,0XF8,0X00,0X03,0X8F,0X00,0X00,0X00,
0X00,0X01,0XFC,0X00,0X01,0XFF,0XFF,0XF0,0XFF,0XC0,0X00,0XC7,0X00,0X00,0X00,0X00,
0X00,0XFF,0XC0,0X00,0X3F,0X00,0X00,0X3F,0XF8,0X02,0X73,0X00,0X00,0X00,0X00,0X00,
0X3F,0XFC,0X00,0X0F,0X00,0X00,0X07,0XFE,0X00,0X33,0X00,0X00,0X00,0X00,0X00,0X07,
0XFF,0X80,0X27,0X80,0X00,0X00,0XFF,0X80,0X5F,0X00,0X00,0X00,0X00,0X00,0X00,0XFF,
0XF0,0X0B,0X80,0X00,0X00,0X1F,0XE0,0X1F,0X00,0X00,0X00,0X00,0X00,0X00,0X1F,0XFE,
0X01,0X80,0X00,0X00,0X03,0XF8,0X0F,0X00,0X00,0X00,0X00,0X00,0X00,0X03,0XFF,0XC3,
0X80,0X00,0X00,0X00,0XFE,0X1E,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X3F,0XF3,0X80,
0X00,0X00,0X00,0X3F,0XBE,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X07,0XFF,0X80,0X00,
0X00,0X00,0X0F,0XFE,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X01,0XFF,0X80,0X00,0X00,
0X00,0X03,0XFE,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X3F,0X00,0X00,0X00,0X00,
0X01,0XFC,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X1E,0X00,0X00,0X00,0X00,0X00,
0XF8,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X20,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00

};

unsigned char  pic[1] ={0x01} ;



