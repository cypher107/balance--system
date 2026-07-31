/*
 * @Author: Thunder_Boy
 * @Date: 2024-06-19 21:49:38
 * @LastEditTime: 2024-06-20 17:23:17
 * @Description: 此OLED代码来源于网络，非本人所写
 */
#include "OLED.h"
#include "OLEDFont.h"
#if (I2C_MODE == 2)
#include "i2c.h"
#elif (I2C_MODE == 3)
#include "main.h"
#endif

#if (I2C_MODE == 1)

uint8_t OLED_GRAM[144][8];

void IIC_Start(void)
{
	OLED_SCLK_Set() ;
	OLED_SDIN_Set();
	OLED_SDIN_Clr();
	OLED_SCLK_Clr();
}

/**********************************************
//IIC Stop
**********************************************/
void IIC_Stop(void)
{
	OLED_SCLK_Set() ;
	OLED_SDIN_Clr();
	OLED_SDIN_Set();
	
}

void IIC_Wait_Ack(void)
{
	OLED_SCLK_Set() ;
	OLED_SCLK_Clr();
}

/**********************************************
// IIC Write byte
**********************************************/
void Write_IIC_Byte(unsigned char IIC_Byte)
{
	unsigned char i;
	unsigned char m,da;
	da=IIC_Byte;
	OLED_SCLK_Clr();
	for(i=0;i<8;i++)		
	{
		m=da;
		m=m&0x80;
		if(m==0x80)
		{OLED_SDIN_Set();}
		else OLED_SDIN_Clr();
			da=da<<1;
		OLED_SCLK_Set();
		OLED_SCLK_Clr();
	}
}

/**********************************************
// IIC Write Command
**********************************************/
void Write_IIC_Command(unsigned char IIC_Command)
{
	IIC_Start();
   	Write_IIC_Byte(0x78); //Slave address,SA0=0
	IIC_Wait_Ack();	
   	Write_IIC_Byte(0x00); //write command
	IIC_Wait_Ack();	
   	Write_IIC_Byte(IIC_Command); 
	IIC_Wait_Ack();	
   	IIC_Stop();
}

/**********************************************
// IIC Write Data
**********************************************/
void Write_IIC_Data(unsigned char IIC_Data)
{
	IIC_Start();
   	Write_IIC_Byte(0x78); //D/C#=0; R/W#=0
	IIC_Wait_Ack();	
   	Write_IIC_Byte(0x40); //write data
	IIC_Wait_Ack();	
   	Write_IIC_Byte(IIC_Data);
	IIC_Wait_Ack();	
   	IIC_Stop();
}

void OLED_WR_Byte(unsigned dat,unsigned cmd)
{
	if(cmd)
	{
   		Write_IIC_Data(dat);
	}
	else
	{
   		Write_IIC_Command(dat);	
	}
}

/********************************************
// fill_Picture
********************************************/
void fill_picture(unsigned char fill_Data)
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		OLED_WR_Byte(0xb0+m,0); //page0-page1
		OLED_WR_Byte(0x00,0);   //low column start address
		OLED_WR_Byte(0x10,0);   //high column start address
		for(n=0;n<128;n++)
		{
			OLED_WR_Byte(fill_Data,1);
		}
	}
}

//坐标设置
void OLED_Set_Pos(unsigned char x, unsigned char y) 
{ 	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f),OLED_CMD); 
}   	  

//开启OLED显示    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}

//关闭OLED显示     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}	

//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
	} //更新显示
}

void OLED_On(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte(1,OLED_DATA); 
	} //更新显示
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示				 
//size:选择字体 16/12 
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size)
{      	
	unsigned char c=0,i=0;	
	c=chr-' ';//得到偏移后的值			
	if(x>Max_Column-1)
	{
		x=0;y=y+2;
	}
	if(Char_Size ==16)
	{
		OLED_Set_Pos(x,y);	
		for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
		OLED_Set_Pos(x,y+1);
		for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
	}
	else
	{	
		OLED_Set_Pos(x,y);
		for(i=0;i<6;i++)
			OLED_WR_Byte(F6x8[c][i],OLED_DATA);	
	}
}

u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size2/2)*t,y,' ',size2);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2); 
	}
} 
//显示一个字符号串
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 Char_Size)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		
		OLED_ShowChar(x,y,chr[j],Char_Size);
		x+=8;
		if(x>120){x=0;y+=2;}
			j++;
	}
}

//显示汉字
void OLED_ShowCHinese(u8 x,u8 y,u8 no)
{      			    
	u8 t,adder=0;
	OLED_Set_Pos(x,y);	
    for(t=0;t<16;t++)
	{
		OLED_WR_Byte(Hzk[2*no][t],OLED_DATA);
		adder+=1;
    }	
	OLED_Set_Pos(x,y+1);	
    for(t=0;t<16;t++)
	{	
		OLED_WR_Byte(Hzk[2*no+1][t],OLED_DATA);
		adder+=1;
    }					
}

/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
//void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
//{ 	
// unsigned int j=0;
// unsigned char x,y;
//  
//  if(y1%8==0) y=y1/8;      
//  else y=y1/8+1;
//	for(y=y0;y<y1;y++)
//	{
//		OLED_Set_Pos(x0,y);
//    for(x=x0;x<x1;x++)
//	    {      
//	    	OLED_WR_Byte(BMP1[j++],OLED_DATA);	    	
//	    }
//	}
//} 

//初始化SSD1306					    
void OLED_Init(void)
{ 	
	delay_ms(800);
	OLED_WR_Byte(0xAE,OLED_CMD);//--display off
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  
	OLED_WR_Byte(0xB0,OLED_CMD);//--set page address
	OLED_WR_Byte(0x81,OLED_CMD); // contract control
	OLED_WR_Byte(0xFF,OLED_CMD);//--128   
	OLED_WR_Byte(0xA1,OLED_CMD);//set segment remap 
	OLED_WR_Byte(0xA6,OLED_CMD);//--normal / reverse
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3F,OLED_CMD);//--1/32 duty
	OLED_WR_Byte(0xC8,OLED_CMD);//Com scan direction
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset
	OLED_WR_Byte(0x00,OLED_CMD);//
	
	OLED_WR_Byte(0xD5,OLED_CMD);//set osc division
	OLED_WR_Byte(0x80,OLED_CMD);//
	
	OLED_WR_Byte(0xD8,OLED_CMD);//set area color mode off
	OLED_WR_Byte(0x05,OLED_CMD);//
	
	OLED_WR_Byte(0xD9,OLED_CMD);//Set Pre-Charge Period
	OLED_WR_Byte(0xF1,OLED_CMD);//
	
	OLED_WR_Byte(0xDA,OLED_CMD);//set com pin configuartion
	OLED_WR_Byte(0x12,OLED_CMD);//
	
	OLED_WR_Byte(0xDB,OLED_CMD);//set Vcomh
	OLED_WR_Byte(0x30,OLED_CMD);//
	
	OLED_WR_Byte(0x8D,OLED_CMD);//set charge pump enable
	OLED_WR_Byte(0x14,OLED_CMD);//
	
	OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel
	
	OLED_Clear(); // 初始化后清屏
}  

// 更新显存到OLED	
void OLED_Refresh(void)
{
	u8 i,n;
	for(i=0;i<8;i++)
	{
	   	OLED_WR_Byte(0xb0+i,OLED_CMD); // 设置行起始地址
	   	OLED_WR_Byte(0x00,OLED_CMD);   // 设置低列起始地址
	   	OLED_WR_Byte(0x10,OLED_CMD);   // 设置高列起始地址
	   	for(n=0;n<128;n++)
			OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
	}
}

#elif ((I2C_MODE == 2) || (I2C_MODE == 3))
volatile uint32_t g_oled_i2c_tx_count_debug;
volatile uint32_t g_oled_i2c_probe_count_debug;
volatile uint32_t g_oled_i2c_nack_count_debug;
volatile uint8_t g_oled_i2c_last_ack_debug;
volatile uint8_t g_oled_i2c_scl_level_debug;
volatile uint8_t g_oled_i2c_sda_level_debug;

#if (I2C_MODE == 3)
#define OLED_SOFT_I2C_HALF_PERIOD_US 5U
#define OLED_SOFT_I2C_PROBE_RETRIES 20U

static void OLED_SoftI2C_Delay(void)
{
  uint32_t start_cycles;
  uint32_t delay_cycles;

  start_cycles = DWT->CYCCNT;
  delay_cycles = (SystemCoreClock / 1000000U) *
                 OLED_SOFT_I2C_HALF_PERIOD_US;
  while ((uint32_t)(DWT->CYCCNT - start_cycles) < delay_cycles)
  {
  }
}

static void OLED_SoftI2C_UpdatePinDebug(void)
{
  g_oled_i2c_scl_level_debug =
      (HAL_GPIO_ReadPin(OLED_SCL_GPIO_Port, OLED_SCL_Pin) ==
       GPIO_PIN_SET) ? 1U : 0U;
  g_oled_i2c_sda_level_debug =
      (HAL_GPIO_ReadPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin) ==
       GPIO_PIN_SET) ? 1U : 0U;
}

static void OLED_SoftI2C_Start(void)
{
  HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET);
  OLED_SoftI2C_Delay();
  HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_RESET);
  OLED_SoftI2C_Delay();
  HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_RESET);
  OLED_SoftI2C_Delay();
}

static void OLED_SoftI2C_Stop(void)
{
  HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_RESET);
  OLED_SoftI2C_Delay();
  HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET);
  OLED_SoftI2C_Delay();
  HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_SET);
  OLED_SoftI2C_Delay();
  OLED_SoftI2C_UpdatePinDebug();
}

static uint8_t OLED_SoftI2C_WriteByte(uint8_t value)
{
  uint8_t bit_index;
  uint8_t acknowledged;

  for (bit_index = 0U; bit_index < 8U; bit_index++)
  {
    HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_SDA_GPIO_Port,
                      OLED_SDA_Pin,
                      ((value & 0x80U) != 0U) ?
                          GPIO_PIN_SET : GPIO_PIN_RESET);
    OLED_SoftI2C_Delay();
    HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET);
    OLED_SoftI2C_Delay();
    value <<= 1U;
  }

  HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_SET);
  OLED_SoftI2C_Delay();
  HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET);
  OLED_SoftI2C_Delay();
  acknowledged =
      (HAL_GPIO_ReadPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin) ==
       GPIO_PIN_RESET) ? 1U : 0U;
  HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_RESET);
  OLED_SoftI2C_Delay();
  return acknowledged;
}

static void OLED_SoftI2C_RecoverBus(void)
{
  uint8_t pulse_index;

  HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_SET);
  for (pulse_index = 0U; pulse_index < 9U; pulse_index++)
  {
    HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_RESET);
    OLED_SoftI2C_Delay();
    HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET);
    OLED_SoftI2C_Delay();
  }
  OLED_SoftI2C_Stop();
}

static uint8_t OLED_SoftI2C_Probe(void)
{
  uint8_t acknowledged;

  OLED_SoftI2C_Start();
  acknowledged = OLED_SoftI2C_WriteByte(
      (uint8_t)(OLED_ADDRESS << 1U));
  OLED_SoftI2C_Stop();
  return acknowledged;
}

static uint8_t OLED_SoftI2C_Transmit(uint8_t control, uint8_t data)
{
  uint8_t acknowledged = 1U;

  OLED_SoftI2C_Start();
  acknowledged &= OLED_SoftI2C_WriteByte(
      (uint8_t)(OLED_ADDRESS << 1U));
  acknowledged &= OLED_SoftI2C_WriteByte(control);
  acknowledged &= OLED_SoftI2C_WriteByte(data);
  OLED_SoftI2C_Stop();

  g_oled_i2c_tx_count_debug++;
  g_oled_i2c_last_ack_debug = acknowledged;
  if (acknowledged == 0U)
  {
    g_oled_i2c_nack_count_debug++;
  }
  return acknowledged;
}

uint8_t OLED_IsReady(void)
{
  uint8_t attempt;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_SET);
  OLED_SoftI2C_RecoverBus();

  for (attempt = 0U; attempt < OLED_SOFT_I2C_PROBE_RETRIES; attempt++)
  {
    g_oled_i2c_probe_count_debug++;
    g_oled_i2c_last_ack_debug = OLED_SoftI2C_Probe();
    if (g_oled_i2c_last_ack_debug != 0U)
    {
      return 1U;
    }
    g_oled_i2c_nack_count_debug++;
    HAL_Delay(5U);
  }
  return 0U;
}

void I2C_WriteByte(uint8_t addr, uint8_t dat)
{
  (void)OLED_SoftI2C_Transmit(addr, dat);
}

#else
uint8_t OLED_IsReady(void)
{
  return (HAL_I2C_IsDeviceReady(&hi2c1,
                                (uint16_t)(OLED_ADDRESS << 1U),
                                2U,
                                10U) == HAL_OK) ? 1U : 0U;
}

void I2C_WriteByte(uint8_t addr, uint8_t dat)
{
  HAL_StatusTypeDef status;
  uint8_t temp[2];
  temp[0] = addr;
  temp[1] = dat;

  status = HAL_I2C_Master_Transmit(&hi2c1,
                                   (uint16_t)(OLED_ADDRESS << 1U),
                                   temp,
                                   2U,
                                   100U);
  g_oled_i2c_tx_count_debug++;
  g_oled_i2c_last_ack_debug = (status == HAL_OK) ? 1U : 0U;
  if (status != HAL_OK)
  {
    g_oled_i2c_nack_count_debug++;
  }
}
#endif


void WriteCmd(unsigned char I2C_Command)
{
    I2C_WriteByte(0x00, I2C_Command);
}

void WriteDat(unsigned char I2C_Data) 
{
    I2C_WriteByte(0x40, I2C_Data);
}

void OLED_WrCmd(unsigned char IIC_Command)
{
	WriteCmd(IIC_Command);
}

void OLED_Init(void)
{
	WriteCmd(0xAE); //display off
	WriteCmd(0x20);	//Set Memory Addressing Mode	
	WriteCmd(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	WriteCmd(0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	WriteCmd(0xc8);	//Set COM Output Scan Direction
	WriteCmd(0x00); //---set low column address
	WriteCmd(0x10); //---set high column address
	WriteCmd(0x40); //--set start line address
	WriteCmd(0x81); //--set contrast control register
	WriteCmd(0xff); //áá?èμ÷?ú 0x00~0xff
	WriteCmd(0xa1); //--set segment re-map 0 to 127
	WriteCmd(0xa6); //--set normal display
	WriteCmd(0xa8); //--set multiplex ratio(1 to 64)
	WriteCmd(0x3F); //
	WriteCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	WriteCmd(0xd3); //-set display offset
	WriteCmd(0x00); //-not offset
	WriteCmd(0xd5); //--set display clock divide ratio/oscillator frequency
	WriteCmd(0xf0); //--set divide ratio
	WriteCmd(0xd9); //--set pre-charge period
	WriteCmd(0x22); //
	WriteCmd(0xda); //--set com pins hardware configuration
	WriteCmd(0x12);
	WriteCmd(0xdb); //--set vcomh
	WriteCmd(0x40); //0x20,0.77xVcc
	WriteCmd(0x8d); //--set DC-DC enable
	WriteCmd(0x14); //
	WriteCmd(0xaf); //--turn on oled panel
}

void OLED_SetPos(unsigned char x, unsigned char y) 
{ 
	WriteCmd(0xb0+y);
	WriteCmd(((x&0xf0)>>4)|0x10);
	WriteCmd((x&0x0f)|0x01);
}

void OLED_Fill(unsigned char fill_Data)//è??áì?3?
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		WriteCmd(0xb0+m);		//page0-page1
		WriteCmd(0x00);		//low column start address
		WriteCmd(0x10);		//high column start address
		for(n=0;n<128;n++)
			{
				WriteDat(fill_Data);
			}
	}
}

void OLED_CLS(void)//???á
{
	OLED_Fill(0xff);
}

void OLED_ON(void)
{
	WriteCmd(0X8D);  //éè??μ?oé±?
	WriteCmd(0X14);  //?a??μ?oé±?
	WriteCmd(0XAF);  //OLED??D?
}

void OLED_OFF(void)
{
	WriteCmd(0X8D);  //éè??μ?oé±?
	WriteCmd(0X10);  //1?±?μ?oé±?
	WriteCmd(0XAE);  //OLEDDY??
}

void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
{
	unsigned char c = 0,i = 0,j = 0;
	switch(TextSize)
	{
		case 1:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 126)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				for(i=0;i<6;i++)
					WriteDat(asc2_0806[c][i]);
				x += 6;
				j++;
			}
		}break;
		case 2:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 120)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				for(i=0;i<8;i++)
					WriteDat(asc2_1608[c][i]);
				OLED_SetPos(x,y+1);
				for(i=0;i<8;i++)
					WriteDat(asc2_1608[c][i+8]);
				x += 8;
				j++;
			}
		}break;
	}
}

// void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N)
// {
// 	unsigned char wm=0;
// 	unsigned int  adder=32*N;
// 	OLED_SetPos(x , y);
// 	for(wm = 0;wm < 16;wm++)
// 	{
// 		WriteDat(asc2_1608[adder]);
// 		adder += 1;
// 	}
// 	OLED_SetPos(x,y + 1);
// 	for(wm = 0;wm < 16;wm++)
// 	{
// 		WriteDat(asc2_1608[adder]);
// 		adder += 1;
// 	}
// }

void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[])
{
	unsigned int j=0;
	unsigned char x,y;

  if(y1%8==0)
		y = y1/8;
  else
		y = y1/8 + 1;
	for(y=y0;y<y1;y++)
	{
		OLED_SetPos(x0,y);
    for(x=x0;x<x1;x++)
		{
			WriteDat(BMP[j++]);
		}
	}
}

void SmartCar_OLED_P8x16Str(uint8_t x, uint8_t y, const char* ch)
{
    uint8_t c = 0;
    uint8_t i = 0;
    uint8_t j = 0;
    while(ch[j] != '\0')
    {
        c = ch[j] - 32;
        if(ch[j] == '\n')
        {
            y += 2;
            x = 0;
            continue;
        }
        if(x>120)
        {
            x = 0;
            y += 2;
        }
        if(y > 6)
        {
            break;
        }
        OLED_SetPos(x, y);
        for(i = 0;i<8;i++)
        {
            WriteDat(asc2_1608[c][i]);//填入数组
        }
        OLED_SetPos(x, y+1);
        for(i = 0;i<8;i++)
        {
            WriteDat(asc2_1608[c][i+8]);//填入数组
        }
        x += 8;
        j++;
    }
}
#endif
