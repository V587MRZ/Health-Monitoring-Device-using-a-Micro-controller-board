
/** include this file in the src folder **/

#include "i2c-lcd.h"
#include "main.h"
extern I2C_HandleTypeDef hi2c1;  // 这里使用I2C1来连接LCD，请根据情况自行修改


#define Write_Add_LCD 0x4E
#define Read_Add_LCD 0x4F
//
//#define Write_Add_LCD 0x3F // LCD的I2C模块地址，参考模块的规格书，0x3F为高7位地址，STM32需要手动左移地址

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	while (HAL_I2C_Master_Transmit (&hi2c1, Write_Add_LCD,(uint8_t *) data_t, 4, 1000)!= HAL_OK)
	    {
	        if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
	                {
	                  Error_Handler();
	                }
	    }
}



void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
//	HAL_I2C_Master_Transmit (&hi2c1, Write_Add_LCD,(uint8_t *) data_t, 4, 100);
	while (HAL_I2C_Master_Transmit (&hi2c1, Write_Add_LCD,(uint8_t *) data_t, 4, 1000)!= HAL_OK)
//	   while(HAL_I2C_Master_Transmit(&hi2c1,I2C1_WRITE_ADDRESS,(uint8_t*)TxData,2,1000) != HAL_OK)
	    {
	        if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
	                {
	                  Error_Handler();
	                }
	    }
}



void lcd_init (void)
{
	lcd_send_cmd (0x02);//
	HAL_Delay(5);
	lcd_send_cmd (0x28);
	HAL_Delay(5);
	lcd_send_cmd (0x0c);//开启显示屏,关光标,光标不闪烁
	HAL_Delay(5);
	lcd_send_cmd (0x80);//光标定位到左上角
	HAL_Delay(5);
//	lcd_send_cmd (0x06);//显示地址递增,即写一个数据后,显示位置右移一位
	HAL_Delay(15);
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

void lcd_setCursor (char col, char row) //col代表列数，从0开始计算；row代表行数，row =0,1,2,3分别代表第1/2/3/4行, LCD1602只有2行，LCD2004有4行
{
	char lineAdd = 0x80;
	switch(row)
	{
		case 0: lineAdd = 0x80 + col;
				break;
		case 1: lineAdd = 0xC0 + col;
				break;
		case 2: lineAdd = 0x94 + col;
				break;
		case 3: lineAdd = 0xD4 + col;
				break;

	}
	lcd_send_cmd(lineAdd);
	HAL_Delay(15);
}



void lcd_clear(void)
{
	lcd_send_cmd(0x01);
	HAL_Delay(15);
}

void lcd_backlight(void)
{
	uint8_t val;
	HAL_I2C_Master_Receive (&hi2c1, Read_Add_LCD,&val, 1, 1000);
	val |= 1<<3; //P3 bit置1
	HAL_I2C_Master_Transmit (&hi2c1, Write_Add_LCD,&val, 1, 1000);
}

void lcd_noBacklight(void)
{
	uint8_t val;
	HAL_I2C_Master_Receive (&hi2c1, Read_Add_LCD,&val, 1, 1000);
	val &= ~(1<<3); //P3 bit清0
	HAL_I2C_Master_Transmit (&hi2c1, Write_Add_LCD,&val, 1, 1000);
}

