/************************************************************************
 * lcd1602.c
 *
 ************************************************************************/

#include "Delay.h"
#include "lcd1602.h"
#include "main.h"

/* initialize the LCD module */
/* define macros for LCD instructions*/
#define LCD_IDLE 0x33
#define LCD_2_LINE_4_BITS 0x28
#define LCD_2_LINE_8_BITS 0x38
// cursor enabled
// #define LCD_DSP_CSR 0x0f
// cursor dissabled
#define LCD_DSP_CSR 0x0f
#define LCD_CLR_DSP 0x01
#define LCD_CSR_INC 0x06
#define LCD_SFT_MOV 0x14

void LCD_init(void) // LCD初始化函数
{
  //	Delay_ms(15);
  //	HAL_GPIO_WritePin(GPIOB,LCD_RS_Pin, GPIO_PIN_RESET); // LCD_RS = 0,send
  // Command 	LCD_PORT = LCD_IDLE; //function set -8 bit interface
  // Delay_ms(5);
  ////wait for 5 ms 	LCD_PORT = LCD_IDLE;	//function set -8 bit interface
  //	Delay_us(100);			//wait for 100us
  //	LCD_PORT = LCD_IDLE;  // function set
  //	Delay_ms(5);
  //	LCD_PORT = LCD_IDLE;
  //	Delay_us(100);

  // LCD_Write_Command(0x38);
  LCD_Write_Command(LCD_2_LINE_8_BITS); // 显示模式设置,8位显示模式
  Delay_ms(5); // 显示模式设置,8位显示模式，两行数据
  LCD_Write_Command(LCD_2_LINE_8_BITS); // 显示模式设置,8位显示模式
  // LCD_Write_Command(0x08); // BJ 关闭显示；显示开关及光标指令设置；
  // LCD_Write_Command(LCD_2_LINE_8_BITS);//显示模式设置,8位显示模式

  // LCD_Write_Command(0x01);
  LCD_Write_Command(LCD_CLR_DSP); // 显示清屏
  Delay_us(100);
  // LCD_Write_Command(0x06);
  LCD_Write_Command(LCD_CSR_INC); // 显示光标右移
  Delay_us(100);
  // LCD_Write_Command(0x0c);
  LCD_Write_Command(LCD_DSP_CSR); // 显示开,光标关，光标闪烁关
}

void LCD_Clear(void) {
  LCD_Write_Command(0x01); //
}

/*****
* RS:  高电平，表示写的是数据data（LCD屏幕显示数据）;
           低电平： 表示写的是命令command（向LCD内部寄存器中数据）;
* R/~W : 低电平， 向LCD1602 写；
         高电平， 从LCD1602 读
* E : 高电平使能信号
*DB0~DB7 数据总线
*  LCD  RS  RW		意思
*		0	0		写命令
*		0   1       读命令
*		1   0		写数据
*		1   1       读数据
*
*/

void LCD_Write_Command(uchar Com) {
  // while(lcd_read_state());
  Delay_ms(10);
  //	unsigned int Read_Dat = 0; //BJ

  HAL_GPIO_WritePin(GPIOB, LCD_RS_Pin, GPIO_PIN_RESET); // LCD_RS = 0;
  HAL_GPIO_WritePin(GPIOB, LCD_RW_Pin, GPIO_PIN_RESET); //	LCD_RW = 0;
  HAL_GPIO_WritePin(GPIOB, LCD_E_Pin, GPIO_PIN_SET);    // LCD_E_Pin = 1;
  Delay_ms(1);

  LCD_PORT = Com;

  HAL_GPIO_WritePin(GPIOB, LCD_E_Pin, GPIO_PIN_RESET); // LCD_E_Pin =0;
  Delay_ms(1);                                         // 延时tpw;
  // HAL_GPIO_WritePin(GPIOB,LCD_E_Pin, GPIO_PIN_SET);  //LCD_E_Pin = 1;
  // Delay_ms(1);//延时tpw;
  // HAL_GPIO_WritePin(GPIOB,LCD_E_Pin, GPIO_PIN_RESET);  //LCD_E_Pin = 0,
  // DB0~DB7 Validate
}

void LCD_Write_Data(uchar dat) {
  Delay_ms(1);

  HAL_GPIO_WritePin(GPIOB, LCD_RS_Pin, GPIO_PIN_SET);   // LCD_RS = 1;
  HAL_GPIO_WritePin(GPIOB, LCD_RW_Pin, GPIO_PIN_RESET); //	LCD_RW = 0;
  HAL_GPIO_WritePin(GPIOB, LCD_E_Pin, GPIO_PIN_SET);    // LCD_E_Pin = 1;
  // Delay_ms(1);

  LCD_PORT = dat; // put data on the bus

  HAL_GPIO_WritePin(GPIOB, LCD_E_Pin, GPIO_PIN_RESET); // Set LCD_E = 0;

  Delay_ms(1); // wait for tpw;
}

void LCD_Set_Position(uchar x, uchar y) {
  if (y == 0) // 第一行开始显示
  {
    LCD_Write_Command(0x80 + x);
  } else if (y == 1) // 第二行开始显示
  {
    LCD_Write_Command(0xc0 + x);
  }
}

void LCD_Display_Char(uchar Char, uchar x, uchar y) // 显示字符ASCII码
{
  LCD_Set_Position(x, y);
  LCD_Write_Data(Char);
}
void LCD_Display_String(uchar x, uchar y, uchar *str) {
  LCD_Set_Position(x, y); // 当前字符的坐标（调用了坐标显示函数）
  // while(str[i] !='\0')   //‘\0'判断字符串是否结束标志
  while (*str != '\0') {
    // LCD_Write_Data(str[i]); //写入内容对应的ASCII用于显示
    LCD_Write_Data(*str++);
    // i++;
  }
}

/*************end of LCD.c**************/
