/*
 * hd44780.c
 *
 *  Created on: 15 sie 2021
 *      Author: fet
 */
#include "hd44780.h"

// init pin as out PP ,medium speed ,  no Pu/Pd
// port -
void lcd_init_pin(PORTx_TypeDef *port, uint8_t pin){

	//--------- port mode ----------------------------//
	// reset configure bits
	RESET_BIT(port->MODER, (3<<(pin*2U)));
	//set as output
	SET_BIT(port->MODER, (1<<(pin*2)));
	//------------------------------------------------//

	//--------- configure output mode ----------------//
	// reset configure bit (0 - set as output push-pull
	RESET_BIT(port->OTYPER, (1<<(pin)));
	//------------------------------------------------//

	//--------- configure seed -----------------------//
	// reset configure bits
	RESET_BIT(port->OSPEDDR, (3<<(pin*2U)));
	// set medium speed
	SET_BIT(port->OSPEDDR, (1<<(pin*2)));
	//------------------------------------------------//

	//--------- pull-up/ pull-down -------------------//
	// reset configure bits (00 - no pull-up/down ----//
	RESET_BIT(port->PUPDR, (3<<(pin*2U)));
	//------------------------------------------------//

}

void init_lcd(void(*delay_us)(uint32_t us)){
	//regist delay funcition
	_delay_us = delay_us;
	// config pins as output pp , low
	lcd_init_pin(PORT(LCD_D7_PORT), LCD_D7);
	lcd_init_pin(PORT(LCD_D6_PORT), LCD_D6);
	lcd_init_pin(PORT(LCD_D5_PORT), LCD_D5);
	lcd_init_pin(PORT(LCD_D4_PORT), LCD_D4);
	lcd_init_pin(PORT(LCD_RS_PORT), LCD_RS);
	lcd_init_pin(PORT(LCD_E_PORT), LCD_E);
#if USE_RW == 1
	lcd_init_pin(PORT(LCD_RW_PORT), LCD_RW);
#endif
	// set high on control lines
	SET_RS;
	SET_E;
#if USE_RW == 1
	SET_RW;
#endif
	// delay 15 ms

	_delay_us(15000);

	// set low on control linese
	CLR_E;
	CLR_RS;
#if USE_RW == 1
	CLR_RW;
#endif
	// DL = 1 , 8 bit mode
	SET_E;
	lcd_send_half(0x03);
	CLR_E;
	_delay_us(4100);

	// DL = 1
	SET_E;
	lcd_send_half(0x03);
	CLR_E;
	_delay_us(100);

	//DL = 1
	SET_E;
	lcd_send_half(0x03);
	CLR_E;
	_delay_us(100);

	// DL = 0 4bit mode
	SET_E;
	lcd_send_half(0x02);
	CLR_E;
	_delay_us(100);

	// settings
	lcd_write_cmd(0b00101000);
	lcd_write_cmd(0b00001000);
	lcd_write_cmd(0b00001111);
	lcd_write_cmd(0b00010100);
	lcd_cls();
	// set cursor on start position
	lcd_write_cmd(0x02);


}
void lcd_cls(void){
	lcd_write_cmd(0x01);
#if USE_RW == 0
	_delay_us(4900);
#endif
}
void lcd_locate(uint8_t y, uint8_t x){
	switch(y){

	case 0: y = LCD_LINE1; break;
#if (LCD_Y>1)
	case 1: y = LCD_LINE2; break;
#endif
#if (LCD_Y>2)
	case 2: y = LCD_LINE3; break;
#endif
#if (LCD_Y>3)
	case 3: y = LCD_LINE4; break;
#endif
	}
	lcd_write_cmd((0x80+y+x));
}
void lcd_str(char *str){
	while(*str)
		lcd_write_data(*str++);
}

// sedn byte to lcd
void lcd_write_byte(unsigned char data){
	// set D4-D7 pins as outputs
	lcd_set_to_output(PORT(LCD_D4_PORT), LCD_D4);
	lcd_set_to_output(PORT(LCD_D5_PORT), LCD_D5);
	lcd_set_to_output(PORT(LCD_D6_PORT), LCD_D6);
	lcd_set_to_output(PORT(LCD_D7_PORT), LCD_D7);

#if USE_RW == 1
	CLR_RW;
#endif

	SET_E;
	//sending 4 old bits
	lcd_send_half(data>>4);
	CLR_E;

	SET_E;
	// sending 4 young bits
	lcd_send_half(data);
	CLR_E;

#if USE_RW == 1
	while(check_BF() & (1<<7));
#else
	_delay_us(120);
#endif

}

void lcd_write_cmd(uint8_t cmd){
	CLR_RS;
	lcd_write_byte(cmd);
}
void lcd_write_data(uint8_t data){
	SET_RS;
	lcd_write_byte(data);
}
#if USE_RW == 1


uint8_t lcd_read_byte(void){
	uint8_t data = 0;
	// set D4-D7 pins as inputs
	lcd_set_to_input(PORT(LCD_D4_PORT), LCD_D4);
	lcd_set_to_input(PORT(LCD_D5_PORT), LCD_D5);
	lcd_set_to_input(PORT(LCD_D6_PORT), LCD_D6);
	lcd_set_to_input(PORT(LCD_D7_PORT), LCD_D7);


	SET_RW;
	SET_E;
	// read 4 old bits
	data|= (lcd_read_half() << 4);
	CLR_E;
	SET_E;
	// read 4 young bits
	data |= lcd_read_half();
	CLR_E;

	return data;
}

uint8_t check_BF(void){
	CLR_RS;
	return lcd_read_byte();
}

#endif
