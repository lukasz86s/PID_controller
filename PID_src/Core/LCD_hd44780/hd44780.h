/*
 * hd44780.h
 *
 *  Created on: 15 sie 2021
 *      Author: fet
 */

#ifndef LCD_HD44780_HD44780_H_
#define LCD_HD44780_HD44780_H_
#include "stdint.h"
#include "LCD_init.h"
// reg operations

#define SET_BIT(REG, BIT) 	((REG) |= (BIT))
#define RESET_BIT(REG, BIT) ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT) 	((REG) & (BIT))
#define WRITE_REG(REG, VAL) ((REG) = (VAL))
#define READ_REG(REG)		((REG))
#define _CLEAR_REG(REG)		((REG) = 0x0)

// ------------ GPIO ADDRESS for cortex4

#define _GPIOA 	0x40020000UL
#define _GPIOB	0x40020400UL
#define _GPIOC	0x40020800UL
#define _GPIOD	0x40020c00UL
#define _GPIOE	0x40021000UL
#define _GPIOH	0x40021c00UL
//--------------------------------------

// address for different sizes of LCD
#if ((LCD_Y == 4) && (LCD_X == 20))
#define LCD_LINE1 0x00
#define LCD_LINE2 0x28
#define LCD_LINE3 0x14
#define LCD_LINE4 0x54
#else
#define LCD_LINE1 0x00
#define LCD_LINE2 0x40
#define LCD_LINE3 0x10
#define LCD_LINE4 0x50
#endif
//---------------------------------------

// ---
#define PORTA ((PORTx_TypeDef *) (_GPIOA))
#define PORTB ((PORTx_TypeDef *) (_GPIOB))
#define PORTC ((PORTx_TypeDef *) (_GPIOC))
#define PORTD ((PORTx_TypeDef *) (_GPIOD))
#define PORTE ((PORTx_TypeDef *) (_GPIOE))
#define PORTH ((PORTx_TypeDef *) (_GPIOH))

// -- port makro
#define PORT(x) SPORT(x)
#define SPORT(x) (PORT##x)

#define SET_RS	PORT(LCD_RS_PORT)->BSRR |= (1<<(LCD_RS))
#define CLR_RS	PORT(LCD_RS_PORT)->BSRR |= (1<<(LCD_RS+16))

#define SET_E	PORT(LCD_E_PORT)->BSRR |= (1<<(LCD_E))
#define CLR_E	PORT(LCD_E_PORT)->BSRR |= (1<<(LCD_E+16))

#define SET_RW	PORT(LCD_RW_PORT)->BSRR |= (1<<(LCD_RW))
#define CLR_RW	PORT(LCD_RW_PORT)->BSRR |= (1<<(LCD_RW+16))
// todo: create GPIO for cortex3
// stuct GPIO in cortex4
typedef
		struct _PORTx_TypeDef{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEDDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
}PORTx_TypeDef;


// functions
static inline void lcd_set_pin(PORTx_TypeDef *port, uint8_t pin){
	port->BSRR |= (1<<pin);
}
static inline void lcd_reset_pin(PORTx_TypeDef *port, uint8_t pin){
	port->BSRR |= (1<<(pin+16));
}

static inline uint8_t lcd_read_pin(PORTx_TypeDef *port, uint8_t pin){
	 if(port->IDR & (1<<pin))
		 return 1;
	 return 0;
}
// todo: maybe create function to read all pins D7-D4?
// 		or get atomic access with bit-band?
// sat as input with resistor to low
static inline void lcd_set_to_input(PORTx_TypeDef *port, uint8_t pin){
	//--------- port mode ----------------------------//
	// reset configure bits ( 00 - set as input )
	RESET_BIT(port->MODER, (3<<(pin*2U)));
	//reset bits
	RESET_BIT(port->PUPDR, (3<<(pin*2U)));
	// push down
	SET_BIT(port->PUPDR, (2<<(pin*2U)));

}
// set as output
static inline void lcd_set_to_output(PORTx_TypeDef *port, uint8_t pin){
	//--------- port mode ----------------------------//
	// reset configure bits
	RESET_BIT(port->MODER, (3<<(pin*2U)));
	//set as output
	SET_BIT(port->MODER, (1<<(pin*2)));

	RESET_BIT(port->PUPDR, (3<<(pin*2U)));
}

static inline void lcd_send_half(uint8_t data){
	if(data & (1<<0))
		lcd_set_pin(PORT(LCD_D4_PORT), LCD_D4);
	else
		lcd_reset_pin(PORT(LCD_D4_PORT), LCD_D4);

	if(data & (1<<1))
		lcd_set_pin(PORT(LCD_D5_PORT), LCD_D5);
	else
		lcd_reset_pin(PORT(LCD_D5_PORT), LCD_D5);

	if(data & (1<<2))
		lcd_set_pin(PORT(LCD_D6_PORT), LCD_D6);
	else
		lcd_reset_pin(PORT(LCD_D6_PORT), LCD_D6);

	if(data & (1<<3))
		lcd_set_pin(PORT(LCD_D7_PORT), LCD_D7);
	else
		lcd_reset_pin(PORT(LCD_D7_PORT), LCD_D7);
}
#if USE_RW == 1
static inline uint8_t lcd_read_half(void){
	uint8_t data = 0;
	if(lcd_read_pin(PORT(LCD_D4_PORT), LCD_D4)) data |= (1<<0);
	if(lcd_read_pin(PORT(LCD_D5_PORT), LCD_D5)) data |= (1<<1);
	if(lcd_read_pin(PORT(LCD_D6_PORT), LCD_D6)) data |= (1<<2);
	if(lcd_read_pin(PORT(LCD_D7_PORT), LCD_D7)) data |= (1<<3);

	return data;
}

// functions
uint8_t lcd_read_byte(void);
uint8_t check_BF(void);
#endif

void lcd_init_pin(PORTx_TypeDef *port, uint8_t pin);
void lcd_write_byte(unsigned char data);
void lcd_write_cmd(uint8_t cmd);
void lcd_write_data(uint8_t data);

uint8_t check_BF(void);



void (*_delay_us)(uint32_t us);
#endif /* LCD_HD44780_HD44780_H_ */
