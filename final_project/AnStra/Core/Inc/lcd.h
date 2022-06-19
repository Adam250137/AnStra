/*
 * lcd.h
 *
 *  Created on: 17 Jun 2022
 *      Author: asliw
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

extern char text[21];

// prototype
void blink(void);

#define SLAVE_ADDRESS_LCD 0x3F<<1 // adres przesunięty dla MasterTransmit()

// lcd library
int lcd_send_cmd (char cmd);

int lcd_checkCon(void);

void lcd_clear();

void lcd_line(int n);

// 3bit: D,C,B
//D - display on/off
//C - cursor on/off
//B - cursor blink on/off
void lcd_display( int set );

int  lcd_send_data (char data);

void lcd_init (void);

void lcd_send_string (char *str);


#endif /* INC_LCD_H_ */
