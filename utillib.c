/*
 * utillib.c
 *
 *  Created on: 17 Jun 2022
 *      Author: asliw
 */
#include "utillib.h"
#include "lcd.h"
#include "main.h"
#include "stdlib.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include "stdio.h"
#include "anstra.h"

extern TIM_HandleTypeDef htim3;
// LCD
char input[21];
char* ptr = input;

// USB
uint8_t TxData[1000];

void blink(void){
	HAL_GPIO_TogglePin(Test_LED_GPIO_Port, Test_LED_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(Test_LED_GPIO_Port, Test_LED_Pin);
	HAL_Delay(100);
}

// Serwa
void init_PWM(void){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	TIM3->CCR3 = 1500;

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	TIM4->CCR4 = 1500;
}

void save_input( int k ){
	char c;
	switch(k){
		case 12:
			c = '-';
			break;
		case 14:
			c = '.';
			break;
		case 0:
			c = '1';
			break;
		case 1:
			c = '2';
			break;
		case 2:
			c = '3';
			break;
		case 4:
			c = '4';
			break;
		case 5:
			c = '5';
			break;
		case 6:
			c = '6';
			break;
		case 8:
			c = '7';
			break;
		case 9:
			c = '8';
			break;
		case 10:
			c = '9';
			break;
		case 13:
			c = '0';
			break;
		default:
			c = 0;
	}

	if( c > 0 ){

		lcd_send_data(c);
		*(ptr++) = c;
	}
}

double get_input(){
	char* tmp;
	*ptr = '\0';
	double ret = strtod(input,&tmp);
	if( ptr != tmp ){
		blink();
	}

	ptr = input;
	return ret;
}

void send_data(){
	sprintf((char*)TxData, "SEND_ANGLES s1 %d; s2 %d; SEND_TARGET latitude %f; longitude %f; height %f; \r\n",
			da, ha, target_latitude, target_longitude, target_height);
	CDC_Transmit_FS(TxData, strlen((char*)TxData));
}


