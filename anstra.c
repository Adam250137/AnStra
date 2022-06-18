/*
 * anstra.c
 *
 *  Created on: 17 Jun 2022
 *      Author: asliw
 */

#include "anstra.h"
#include "main.h"
#include "lcd.h"
#include "stdio.h"
#include "math.h"


// ilość otrzymanych zmiennych
char pos_stat = 0;

// status pozycji anteny
char status = 0;

double target_latitude, target_longitude, target_height;

// height angle, direction angle
int ha, da;

// N, E, kąt do równoleżnika
double latitude;
double longitude;
double direction;

int R = 6371;

// od 0 do 90 stopni, 0 oznacza pion
void height_angle( int angle ){
	ha = angle;
	float tmp = 893.0/90.0;
	TIM3->CCR3 = 1607 + tmp*angle;
}

// od -135 do 135
void direction_angle( int angle ){
	da = angle;
	float tmp = 1000.0/135.0;
	TIM3->CCR4 = 1500 + tmp*angle;
}

void display_angle(){
	lcd_clear();
	sprintf(text, "Wysokosc: %03d",ha);
	lcd_send_string(text);

	lcd_line(1);
	sprintf(text, "Kierunek: %03d",da);
	lcd_send_string(text);
}

void set_ANSTRA_pos(){
	pos_stat = 0;
	status = 0;
	lcd_display(0b111);

	lcd_clear();
	sprintf(text, "Pozycja ANSTRY:" );
	lcd_send_string(text);

	lcd_line(1);
}

void set_ANSTRA_angle(){
	status = 2;
	lcd_display(0b111);
	pos_stat = 0;

	lcd_clear();
	sprintf(text, "Ustaw kat:" );
	lcd_send_string(text);

	lcd_line(1);
}

void set_ANSTRA_target(){
	pos_stat = 0;
	status = 3;
	lcd_display(0b111);

	lcd_clear();
	sprintf(text, "Ustaw cel:" );
	lcd_send_string(text);

	lcd_line(1);
}

void get_ANSTRA_pos( double ret ){
	if( pos_stat == 0 )
		latitude = ret;

	if( pos_stat == 1 )
		longitude = ret;

	if( pos_stat == 2 )
		direction = ret;

	pos_stat++;
	if( pos_stat == 3 ){
		status = 1;
		lcd_display(0b100);
		height_angle(ha);
		direction_angle(da);
		display_angle();
	}

	lcd_line(pos_stat+1);
}

void get_ANSTRA_angle( double ret ){
	if( pos_stat == 0 )
	  da = ret;

	if( pos_stat == 1 )
	  ha = ret;

	pos_stat++;
	if( pos_stat == 2 ){
	  height_angle(ha);
	  direction_angle(da);
	  display_angle();
	  status = 1;
	  lcd_display(0b100);
	}

	lcd_line( pos_stat+1 );
}

void get_ANSTRA_target( double ret ){
	if( pos_stat == 0 )
	  target_latitude = ret;

	if( pos_stat == 1 )
	  target_longitude = ret;

	if( pos_stat == 2 )
	  target_height= ret;

	pos_stat++;
	if( pos_stat == 3 ){
	  target_to_angle();
	  height_angle(ha);
	  direction_angle(da);
	  display_angle();
	  status = 1;
	  lcd_display(0b100);
	}

	lcd_line( pos_stat+1 );
}

void target_to_angle(){
	double alf = (target_longitude - longitude)/180.0*M_PI;
	double bet = (target_latitude - latitude)/180.0*M_PI;
//
	double satX = cos(alf)*cos(bet)*(target_height+R)-R;
	double satY = sin(alf)*cos(bet)*(target_height+R);
	double satZ = sin(bet)*(target_height+R);
//
	ha = acos( satX/( sqrt( satX*satX+satY*satY+satZ*satZ ) ) )/M_PI*180.0;
	da = atan(satZ/satY)/M_PI*180.0 - direction;
	if( satY < 0 )
		if( da > 0 )
			da -= 180;
		else
			da += 180;
}

