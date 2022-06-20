/*
 * anstra.h
 *
 *  Created on: 17 Jun 2022
 *      Author: asliw
 */

#ifndef INC_ANSTRA_H_
#define INC_ANSTRA_H_

extern int ha, da;
extern double target_latitude, target_longitude, target_height;
extern char status;

// od 0 do 90 stopni, 0 oznacza pion
void height_angle( int angle );

// od -135 do 135
void direction_angle( int angle );

void display_angle();

void set_ANSTRA_pos();

void set_ANSTRA_angle();

void set_ANSTRA_target();

void get_ANSTRA_pos( double ret );

void get_ANSTRA_angle( double ret );

void get_ANSTRA_target( double ret );

void target_to_angle();

#endif /* INC_ANSTRA_H_ */
