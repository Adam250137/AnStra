/*
 * utillib.h
 *
 *  Created on: 17 Jun 2022
 *      Author: asliw
 */

#ifndef INC_UTILLIB_H_
#define INC_UTILLIB_H_

void blink(void);

// Serwa
void init_PWM(void);

void save_input( int k );

double get_input();

void send_data();

#endif /* INC_UTILLIB_H_ */
