/*
 * klawiatura.h
 *
 *  Created on: 17 Jun 2022
 *      Author: asliw
 */

#ifndef INC_KLAWIATURA_H_
#define INC_KLAWIATURA_H_

void init_keyboard(void);

// Odczytaj numer przycisku klikniętego na klawiaturze (odczytany tylko o najniższym numerze)
// Numerowanie od lewego górnego rogu, od 0
int keyboard(void);

#endif /* INC_KLAWIATURA_H_ */
