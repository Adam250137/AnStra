/*
 * klawiatura.c
 *
 *  Created on: 17 Jun 2022
 *      Author: asliw
 */
#include "klawiatura.h"

// klawiatura
int K[8];
GPIO_TypeDef* KI;
GPIO_TypeDef* KO;

void init_keyboard(void){
	K[0] = K1_Pin;
	K[1] = K2_Pin;
	K[2] = K3_Pin;
	K[3] = K4_Pin;
	K[4] = K5_Pin;
	K[5] = K6_Pin;
	K[6] = K7_Pin;
	K[7] = K8_Pin;

	KI = K1_GPIO_Port;
	KO = K5_GPIO_Port;
}

// Odczytaj numer przycisku klikniętego na klawiaturze (odczytany tylko o najniższym numerze)
// Numerowanie od lewego górnego rogu, od 0
int keyboard(void){
	for(int i=4; i<8; i++){
		HAL_GPIO_TogglePin(KO, K[i]);
		for(int j=0; j<4; j++)
			if( HAL_GPIO_ReadPin(KI, K[j]) ){
				HAL_GPIO_TogglePin(KO, K[i]);
				return (i-4)+j*4;
			}
		HAL_GPIO_TogglePin(KO, K[i]);
	}
	return -1;
}


