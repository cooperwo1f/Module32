#include "user.h"
#include <stdio.h>

int i = 0;

void init(void) {
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void run(void) {
	printf("i = %u \n", i++);
	HAL_Delay(50);
}

void err(void) {
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}
