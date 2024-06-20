#include "user.h"

#include <stdio.h>
#include <string.h>

#define BASE_ADDR 0
#include "led.h"

uint8_t channels[256];

void init(void) {
	memset(&channels, 0, sizeof(channels));
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	channels[0] = 1;
	channels[1] = 0;
	channels[2] = 1;

	HAL_Delay(20); /* Give trace time to start before printing */
	printf("ADDR: %u \n", BASE_ADDR);
	printf("L1: \t R: %u \t G: %u \t B: %u \n", LR1_ADDR, LG1_ADDR, LB1_ADDR);
	printf("L2: \t R: %u \t G: %u \t B: %u \n", LR2_ADDR, LG2_ADDR, LB2_ADDR);
	printf("L3: \t R: %u \t G: %u \t B: %u \n", LR3_ADDR, LG3_ADDR, LB3_ADDR);
}

void run(void) {
	for (size_t i = 0; i < sizeof(channels); i++) {
		led_write(i, channels[i]);
	}
}

void err(void) {
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}
