#include "user.h"
#include "stm32f1xx_hal.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BASE_ADDR 0
#include "led.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern DMA_HandleTypeDef hdma_tim2_up;
extern DMA_HandleTypeDef hdma_tim3_up;
extern DMA_HandleTypeDef hdma_tim4_up;

#define CHANNELS_LENGTH 256

uint16_t channels[CHANNELS_LENGTH];

uint32_t data_A[256];
uint32_t data_B[256];

void init(void) {
	memset(&channels, 0, sizeof(channels));

	channels[0] = 255;

	memset(&data_A, 0, sizeof(data_A));
	memset(&data_B, 0, sizeof(data_B));

	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim3);

	/* CONFIGURE DMA */
	HAL_DMA_Start(&hdma_tim2_up, (uint32_t)&(data_A[0]), (uint32_t)&(GPIOA->BSRR), sizeof(data_A)/sizeof(data_A[0]));
	HAL_DMA_Start(&hdma_tim3_up, (uint32_t)&(data_B[0]), (uint32_t)&(GPIOB->BSRR), sizeof(data_B)/sizeof(data_B[0]));

	/* START DMA */
	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);
	__HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE);

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void run(void) {
	for (size_t i = 0; i < CHANNELS_LENGTH; i++) {
		led_write(data_A, data_B, sizeof(data_A)/sizeof(data_A[0]), i, channels[i]);
	}
}

void err(void) {
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}
