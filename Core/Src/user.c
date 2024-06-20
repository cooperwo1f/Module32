#include "user.h"
#include "stm32f1xx_hal.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* BASE ADDR OF CONTROLLER */
#define BASE_ADDR 0
#include "led.h"

/* TIMER HANDLERS */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* DMA HANDLERS */
extern DMA_HandleTypeDef hdma_tim2_up;
extern DMA_HandleTypeDef hdma_tim3_ch4_up;

/* ARRAY CONTAINING LED CHANNEL VALUES */
#define CHANNELS_LENGTH 256
uint16_t channels[CHANNELS_LENGTH];

/* VALUES TO BE CONTINUALLY TRANSFERRED VIA DMA */
uint32_t data_A[256];
uint32_t data_B[256];

void init(void) {
	/* CLEAR ALL ARRAYS */
	memset(&channels, 0, sizeof(channels));
	memset(&data_A, 0, sizeof(data_A));
	memset(&data_B, 0, sizeof(data_B));

	/* START TIMERS */
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim3);

	/* CONFIGURE DMA */
	HAL_DMA_Start(&hdma_tim2_up, (uint32_t)&(data_A[0]), (uint32_t)&(GPIOA->BSRR), sizeof(data_A)/sizeof(data_A[0]));
	HAL_DMA_Start(&hdma_tim3_ch4_up, (uint32_t)&(data_B[0]), (uint32_t)&(GPIOB->BSRR), sizeof(data_B)/sizeof(data_B[0]));

	/* START DMA */
	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);
	__HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE);

	/* SIGNAL PROGRAM RUNNING */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void run(void) {
	/* WRITE PWM VALUES FROM CHANNEL ARRAY */
	for (size_t i = 0; i < CHANNELS_LENGTH; i++) {
		led_write(data_A, data_B, sizeof(data_A)/sizeof(data_A[0]), i, channels[i]);
	}
}

void err(void) {
	/* CLEAR PROGRAM RUNNING SIGNAL */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}
