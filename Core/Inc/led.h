#ifndef __LED_H
#define __LED_H

#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define L1_OFFSET 0
#define L2_OFFSET 3
#define L3_OFFSET 6

#define R_OFFSET 0
#define G_OFFSET 1
#define B_OFFSET 2

#define LR1_ADDR (BASE_ADDR + L1_OFFSET + R_OFFSET)
#define LG1_ADDR (BASE_ADDR + L1_OFFSET + G_OFFSET)
#define LB1_ADDR (BASE_ADDR + L1_OFFSET + B_OFFSET)

#define LR2_ADDR (BASE_ADDR + L2_OFFSET + R_OFFSET)
#define LG2_ADDR (BASE_ADDR + L2_OFFSET + G_OFFSET)
#define LB2_ADDR (BASE_ADDR + L2_OFFSET + B_OFFSET)

#define LR3_ADDR (BASE_ADDR + L3_OFFSET + R_OFFSET)
#define LG3_ADDR (BASE_ADDR + L3_OFFSET + G_OFFSET)
#define LB3_ADDR (BASE_ADDR + L3_OFFSET + B_OFFSET)

#define START_ADDR BASE_ADDR
#define END_ADDR LB3_ADDR

#define LR1 LR1_GPIO_Port, LR1_Pin
#define LG1 LG1_GPIO_Port, LG1_Pin
#define LB1 LB1_GPIO_Port, LB1_Pin

#define LR2 LR2_GPIO_Port, LR2_Pin
#define LG2 LG2_GPIO_Port, LG2_Pin
#define LB2 LB2_GPIO_Port, LB2_Pin

#define LR3 LR3_GPIO_Port, LR3_Pin
#define LG3 LG3_GPIO_Port, LG3_Pin
#define LB3 LB3_GPIO_Port, LB3_Pin

#define LED_ON(LED) HAL_GPIO_WritePin(LED, GPIO_PIN_RESET)
#define LED_OFF(LED) HAL_GPIO_WritePin(LED, GPIO_PIN_SET)

void led_pwm(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t value) {
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, value ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void led_write(size_t index, uint8_t value) {
    if (index == LR1_ADDR) { led_pwm(LR1, value); }
    if (index == LG1_ADDR) { led_pwm(LG1, value); }
    if (index == LB1_ADDR) { led_pwm(LB1, value); }

    if (index == LR2_ADDR) { led_pwm(LR2, value); }
    if (index == LG2_ADDR) { led_pwm(LG2, value); }
    if (index == LB2_ADDR) { led_pwm(LB2, value); }

    if (index == LR3_ADDR) { led_pwm(LR3, value); }
    if (index == LG3_ADDR) { led_pwm(LG3, value); }
    if (index == LB3_ADDR) { led_pwm(LB3, value); }
}

#ifdef __cplusplus
}
#endif

#endif /* __LED_H */
