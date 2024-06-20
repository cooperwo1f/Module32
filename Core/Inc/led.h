#ifndef __LED_H
#define __LED_H

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

void led_pwm(uint32_t* buf, size_t len, uint16_t pin, size_t duty) {
    for (size_t i = 0; i < len; i++) {
        if (i > duty) {
            buf[i] &= (uint32_t)~(pin << 16);
            buf[i] |= (uint32_t)(pin);
        }

        else {
            buf[i] &= (uint32_t)~(pin);
            buf[i] |= (uint32_t)(pin << 16);
        }
    }
}

void led_write(uint32_t* buf_A, uint32_t* buf_B, size_t len, size_t index, size_t value) {
    if (index == LR1_ADDR) { led_pwm(buf_A, len, LR1_Pin, value); }
    if (index == LG1_ADDR) { led_pwm(buf_A, len, LG1_Pin, value); }
    if (index == LB1_ADDR) { led_pwm(buf_A, len, LB1_Pin, value); }

    if (index == LR2_ADDR) { led_pwm(buf_A, len, LR2_Pin, value); }
    if (index == LG2_ADDR) { led_pwm(buf_A, len, LG2_Pin, value); }
    if (index == LB2_ADDR) { led_pwm(buf_A, len, LB2_Pin, value); }

    if (index == LR3_ADDR) { led_pwm(buf_A, len, LR3_Pin, value); }
    if (index == LG3_ADDR) { led_pwm(buf_A, len, LG3_Pin, value); }
    if (index == LB3_ADDR) { led_pwm(buf_B, len, LB3_Pin, value); }
}

#ifdef __cplusplus
}
#endif

#endif /* __LED_H */
