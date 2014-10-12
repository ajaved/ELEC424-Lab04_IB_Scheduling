#ifndef __SCHEDULING_H__
#define __SCHEDULING_H__

#include <stdio.h>
#include <stdbool.h>
#include "stm32f10x_conf.h"

#define LED_POL_POS 0
#define LED_POL_NEG 1

#define LED_GPIO_PERIF   RCC_APB2Periph_GPIOB
#define LED_GPIO_PORT    GPIOB
#define LED_GPIO_GREEN   GPIO_Pin_5
#define LED_POL_GREEN    LED_POL_NEG
#define LED_GPIO_RED     GPIO_Pin_4
#define LED_POL_RED      LED_POL_NEG

#define TIMER_PRESCALE 40000
#define TIMER_PERIOD 200
// #define TIMER_PERIOD 

typedef enum {LED_RED=0, LED_GREEN} led_t;

void InitializeTimer(void);
void ledToggle(led_t led);
void EnableTimerInterrupt(void);
void TIM2_IRQHandler(void);

#endif //__SCHEDULING_H__