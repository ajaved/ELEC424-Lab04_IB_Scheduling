#ifndef __SCHEDULING_H__
#define __SCHEDULING_H__

// #include <stdio.h>
#include <stdbool.h>
#include "stm32f10x_conf.h"

// Timer and LED defines
#define LED_POL_POS 0
#define LED_POL_NEG 1

#define LED_GPIO_PERIF   RCC_APB2Periph_GPIOB
#define LED_GPIO_PORT    GPIOB
#define LED_GPIO_GREEN   GPIO_Pin_5
#define LED_POL_GREEN    LED_POL_NEG
#define LED_GPIO_RED     GPIO_Pin_4
#define LED_POL_RED      LED_POL_NEG

// 40000
// 200
#define TIMER_PRESCALE 7200
#define TIMER_PERIOD 10000
// #define TIMER_PERIOD 

typedef enum {LED_RED=0, LED_GREEN} led_t;


// Motor defines
// #define BLMC_PERIOD 0.01
#define BLMC_PERIOD 0.005   // 5ms = 200Hz
// #define BLMC_PERIOD 0.000001

#define MOTORS_GPIO_TIM_PERIF     RCC_APB1Periph_TIM3
#define MOTORS_GPIO_TIM_M1_2      TIM3
#define MOTORS_GPIO_TIM_M1_2_DBG  DBGMCU_TIM3_STOP
#define MOTORS_REMAP              GPIO_PartialRemap_TIM3

#define MOTORS_GPIO_TIM_M3_4_PERIF  RCC_APB1Periph_TIM4
#define MOTORS_GPIO_TIM_M3_4        TIM4
#define MOTORS_GPIO_TIM_M3_4_DBG    DBGMCU_TIM4_STOP

#define MOTORS_GPIO_PERIF         RCC_APB2Periph_GPIOB
#define MOTORS_GPIO_PORT          GPIOB
#define MOTORS_GPIO_M1            GPIO_Pin_1 // T3_CH4
#define MOTORS_GPIO_M2            GPIO_Pin_0 // T3_CH3
#define MOTORS_GPIO_M3            GPIO_Pin_9 // T4_CH4
#define MOTORS_GPIO_M4            GPIO_Pin_8 // T4_CH3

#define MOTORS_PWM_PRESCALE_RAW   (uint32_t)((72000000/0xFFFF) * BLMC_PERIOD + 1) // +1 is to not end up above 0xFFFF in the end
#define MOTORS_PWM_CNT_FOR_PERIOD (uint32_t)(72000000 * BLMC_PERIOD / MOTORS_PWM_PRESCALE_RAW)
#define MOTORS_PWM_CNT_FOR_1MS    (uint32_t)(72000000 * 0.001 / MOTORS_PWM_PRESCALE_RAW)
// #define MOTORS_PWM_PERIOD         MOTORS_PWM_CNT_FOR_PERIOD
#define MOTORS_PWM_PERIOD 20000
#define MOTORS_PWM_BITS           11  // Only for compatibiliy
// #define MOTORS_PWM_PRESCALE       (uint16_t)(MOTORS_PWM_PRESCALE_RAW - 1)
#define MOTORS_PWM_PRESCALE 1
#define MOTORS_POLARITY           TIM_OCPolarity_Low



// #define MOTORS_PWM_BITS     9
// #define MOTORS_PWM_PERIOD   ((1<<MOTORS_PWM_BITS) - 1)
// #define MOTORS_PWM_PRESCALE 0
// #define MOTORS_POLARITY           TIM_OCPolarity_High


// Motors IDs define
#define MOTOR_M1  0
#define MOTOR_M2  1
#define MOTOR_M3  2
#define MOTOR_M4  3

// Test defines
#define MOTORS_TEST_RATIO         (uint16_t)(0.5*(1<<16))
#define MOTORS_TEST_ON_TIME_MS    10
#define MOTORS_TEST_DELAY_TIME_MS 50

#define C_BITS_TO_16(X) (0xFFFF * (X - MOTORS_PWM_CNT_FOR_1MS) / MOTORS_PWM_CNT_FOR_1MS)
#define C_16_TO_BITS(X) (MOTORS_PWM_CNT_FOR_1MS + ((X * MOTORS_PWM_CNT_FOR_1MS) / 0xFFFF))

// #define C_BITS_TO_16(X) ((X)<<(16-MOTORS_PWM_BITS))
// #define C_16_TO_BITS(X) ((X)>>(16-MOTORS_PWM_BITS)&((1<<MOTORS_PWM_BITS)-1))


// Function definitions
void InitializeTimer(void);
void ledToggle(led_t led);
void EnableTimerInterrupt(void);
void TIM2_IRQHandler(void);
void motorsInit(void);
void motorsSetRatio(int id, uint16_t ratio);

#endif //__SCHEDULING_H__