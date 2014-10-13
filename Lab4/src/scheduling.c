#include "scheduling.h"
#include "sys_clk_init.h"

// Port used by the LEDs
static GPIO_TypeDef* led_port[] = {
  [LED_GREEN] = LED_GPIO_PORT, 
  [LED_RED] = LED_GPIO_PORT,
};
// Pins used by the LEDs
static unsigned int led_pin[] = {
  [LED_GREEN] = LED_GPIO_GREEN, 
  [LED_RED]   = LED_GPIO_RED,
};
// Value used to toggle the led
bool ledValue = false;

// Motor variables
const int MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };
static bool isInit = false;

int main(void)
{
  SetSysClockToHSE();
  // InitializeTimer();
  // EnableTimerInterrupt();
  // Initialize motors
  motorsInit();
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);

  // Initialize LED clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | LED_GPIO_PERIF, ENABLE);

  // Initialize LED and turn it on
  GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = LED_GPIO_GREEN | LED_GPIO_RED;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Run infinite loop and allow interrupt to toggle LED
  for (;;){}
}

// Interrupt handler for when timer finishes
void TIM2_IRQHandler(void)
{
  // Reset timer
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  // Toggle the LED
  ledToggle(LED_GREEN);
}

// Register TIM2_IRQHandler as the callback when the custom timer
// finishes a cycle.
void EnableTimerInterrupt(void)
{
  // Register the TIM2_IRQHandler interrupt  
  NVIC_InitTypeDef nvicStructure;
  nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);
}

// Initialize custom timer
void InitializeTimer(void)
{
  // Enable the TIM2 clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_TimeBaseInitTypeDef timerInitStructure;
  timerInitStructure.TIM_Prescaler = TIMER_PRESCALE;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = TIMER_PERIOD;
  timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timerInitStructure.TIM_RepetitionCounter = 0;
  // Install the defined timer (with custom period/prescaler etc..)
  TIM_TimeBaseInit(TIM2, &timerInitStructure);
  // Enable TIM2 for the system
  TIM_Cmd(TIM2, ENABLE);
  // Initialize timer interrupt
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void ledToggle(led_t led)
{
    ledValue = !ledValue;
    
    if(ledValue)
      // Turn LED on
      GPIO_SetBits(led_port[led], led_pin[led]);
    else
      // Turn LED off
      GPIO_ResetBits(led_port[led], led_pin[led]); 
}

// Initialization. Will set all motors ratio to 0%
void motorsInit()
{
  if (isInit)
    return;

  // Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  // Enable gpio and the timer
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | MOTORS_GPIO_PERIF, ENABLE);
  RCC_APB1PeriphClockCmd(MOTORS_GPIO_TIM_PERIF | MOTORS_GPIO_TIM_M3_4_PERIF, ENABLE);

  // Configure the GPIO for the timer output
  GPIO_InitStructure.GPIO_Pin = (
                                 MOTORS_GPIO_M1 |
                                 MOTORS_GPIO_M2 |
                                 MOTORS_GPIO_M3 | 
                                 MOTORS_GPIO_M4
                                 );
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  GPIO_Init(MOTORS_GPIO_PORT, &GPIO_InitStructure);

  // Remap M2-4
  GPIO_PinRemapConfig(MOTORS_REMAP , ENABLE);

  // Timer configuration
  TIM_TimeBaseStructure.TIM_Period = MOTORS_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(MOTORS_GPIO_TIM_M1_2, &TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Period = MOTORS_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(MOTORS_GPIO_TIM_M3_4, &TIM_TimeBaseStructure);

  // PWM channels configuration (All identical!)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = MOTORS_POLARITY;

  TIM_OC3Init(MOTORS_GPIO_TIM_M3_4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(MOTORS_GPIO_TIM_M3_4, TIM_OCPreload_Enable);

  TIM_OC4Init(MOTORS_GPIO_TIM_M3_4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(MOTORS_GPIO_TIM_M3_4, TIM_OCPreload_Enable);

  TIM_OC3Init(MOTORS_GPIO_TIM_M1_2, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(MOTORS_GPIO_TIM_M1_2, TIM_OCPreload_Enable);

  TIM_OC4Init(MOTORS_GPIO_TIM_M1_2, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(MOTORS_GPIO_TIM_M1_2, TIM_OCPreload_Enable);

  // Enable the timer
  TIM_Cmd(MOTORS_GPIO_TIM_M1_2, ENABLE);
  TIM_Cmd(MOTORS_GPIO_TIM_M3_4, ENABLE);

  // Enable the timer PWM outputs
  TIM_CtrlPWMOutputs(MOTORS_GPIO_TIM_M1_2, ENABLE);
  TIM_CtrlPWMOutputs(MOTORS_GPIO_TIM_M3_4, ENABLE);

  // Halt timer during debug halt.
  DBGMCU_Config(MOTORS_GPIO_TIM_M1_2_DBG, ENABLE);
  DBGMCU_Config(MOTORS_GPIO_TIM_M3_4_DBG, ENABLE);
  
  isInit = true;
}


void motorsSetRatio(int id, uint16_t ratio)
{
  switch(id)
  {
    case MOTOR_M1:
      TIM_SetCompare4(MOTORS_GPIO_TIM_M1_2, C_16_TO_BITS(ratio));
      // TIM_DMAConfig(MOTORS_GPIO_TIM_M1_2, TIM_DMABase_CCR1, ratio);
      break;
    case MOTOR_M2:
      TIM_SetCompare3(MOTORS_GPIO_TIM_M1_2, C_16_TO_BITS(ratio));
      // TIM_DMAConfig(MOTORS_GPIO_TIM_M1_2, TIM_DMABase_CCR1, ratio);
      break;
    case MOTOR_M3:
      TIM_SetCompare4(MOTORS_GPIO_TIM_M3_4, C_16_TO_BITS(ratio));
      // TIM_DMAConfig(MOTORS_GPIO_TIM_M3_4, TIM_DMABase_CCR1, ratio);
      break;
    case MOTOR_M4:
      TIM_SetCompare3(MOTORS_GPIO_TIM_M3_4, C_16_TO_BITS(ratio));
      // TIM_DMAConfig(MOTORS_GPIO_TIM_M3_4, TIM_DMABase_CCR1, ratio);
      break;
  }
}
