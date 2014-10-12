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

int main(void)
{
  SetSysClockToHSE();
  InitializeTimer();
  EnableTimerInterrupt();

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


