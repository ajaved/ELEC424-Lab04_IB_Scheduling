#include "sys_clk_init.h"

// Errorstatus indicating whether or not HSE start up
ErrorStatus HSEStartUpStatus;

// Set system clock to external oscillator
void SetSysClockToHSE(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/    
  /* Enable HSE */    
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;  
  } while((HSEStatus == 0) && (StartUpCounter != HSEStartUp_TimeOut));


  // Check if the HSE started correctly
  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }  



  if (HSEStatus == (uint32_t)0x01)
  {
    /* Enable Prefetch Buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    /* Flash 2 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;    

    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
      
    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
    
    /* PCLK1 = HCLK/2 */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;

// #ifdef STM32F10X_CL
//     /* Configure PLLs ------------------------------------------------------*/
//     /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
//     /* PREDIV1 configuration: PREDIV1CLK = PLL2 / 5 = 8 MHz */
        
//     RCC->CFGR2 &= (uint32_t)~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL |
//                               RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);
//     RCC->CFGR2 |= (uint32_t)(RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 |
//                              RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV5);
  
//     /* Enable PLL2 */
//     RCC->CR |= RCC_CR_PLL2ON;
//     /* Wait till PLL2 is ready */
//     while((RCC->CR & RCC_CR_PLL2RDY) == 0)
//     {
//     }
    
   
//     /* PLL configuration: PLLCLK = PREDIV1 * 9 = 72 MHz */ 
//     RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
//     RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 | 
//                             RCC_CFGR_PLLMULL9); 
// #else    
    /*  PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |
                                        RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);
// #endif /* STM32F10X_CL */

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }
    
    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
    {
    }
  }
  else
  {   
  	// HSE didn't start up TODO: throw error?
  }
}


// // Set system clock to external oscillator
// void SetSysClockToHSE(void)
// {
//   // Reset RCC clock
//   RCC_DeInit();
//   // Turn HSE on
//   RCC_HSEConfig(RCC_HSE_ON);

//   /* Wait till HSE is ready */
//   HSEStartUpStatus = RCC_WaitForHSEStartUp();

//   if (HSEStartUpStatus == SUCCESS)
//   {
//     /* Select HSE as system clock source */
//     // RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);
//     RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

//     /* Wait till PLL is used as system clock source */
//     while (RCC_GetSYSCLKSource() != 0x04)
//     {
//     }


//   }
// }