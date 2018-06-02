/*
 * The clock configuration assumes the following:
 *
 * For STM32F407:
 * - A low-speed external oscillator of 32.768kHz is fitted
 * - A high-speed external oscillator of 8MHz is fitted
 * - Maximum clock speed of 168MHz is used
 * - The HSE is used as the PLL clock source
 * - Using the PLL as the system clock source
 * - System clock is 168MHz (maximum for the micro)
 *   - PLL parameters are M=8, N=336, P=2 [(((8MHz/M)*N)/P)=168e6]
 * - AHB clock prescaler is 1 (168MHz - maximum for the micro)
 * - APB1 clock prescaler is 4 (42MHz - maximum for the micro)
 * - APB2 clock prescaler is 2 (84MHz - maximum for the micro)
 * - Input voltage is 3.3V
 * - Internal FLASH latency is 5 wait states
 * - Not using I2S PLL unit or 48MHz clock generator
 * - Enabling FLASH prefetch buffer
 * - Enabling FLASH data and instruction caches
 */

#include "stm32f4xx.h"

#define PLL_M                   8               //!< M parameter for the system clock PLL
#define PLL_N                   336             //!< N parameter for the system clock PLL
#define PLL_P                   2               //!< P parameter for the system clock PLL
#define PLL_Q                   7               //!< Q parameter for the system clock PLL (not used downstream)

uint32_t SystemCoreClock        = 168000000;    //!< System core clock speed (in hertz)

/*! \brief Perform early initialisation of the microcontroller system
 */
void SystemInit(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));    // full access for co-processors CP10 and CP11
#endif

    // reset the RCC unit
    RCC->CR |= RCC_CR_HSION;                        // enable HSI
    RCC->CFGR = 0x00000000;                         // set to default
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON |      // disable HSE, CSS and PLL
                 RCC_CR_PLLON);
    RCC->PLLCFGR = 0x24003010;                      // set to default value
    RCC->CR &= ~RCC_CR_HSEBYP;                      // disable HSE bypass
    RCC->CIR = 0x00000000;                          // disable all interrupts

    // setup the HSE
    RCC->CR |= RCC_CR_HSEON;                        // enable HSE
    while ((RCC->CR & RCC_CR_HSERDY) == 0)          // wait until HSE has stabilised
    {
    }

    // setup regulator voltage output for maximum SYSCLK
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;              // enable the power controller
    PWR->CR |= PWR_CR_VOS;                          // setup for Scale1 mode

    // setup APB and AHB clocks
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                // setup AHB clock for /1 prescaler
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;               // setup APB2 clock for /2 prescaler
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;               // setup APB1 clock for /4 prescaler

    // setup PLL
    RCC->PLLCFGR = PLL_M | (PLL_N << 6) |           // setup M/N/P/Q parameters
                   (((PLL_P >> 1) -1) << 16) |
                   (RCC_PLLCFGR_PLLSRC_HSE) |
                   (PLL_Q << 24);
    RCC->CR |= RCC_CR_PLLON;                        // enable PLL
    while ((RCC->CR & RCC_CR_PLLRDY) == 0)          // wait until PLL has stabilised
    {
    }

    // setup FLASH interface
    FLASH->ACR = FLASH_ACR_PRFTEN |                 // enable prefetch
                 FLASH_ACR_ICEN | FLASH_ACR_DCEN |  // enable instruction and data caches
                 FLASH_ACR_LATENCY_5WS;             // 5 wait state latency

    // setup PLL as system clock source
    RCC->CFGR &= ~RCC_CFGR_SW;                      // clear system clock switch bits
    RCC->CFGR |= RCC_CFGR_SW_PLL;                   // use PLL as system clock
    while ((RCC->CFGR & RCC_CFGR_SWS) !=            // wait until clock is switched to PLL
           RCC_CFGR_SWS_PLL);
    {
    }

    SCB->VTOR = 0x00000000;                         // vector table is located at 0x00000000
}
