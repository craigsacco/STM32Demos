#include "stm32f4xx_it.h"

/*! \brief Interrupt handler for the NMI (non-maskable interrupt)
 */
void NMI_Handler(void)
{
}

/*! \brief Interrupt handler for the hardware fault exception
 */
void HardFault_Handler(void)
{
    // loop indefinitely
    while (1)
    {
    }
}

/*! \brief Interrupt handler for the memory management fault exception
 */
void MemManage_Handler(void)
{
    // loop indefinitely
    while (1)
    {
    }
}

/*! \brief Interrupt handler for the bus fault exception
 */
void BusFault_Handler(void)
{
    // loop indefinitely
    while (1)
    {
    }
}

/*! \brief Interrupt handler for the usage fault exception
 */
void UsageFault_Handler(void)
{
    // loop indefinitely
    while (1)
    {
    }
}

/*! \brief Interrupt handler for the debug monitor exception
 */
void DebugMon_Handler(void)
{
}

/*! \brief Default interrupt handler
 */
void Default_Handler(void)
{
    // loop indefinitely
    while (1)
    {
    }
}
