#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "stdio.h"

typedef struct Pad
{
    uint32_t rccAHB1Peripheral;
    GPIO_TypeDef* gpioPort;
    uint16_t gpioPin;
    uint8_t extiPortSource;
    uint8_t extiPinSource;
    uint32_t extiLine;
    uint8_t nvicIRQChannel;
} Pad;

const Pad const inputPads[] = {
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_2, EXTI_PortSourceGPIOE, EXTI_PinSource2, EXTI_Line2, EXTI2_IRQn },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_3, EXTI_PortSourceGPIOE, EXTI_PinSource3, EXTI_Line3, EXTI3_IRQn },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_4, EXTI_PortSourceGPIOE, EXTI_PinSource4, EXTI_Line4, EXTI4_IRQn },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_5, EXTI_PortSourceGPIOE, EXTI_PinSource5, EXTI_Line5, EXTI9_5_IRQn },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_6, EXTI_PortSourceGPIOE, EXTI_PinSource6, EXTI_Line6, EXTI9_5_IRQn },
};

const int const numOfInputPads = sizeof(inputPads) / sizeof(Pad);

const char* const inputButtonNames[16] = {
    NULL,
    NULL,
    "JoystickA",
    "JoystickB",
    "JoystickC",
    "JoystickD",
    "JoystickCtr",
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
};

void buttonInputHandler(uint32_t extiLine, size_t buttonIndex)
{
    if (EXTI_GetITStatus(extiLine) != RESET)
    {
        // clear the interrupt pending bit
        EXTI_ClearITPendingBit(extiLine);

        // print to console if it is a known button
        if (inputButtonNames[buttonIndex] != NULL)
        {
            printf("%s button pressed\n", inputButtonNames[buttonIndex]);
        }
    }
}

void EXTI2_IRQHandler(void)
{
    buttonInputHandler(EXTI_Line2, 2);
}

void EXTI3_IRQHandler(void)
{
    buttonInputHandler(EXTI_Line3, 3);
}

void EXTI4_IRQHandler(void)
{
    buttonInputHandler(EXTI_Line4, 4);
}

void EXTI9_5_IRQHandler(void)
{
    buttonInputHandler(EXTI_Line5, 5);
    buttonInputHandler(EXTI_Line6, 6);
}

void setupExampleDigitalInputs(void)
{
    // enable SYSCFG peripheral
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    for (int i = 0; i < numOfInputPads; i++)
    {
        // enable GPIO port associated with the pad
        RCC_AHB1PeriphClockCmd(inputPads[i].rccAHB1Peripheral, ENABLE);

        // setup pad as a digital input
        GPIO_InitTypeDef gpioInit;
        gpioInit.GPIO_Pin = inputPads[i].gpioPin;       // pins to set (in this case, only one)
        gpioInit.GPIO_Mode = GPIO_Mode_IN;              // pin mode (in this case, a digital input)
        gpioInit.GPIO_PuPd = GPIO_PuPd_UP;              // pin pull-up/down setup (in this case, pull-up)
        GPIO_Init(inputPads[i].gpioPort, &gpioInit);    // setup pad on the specified port

        // setup EXTI line configuration
        SYSCFG_EXTILineConfig(inputPads[i].extiPortSource, inputPads[i].extiPinSource);

        // configure external line interrupt
        EXTI_InitTypeDef extiInit;
        extiInit.EXTI_Line = inputPads[i].extiLine;             // EXTI line to use
        extiInit.EXTI_Mode = EXTI_Mode_Interrupt;               // setup as an interrupt
        extiInit.EXTI_Trigger = EXTI_Trigger_Falling;           // raise ISR on falling edge
        extiInit.EXTI_LineCmd = ENABLE;                         // set line interrupt enable flag
        EXTI_Init(&extiInit);                                   // configure line interrupt

        // enable EXTI interrupt
        NVIC_InitTypeDef nvicInit;
        nvicInit.NVIC_IRQChannel = inputPads[i].nvicIRQChannel; // IRQ channel
        nvicInit.NVIC_IRQChannelPreemptionPriority = 0x0F;      // lowest preemption priority
        nvicInit.NVIC_IRQChannelSubPriority = i;                // unique subpriority
        nvicInit.NVIC_IRQChannelCmd = ENABLE;                   // set interrupt enable flag
        NVIC_Init(&nvicInit);                                   // configure interrupt
    }
}
