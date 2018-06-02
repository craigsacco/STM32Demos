#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_rcc.h"

typedef struct Pad
{
    uint32_t rccAHB1Peripheral;
    GPIO_TypeDef* gpioPort;
    uint16_t gpioPin;
} Pad;

const Pad const outputPads[] = {
    { RCC_AHB1Periph_GPIOH, GPIOH, GPIO_Pin_2 },
    { RCC_AHB1Periph_GPIOH, GPIOH, GPIO_Pin_3 },
    { RCC_AHB1Periph_GPIOI, GPIOI, GPIO_Pin_8 },
    { RCC_AHB1Periph_GPIOI, GPIOI, GPIO_Pin_10 },
};

const int const numOfOutputPads = sizeof(outputPads) / sizeof(Pad);

void exampleDigitalOutputsTask(void * pvParameters)
{
    while (1)
    {
        for (int i = 0; i < numOfOutputPads; i++)
        {
            for (int n = 0; n < numOfOutputPads; n++)
            {
                GPIO_WriteBit(outputPads[n].gpioPort, outputPads[n].gpioPin, i == n);
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        for (int i = 0; i < numOfOutputPads; i++)
        {
            for (int n = 0; n < numOfOutputPads; n++)
            {
                GPIO_WriteBit(outputPads[n].gpioPort, outputPads[n].gpioPin, 1);
            }
            vTaskDelay(250 / portTICK_PERIOD_MS);
            for (int n = 0; n < numOfOutputPads; n++)
            {
                GPIO_WriteBit(outputPads[n].gpioPort, outputPads[n].gpioPin, 0);
            }
            vTaskDelay(250 / portTICK_PERIOD_MS);
        }
    }
}

void setupExampleDigitalOutputs(void)
{
    for (int i = 0; i < numOfOutputPads; i++)
    {
        // enable GPIO port associated with the pad
        RCC_AHB1PeriphClockCmd(outputPads[i].rccAHB1Peripheral, ENABLE);

        // setup pad as a digital input
        GPIO_InitTypeDef init;
        init.GPIO_Pin = outputPads[i].gpioPin;      // pins to set (in this case, only one)
        init.GPIO_Mode = GPIO_Mode_OUT;             // pin mode (in this case, a digital output)
        init.GPIO_Speed = GPIO_Low_Speed;           // pin output speed (in this case, lowest speed)
        init.GPIO_OType = GPIO_OType_PP;            // pin output type (in this case, use both high-side and low-side FETs)
        init.GPIO_PuPd = GPIO_PuPd_NOPULL;          // pin pull-up/down setup (in this case, floating)
        GPIO_Init(outputPads[i].gpioPort, &init);   // initialise pad on the specified port
    }

    // create threads that cycle LEDs associated with the outputs
    TaskHandle_t xHandle = NULL;
    xTaskCreate(exampleDigitalOutputsTask,      // task function
                "ExampleDigitalOutputs",        // name of task
                128,                            // size of the tasks stack
                NULL,                           // task parameters
                configMAX_PRIORITIES / 2,       // task priority
                &xHandle);                      // task handle
}
