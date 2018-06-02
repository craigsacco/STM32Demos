#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_rcc.h"

const uint32_t outputPeripherals[] = {
    RCC_AHB1Periph_GPIOH,
    RCC_AHB1Periph_GPIOI,
};

const int numOfOutputPeripherals = sizeof(outputPeripherals) / sizeof(uint32_t);

typedef struct Pad
{
    GPIO_TypeDef* port;
    uint16_t pad;
} Pad;

const Pad outputPads[] = {
    { GPIOH, GPIO_Pin_2 },
    { GPIOH, GPIO_Pin_3 },
    { GPIOI, GPIO_Pin_8 },
    { GPIOI, GPIO_Pin_10 },
};

const int numOfOutputPads = sizeof(outputPads) / sizeof(Pad);

void exampleDigitalOutputsTask(void * pvParameters)
{
    while (1)
    {
        for (int i = 0; i < numOfOutputPads; i++)
        {
            for (int n = 0; n < numOfOutputPads; n++)
            {
                GPIO_WriteBit(outputPads[n].port, outputPads[n].pad, i == n);
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        for (int i = 0; i < numOfOutputPads; i++)
        {
            for (int n = 0; n < numOfOutputPads; n++)
            {
                GPIO_WriteBit(outputPads[n].port, outputPads[n].pad, 1);
            }
            vTaskDelay(250 / portTICK_PERIOD_MS);
            for (int n = 0; n < numOfOutputPads; n++)
            {
                GPIO_WriteBit(outputPads[n].port, outputPads[n].pad, 0);
            }
            vTaskDelay(250 / portTICK_PERIOD_MS);
        }
    }
}


void exampleDigitalOutputs(void)
{
    // enable peripherals
    for (int i = 0; i < numOfOutputPeripherals; i++)
    {
        RCC_AHB1PeriphClockCmd(outputPeripherals[i], ENABLE);
    }

    // setup pads as outputs
    for (int i = 0; i < numOfOutputPads; i++)
    {
        GPIO_InitTypeDef init;
        init.GPIO_Pin = outputPads[i].pad;
        init.GPIO_Mode = GPIO_Mode_OUT;
        init.GPIO_Speed = GPIO_Low_Speed;
        init.GPIO_OType = GPIO_OType_PP;
        init.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(outputPads[i].port, &init);
    }

    TaskHandle_t xHandle = NULL;
    xTaskCreate(exampleDigitalOutputsTask,      // task function
                "ExampleDigitalOutputs",        // name of task
                128,                            // size of the tasks stack
                NULL,                           // task parameters
                configMAX_PRIORITIES / 2,       // task priority
                &xHandle);                      // task handle
}
