#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

/*! \brief FreeRTOS callback when the SysTick interrupt is raised
 */
void vApplicationTickHook(void)
{
}

/*! \brief FreeRTOS callback when a task stack overflow is detected
 */
void vApplicationStackOverflowHook(
    TaskHandle_t pxTask,    /**< [in] Handle of the task whose stack overflowed */
    char* pcTaskName        /**< [in] Name of the affected task  */
    )
{
	// ignore parameters
	(void)pcTaskName;
	(void)pxTask;

	// disable interrupts and loop indefinitely
	taskDISABLE_INTERRUPTS();
	while (1)
	{
	}
}

void vApplicationIdleHook(void)
{
}

void vApplicationMallocFailedHook(void)
{
    // disable interrupts and loop indefinitely
    taskDISABLE_INTERRUPTS();
    while (1)
    {
    }
}

void assert_failed(uint8_t* file, uint32_t line)
{
    // disable interrupts and loop indefinitely
    taskDISABLE_INTERRUPTS();
    while (1)
    {
    }
}

extern void setupExampleDigitalOutputs(void);
extern void setupExampleDigitalInputs(void);

int main(void)
{
    // setup examples
    setupExampleDigitalOutputs();
    setupExampleDigitalInputs();

    // start the task scheduler - should never return
    vTaskStartScheduler();
    return 0;
}
