#pragma once

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f4xx.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void Default_Handler(void);

#ifdef __cplusplus
}
#endif
