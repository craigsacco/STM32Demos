	.syntax unified
	.cpu cortex-m4
	.fpu fpv4-sp-d16
	.thumb

	.global g_pfnVectors
	.global Default_Handler

	// section addresses defined in the linker script
	.word _sidata	// start address for initialisation values of the .data section
	.word _sdata	// start address for the .data section
	.word _edata	// end address for the .data section
	.word _sbss	// start address for the .bss section
	.word _ebss	// end address for the .bss section

	// reset handler
	.section .text.Reset_Handler
	.weak Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:				// reset handler start function
  	ldr sp, =_estack		// load end-of-stack address to stack pointer

	// copy initialisation data from FLASH to RAM
	// R0 = start address of .data section
    // R1 = relative offset from start of .data section
	// R2 = address inside of .data section
	// R3 = temporary
StartCopyDataInit:			// start copying initialisation data from FLASH to RAM
	movs r1, #0				// initialise offset to 0
	b LoopCopyDataInit		// jump to LoopCopyDataInit
CopyDataInit:				// copies a word of data from FLASH to RAM
							// R3 = word to copy
	ldr r3, =_sidata		// temporarily store the starting source address for initialisation values into R3
	ldr r3, [r3, r1]		// load word from address in R3+R1 into R3
	str r3, [r0, r1]		// store word in R0+R1 into address of R3
	adds r1, r1, #4			// increment offset by 4
LoopCopyDataInit:			// check if all initialisation values have been copied
							// R3 = end address of .data section
	ldr r0, =_sdata			// load start address of .data section
	ldr r3, =_edata			// load end address of .data section
	adds r2, r0, r1			// calculate address inside of .data section
	cmp r2, r3				// compare absolute offset with end address
	bcc CopyDataInit		// if not equal, jump to CopyDataInit for copying the next word

	// zero-fill the .bss section in RAM
	// R2 = address inside of .bss section
	// R3 = temporary
StartFillZerobss:			// start zero-filling the .bss section
	ldr r2, =_sbss			// set starting address
	b LoopFillZerobss		// jump to LoopFillZerobss
FillZerobss:				// zero-fill a word in RAM
							// R3 = fill pattern
	movs r3, #0				// load zero as a fill pattern
	str r3, [r2], #4		// store fill pattern into memort address, then post-increment the address by 4
LoopFillZerobss:			// check if all of the section has been filled
							// R3 = end address of .bss section
	ldr r3, = _ebss			// load end address of .bss section
	cmp r2, r3				// compare address with end address
	bcc FillZerobss			// if not equal, jump to FillZerobss for filling the next word

	// start system initialisation
StartSystemInit:
	bl SystemInit			// branch (with return) to SystemInit (see system_stm32f4xx.c)

	// call static constructors
StartInitArray:
	bl __libc_init_array	// branch (with return) to __libc_init_array (part of newlib)

    // call the main method
StartMainMethod:
  	bl main					// branch (with return) to the main method (see main.c)
  	bx lr
	.size Reset_Handler, .-Reset_Handler

	// vector table
	.section .isr_vector,"a",%progbits	// store vector table into the .isr_vector section
	.type g_pfnVectors, %object
g_pfnVectors:
	.word _estack							// address of bottom of the stack
	// core exception handlers (1-15)
	.word Reset_Handler						// address of Reset Handler
	.word NMI_Handler						// address of NMI Handler
	.word HardFault_Handler					// address of Hard Fault Handler
	.word MemManage_Handler					// address of Memory Management Fault Handler
	.word BusFault_Handler					// address of Bus Fault Handler
	.word UsageFault_Handler				// address of Usage Fault Handler
	.word 0
	.word 0
	.word 0
	.word 0
	.word SVC_Handler						// address of Service Call Handler
	.word DebugMon_Handler					// address of Debug Monitor Handler
	.word 0
	.word PendSV_Handler					// address of PendingSV Handler
	.word SysTick_Handler					// address of SysTick Handler
	// external interrupts (1-10)
	.word WWDG_IRQHandler					// address of Window WatchDog IRQ Handler
	.word PVD_IRQHandler					// address of PVD IRQ Handler
	.word TAMP_STAMP_IRQHandler				// address of Tamper and TimeStamps IRQ Handler
	.word RTC_WKUP_IRQHandler				// address of RTC Wakeup IRQ Handler
	.word FLASH_IRQHandler					// address of FLASH IRQ Handler
	.word RCC_IRQHandler					// address of RCC IRQ Handler
	.word EXTI0_IRQHandler					// address of EXTI Line0 IRQ Handler
	.word EXTI1_IRQHandler					// address of EXTI Line1 IRQ Handler
	.word EXTI2_IRQHandler					// address of EXTI Line2 IRQ Handler
	.word EXTI3_IRQHandler					// address of EXTI Line3 IRQ Handler
	// external interrupts (11-20)
	.word EXTI4_IRQHandler					// address of EXTI Line4 IRQ Handler
	.word DMA1_Stream0_IRQHandler			// address of DMA1 Stream 0 IRQ Handler
	.word DMA1_Stream1_IRQHandler			// address of DMA1 Stream 1 IRQ Handler
	.word DMA1_Stream2_IRQHandler			// address of DMA1 Stream 2 IRQ Handler
	.word DMA1_Stream3_IRQHandler			// address of DMA1 Stream 3 IRQ Handler
	.word DMA1_Stream4_IRQHandler			// address of DMA1 Stream 4 IRQ Handler
	.word DMA1_Stream5_IRQHandler			// address of DMA1 Stream 5 IRQ Handler
	.word DMA1_Stream6_IRQHandler			// address of DMA1 Stream 6 IRQ Handler
	.word ADC_IRQHandler					// address of ADC1/2/3 IRQ Handler
	.word CAN1_TX_IRQHandler				// address of CAN1 TX IRQ Handler
	// external interrupts (21-30)
	.word CAN1_RX0_IRQHandler				// address of CAN1 RX0 IRQ Handler
	.word CAN1_RX1_IRQHandler				// address of CAN1 RX1 IRQ Handler
	.word CAN1_SCE_IRQHandler				// address of CAN1 SCE IRQ Handler
	.word EXTI9_5_IRQHandler				// address of External Line[9:5] IRQ Handler
	.word TIM1_BRK_TIM9_IRQHandler			// address of TIM1 Break and TIM9 IRQ Handler
	.word TIM1_UP_TIM10_IRQHandler			// address of TIM1 Update and TIM10 IRQ Handler
	.word TIM1_TRG_COM_TIM11_IRQHandler		// address of TIM1 Trigger and Commutation and TIM11 IRQ Handler
	.word TIM1_CC_IRQHandler				// address of TIM1 Capture Compare IRQ Handler
	.word TIM2_IRQHandler					// address of TIM2 IRQ Handler
	.word TIM3_IRQHandler					// address of TIM3 IRQ Handler
	// external interrupts (31-40)
	.word TIM4_IRQHandler					// address of TIM4 IRQ Handler
	.word I2C1_EV_IRQHandler				// address of I2C1 Event IRQ Handler
	.word I2C1_ER_IRQHandler				// address of I2C1 Error IRQ Handler
	.word I2C2_EV_IRQHandler				// address of I2C2 Event IRQ Handler
	.word I2C2_ER_IRQHandler				// address of I2C2 Error IRQ Handler
	.word SPI1_IRQHandler					// address of SPI1 IRQ Handler
	.word SPI2_IRQHandler					// address of SPI2 IRQ Handler
	.word USART1_IRQHandler					// address of USART1 IRQ Handler
	.word USART2_IRQHandler					// address of USART2 IRQ Handler
	.word USART3_IRQHandler					// address of USART3 IRQ Handler
	// external interrupts (41-50)
	.word EXTI15_10_IRQHandler				// address of External Line[15:10] IRQ Handler
	.word RTC_Alarm_IRQHandler				// address of RTC Alarm IRQ Handler
	.word OTG_FS_WKUP_IRQHandler			// address of USB OTG FS Wakeup IRQ Handler
	.word TIM8_BRK_TIM12_IRQHandler			// address of TIM8 Break and TIM12 IRQ Handler
	.word TIM8_UP_TIM13_IRQHandler			// address of TIM8 Update and TIM13 IRQ Handler
	.word TIM8_TRG_COM_TIM14_IRQHandler		// address of TIM8 Trigger and Commutation and TIM14 IRQ Handler
	.word TIM8_CC_IRQHandler				// address of TIM8 Capture Compare IRQ Handler
	.word DMA1_Stream7_IRQHandler			// address of DMA1 Stream7 IRQ Handler
	.word FSMC_IRQHandler					// address of FSMC IRQ Handler
	.word SDIO_IRQHandler					// address of SDIO IRQ Handler
	// external interrupts (51-60)
	.word TIM5_IRQHandler					// address of TIM5 IRQ Handler
	.word SPI3_IRQHandler					// address of SPI3 IRQ Handler
	.word UART4_IRQHandler					// address of UART4 IRQ Handler
	.word UART5_IRQHandler					// address of UART5 IRQ Handler
	.word TIM6_DAC_IRQHandler				// address of TIM6 and DAC IRQ Handler
	.word TIM7_IRQHandler					// address of TIM7 IRQ Handler
	.word DMA2_Stream0_IRQHandler			// address of DMA2 Stream 0 IRQ Handler
	.word DMA2_Stream1_IRQHandler			// address of DMA2 Stream 1 IRQ Handler
	.word DMA2_Stream2_IRQHandler			// address of DMA2 Stream 2 IRQ Handler
	.word DMA2_Stream3_IRQHandler			// address of DMA2 Stream 3 IRQ Handler
	// external interrupts (61-70)
	.word DMA2_Stream4_IRQHandler			// address of DMA2 Stream 4 IRQ Handler
	.word ETH_IRQHandler					// address of Ethernet IRQ Handler
	.word ETH_WKUP_IRQHandler				// address of Ethernet Wakeup IRQ Handler
	.word CAN2_TX_IRQHandler				// address of CAN2 TX IRQ Handler
	.word CAN2_RX0_IRQHandler				// address of CAN2 RX0 IRQ Handler
	.word CAN2_RX1_IRQHandler				// address of CAN2 RX1 IRQ Handler
	.word CAN2_SCE_IRQHandler				// address of CAN2 SCE IRQ Handler
	.word OTG_FS_IRQHandler					// address of USB OTG FS IRQ Handler
	.word DMA2_Stream5_IRQHandler			// address of DMA2 Stream 5 IRQ Handler
	.word DMA2_Stream6_IRQHandler			// address of DMA2 Stream 6 IRQ Handler
	// external interrupts (71-80)
	.word DMA2_Stream7_IRQHandler			// address of DMA2 Stream 7 IRQ Handler
	.word USART6_IRQHandler					// address of USART6 IRQ Handler
	.word I2C3_EV_IRQHandler				// address of I2C3 Event IRQ Handler
	.word I2C3_ER_IRQHandler				// address of I2C3 Error IRQ Handler
	.word OTG_HS_EP1_OUT_IRQHandler			// address of USB OTG HS End Point 1 Out IRQ Handler
	.word OTG_HS_EP1_IN_IRQHandler			// address of USB OTG HS End Point 1 In IRQ Handler
	.word OTG_HS_WKUP_IRQHandler			// address of USB OTG HS Wakeup IRQ Handler
	.word OTG_HS_IRQHandler					// address of USB OTG HS IRQ Handler
	.word DCMI_IRQHandler					// address of DCMI IRQ Handler
	.word CRYP_IRQHandler					// address of CRYP Crypto IRQ Handler
	// external interrupts (81-82)
	.word HASH_RNG_IRQHandler				// address of Hash/RNG IRQ Handler
	.word FPU_IRQHandler					// address of FPU IRQ Handler
	.size g_pfnVectors, .-g_pfnVectors
                        
	// weak aliasing of external interrupt handlers
	.weak      WWDG_IRQHandler
	.thumb_set WWDG_IRQHandler,Default_Handler
	.weak      PVD_IRQHandler      
	.thumb_set PVD_IRQHandler,Default_Handler
	.weak      TAMP_STAMP_IRQHandler            
	.thumb_set TAMP_STAMP_IRQHandler,Default_Handler
	.weak      RTC_WKUP_IRQHandler                  
	.thumb_set RTC_WKUP_IRQHandler,Default_Handler
	.weak      FLASH_IRQHandler         
	.thumb_set FLASH_IRQHandler,Default_Handler
	.weak      RCC_IRQHandler      
	.thumb_set RCC_IRQHandler,Default_Handler
	.weak      EXTI0_IRQHandler         
	.thumb_set EXTI0_IRQHandler,Default_Handler
	.weak      EXTI1_IRQHandler         
	.thumb_set EXTI1_IRQHandler,Default_Handler
	//.weak      EXTI2_IRQHandler
	//.thumb_set EXTI2_IRQHandler,Default_Handler
	//.weak      EXTI3_IRQHandler
	//.thumb_set EXTI3_IRQHandler,Default_Handler
	//.weak      EXTI4_IRQHandler
	//.thumb_set EXTI4_IRQHandler,Default_Handler
	.weak      DMA1_Stream0_IRQHandler               
	.thumb_set DMA1_Stream0_IRQHandler,Default_Handler
	.weak      DMA1_Stream1_IRQHandler               
	.thumb_set DMA1_Stream1_IRQHandler,Default_Handler
	.weak      DMA1_Stream2_IRQHandler               
	.thumb_set DMA1_Stream2_IRQHandler,Default_Handler
	.weak      DMA1_Stream3_IRQHandler               
	.thumb_set DMA1_Stream3_IRQHandler,Default_Handler 
	.weak      DMA1_Stream4_IRQHandler              
	.thumb_set DMA1_Stream4_IRQHandler,Default_Handler
	.weak      DMA1_Stream5_IRQHandler               
	.thumb_set DMA1_Stream5_IRQHandler,Default_Handler
	.weak      DMA1_Stream6_IRQHandler               
	.thumb_set DMA1_Stream6_IRQHandler,Default_Handler
	.weak      ADC_IRQHandler      
	.thumb_set ADC_IRQHandler,Default_Handler
	.weak      CAN1_TX_IRQHandler   
	.thumb_set CAN1_TX_IRQHandler,Default_Handler
	.weak      CAN1_RX0_IRQHandler                  
	.thumb_set CAN1_RX0_IRQHandler,Default_Handler
	.weak      CAN1_RX1_IRQHandler                  
	.thumb_set CAN1_RX1_IRQHandler,Default_Handler
	.weak      CAN1_SCE_IRQHandler                  
	.thumb_set CAN1_SCE_IRQHandler,Default_Handler
	//.weak      EXTI9_5_IRQHandler
	//.thumb_set EXTI9_5_IRQHandler,Default_Handler
	.weak      TIM1_BRK_TIM9_IRQHandler            
	.thumb_set TIM1_BRK_TIM9_IRQHandler,Default_Handler
	.weak      TIM1_UP_TIM10_IRQHandler            
	.thumb_set TIM1_UP_TIM10_IRQHandler,Default_Handler
	.weak      TIM1_TRG_COM_TIM11_IRQHandler      
	.thumb_set TIM1_TRG_COM_TIM11_IRQHandler,Default_Handler
	.weak      TIM1_CC_IRQHandler   
	.thumb_set TIM1_CC_IRQHandler,Default_Handler
	.weak      TIM2_IRQHandler            
	.thumb_set TIM2_IRQHandler,Default_Handler
	.weak      TIM3_IRQHandler            
	.thumb_set TIM3_IRQHandler,Default_Handler
	.weak      TIM4_IRQHandler            
	.thumb_set TIM4_IRQHandler,Default_Handler
	.weak      I2C1_EV_IRQHandler   
	.thumb_set I2C1_EV_IRQHandler,Default_Handler
	.weak      I2C1_ER_IRQHandler   
	.thumb_set I2C1_ER_IRQHandler,Default_Handler
	.weak      I2C2_EV_IRQHandler   
	.thumb_set I2C2_EV_IRQHandler,Default_Handler
	.weak      I2C2_ER_IRQHandler   
	.thumb_set I2C2_ER_IRQHandler,Default_Handler
	.weak      SPI1_IRQHandler            
	.thumb_set SPI1_IRQHandler,Default_Handler
	.weak      SPI2_IRQHandler            
	.thumb_set SPI2_IRQHandler,Default_Handler
	.weak      USART1_IRQHandler      
	.thumb_set USART1_IRQHandler,Default_Handler
	.weak      USART2_IRQHandler      
	.thumb_set USART2_IRQHandler,Default_Handler
	.weak      USART3_IRQHandler      
	.thumb_set USART3_IRQHandler,Default_Handler
	.weak      EXTI15_10_IRQHandler               
	.thumb_set EXTI15_10_IRQHandler,Default_Handler
	.weak      RTC_Alarm_IRQHandler               
	.thumb_set RTC_Alarm_IRQHandler,Default_Handler
	.weak      OTG_FS_WKUP_IRQHandler         
	.thumb_set OTG_FS_WKUP_IRQHandler,Default_Handler
	.weak      TIM8_BRK_TIM12_IRQHandler         
	.thumb_set TIM8_BRK_TIM12_IRQHandler,Default_Handler
	.weak      TIM8_UP_TIM13_IRQHandler            
	.thumb_set TIM8_UP_TIM13_IRQHandler,Default_Handler
	.weak      TIM8_TRG_COM_TIM14_IRQHandler      
	.thumb_set TIM8_TRG_COM_TIM14_IRQHandler,Default_Handler
	.weak      TIM8_CC_IRQHandler   
	.thumb_set TIM8_CC_IRQHandler,Default_Handler
	.weak      DMA1_Stream7_IRQHandler               
	.thumb_set DMA1_Stream7_IRQHandler,Default_Handler
	.weak      FSMC_IRQHandler            
	.thumb_set FSMC_IRQHandler,Default_Handler
	.weak      SDIO_IRQHandler            
	.thumb_set SDIO_IRQHandler,Default_Handler
	.weak      TIM5_IRQHandler            
	.thumb_set TIM5_IRQHandler,Default_Handler
	.weak      SPI3_IRQHandler            
	.thumb_set SPI3_IRQHandler,Default_Handler
	.weak      UART4_IRQHandler         
	.thumb_set UART4_IRQHandler,Default_Handler
	.weak      UART5_IRQHandler         
	.thumb_set UART5_IRQHandler,Default_Handler
	.weak      TIM6_DAC_IRQHandler                  
	.thumb_set TIM6_DAC_IRQHandler,Default_Handler
	.weak      TIM7_IRQHandler            
	.thumb_set TIM7_IRQHandler,Default_Handler
	.weak      DMA2_Stream0_IRQHandler               
	.thumb_set DMA2_Stream0_IRQHandler,Default_Handler
	.weak      DMA2_Stream1_IRQHandler               
	.thumb_set DMA2_Stream1_IRQHandler,Default_Handler
	.weak      DMA2_Stream2_IRQHandler               
	.thumb_set DMA2_Stream2_IRQHandler,Default_Handler
	.weak      DMA2_Stream3_IRQHandler               
	.thumb_set DMA2_Stream3_IRQHandler,Default_Handler
	.weak      DMA2_Stream4_IRQHandler               
	.thumb_set DMA2_Stream4_IRQHandler,Default_Handler
	.weak      ETH_IRQHandler      
	.thumb_set ETH_IRQHandler,Default_Handler
	.weak      ETH_WKUP_IRQHandler                  
	.thumb_set ETH_WKUP_IRQHandler,Default_Handler
	.weak      CAN2_TX_IRQHandler   
	.thumb_set CAN2_TX_IRQHandler,Default_Handler
	.weak      CAN2_RX0_IRQHandler                  
	.thumb_set CAN2_RX0_IRQHandler,Default_Handler
	.weak      CAN2_RX1_IRQHandler                  
	.thumb_set CAN2_RX1_IRQHandler,Default_Handler
	.weak      CAN2_SCE_IRQHandler                  
	.thumb_set CAN2_SCE_IRQHandler,Default_Handler
	.weak      OTG_FS_IRQHandler      
	.thumb_set OTG_FS_IRQHandler,Default_Handler
	.weak      DMA2_Stream5_IRQHandler               
	.thumb_set DMA2_Stream5_IRQHandler,Default_Handler
	.weak      DMA2_Stream6_IRQHandler               
	.thumb_set DMA2_Stream6_IRQHandler,Default_Handler
	.weak      DMA2_Stream7_IRQHandler               
	.thumb_set DMA2_Stream7_IRQHandler,Default_Handler
	.weak      USART6_IRQHandler      
	.thumb_set USART6_IRQHandler,Default_Handler
	.weak      I2C3_EV_IRQHandler   
	.thumb_set I2C3_EV_IRQHandler,Default_Handler
	.weak      I2C3_ER_IRQHandler   
	.thumb_set I2C3_ER_IRQHandler,Default_Handler
	.weak      OTG_HS_EP1_OUT_IRQHandler         
	.thumb_set OTG_HS_EP1_OUT_IRQHandler,Default_Handler
	.weak      OTG_HS_EP1_IN_IRQHandler            
	.thumb_set OTG_HS_EP1_IN_IRQHandler,Default_Handler
	.weak      OTG_HS_WKUP_IRQHandler         
	.thumb_set OTG_HS_WKUP_IRQHandler,Default_Handler
	.weak      OTG_HS_IRQHandler      
	.thumb_set OTG_HS_IRQHandler,Default_Handler
	.weak      DCMI_IRQHandler            
	.thumb_set DCMI_IRQHandler,Default_Handler
	.weak      CRYP_IRQHandler            
	.thumb_set CRYP_IRQHandler,Default_Handler
	.weak      HASH_RNG_IRQHandler                  
	.thumb_set HASH_RNG_IRQHandler,Default_Handler   
	.weak      FPU_IRQHandler                  
	.thumb_set FPU_IRQHandler,Default_Handler  
