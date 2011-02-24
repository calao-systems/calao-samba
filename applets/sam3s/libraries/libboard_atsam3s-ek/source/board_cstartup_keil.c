/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------
#include <board.h>

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------
/* The mask of VTOR register */
#define SCB_VTOR_MASK   0x3FFFFF80


//------------------------------------------------------------------------------
//         Types
//------------------------------------------------------------------------------
typedef union { IntFunc __fun; void * __ptr; } IrqVector;

//------------------------------------------------------------------------------
//         ProtoTypes
//------------------------------------------------------------------------------
extern int Image$$ARM_LIB_STACK$$ZI$$Limit;
extern int Image$$Vector_region$$Base;
extern int Image$$Vector_region$$Limit;

extern void __main(void);
void resetHandler( void );

//------------------------------------------------------------------------------
//         Variables
//------------------------------------------------------------------------------

#pragma arm section rodata="vectors"
const IrqVector __vector_table[] =
{
    (IntFunc)&Image$$ARM_LIB_STACK$$ZI$$Limit,
    resetHandler,

    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,             // Reserved
    SVC_Handler,
    DebugMon_Handler,
    0,                      // Reserved
    PendSV_Handler,
    SysTick_Handler,

    // Configurable interrupts
    SUPC_IrqHandler,    /*  0  SUPPLY CONTROLLER */
    RSTC_IrqHandler,    /*  1  RESET CONTROLLER */
    RTC_IrqHandler,     /*  2  REAL TIME CLOCK */
    RTT_IrqHandler,     /*  3  REAL TIME TIMER */
    WDT_IrqHandler,     /*  4  WATCHDOG TIMER */
    PMC_IrqHandler,     /*  5  PMC */
    EEFC_IrqHandler,    /*  6  EEFC */
    IrqHandlerNotUsed,  /*  7  Reserved */
    UART0_IrqHandler,   /*  8  UART0 */
    UART1_IrqHandler,   /*  9  UART1 */
    SMC_IrqHandler,     /*  10 SMC */
    PIOA_IrqHandler,    /*  11 Parallel IO Controller A */
    PIOB_IrqHandler,    /*  12 Parallel IO Controller B */
    PIOC_IrqHandler,    /*  13 Parallel IO Controller C */
    USART0_IrqHandler,  /*  14 USART 0 */
    USART1_IrqHandler,  /*  15 USART 1 */
    IrqHandlerNotUsed,  /*  16 Reserved */
    IrqHandlerNotUsed,  /*  17 Reserved */
    MCI_IrqHandler,     /*  18 MCI */
    TWI0_IrqHandler,    /*  19 TWI 0 */
    TWI1_IrqHandler,    /*  20 TWI 1 */
    SPI_IrqHandler,     /*  21 SPI */
    SSC_IrqHandler,     /*  22 SSC */
    TC0_IrqHandler,     /*  23 Timer Counter 0 */
    TC1_IrqHandler,     /*  24 Timer Counter 1 */
    TC2_IrqHandler,     /*  25 Timer Counter 2 */
    TC3_IrqHandler,     /*  26 Timer Counter 3 */
    TC4_IrqHandler,     /*  27 Timer Counter 4 */
    TC5_IrqHandler,     /*  28 Timer Counter 5 */
    ADC_IrqHandler,     /*  29 ADC controller */
    DAC_IrqHandler,     /*  30 DAC controller */
    PWM_IrqHandler,     /*  31 PWM */
    CRCCU_IrqHandler,   /*  32 CRC Calculation Unit */
    ACC_IrqHandler,     /*  33 Analog Comparator */
    USBD_IrqHandler,    /*  34 USB Device Port */
    IrqHandlerNotUsed   /*  35 not used */
};
#pragma arm section

//------------------------------------------------------------------------------
/// This is the code that gets called on processor reset. To initialize the
/// device.
//------------------------------------------------------------------------------
void resetHandler( void )
{
    unsigned int *pSrc = (unsigned int *)&Image$$Vector_region$$Base;

    // Low level Initialize
    LowLevelInit();

    SCB->VTOR = ((uint32_t)(pSrc)) & SCB_VTOR_MASK;

    // Enter C library entry point
    __main();

    /* Infinite loop */
    while(1);
}
