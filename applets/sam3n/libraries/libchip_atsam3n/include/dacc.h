/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2010, Atmel Corporation
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

/**
 *  \file
 *
 *  \section Purpose
 *
 *  Interface for configuration the Analog-to-Digital Converter (DACC) peripheral.
 *
 *  \section Usage
 *
 *  -# Configurate the pins for DACC
 *  -# Initialize the DACC with DACC_Initialize().
 *  -# Send the converted data through a PDC channel with DACC_WriteBuffer()
 *  -# Wait the end of the conversion by polling status with DACC_GetStatus()
 *
*/
#ifndef _DACC_
#define _DACC_

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include <chip.h>
#include <stdint.h>
#include <assert.h>

/*------------------------------------------------------------------------------
 *         Definitions
 *------------------------------------------------------------------------------*/

#ifdef __cplusplus
 extern "C" {
#endif

/*------------------------------------------------------------------------------
 *         Macros function of register access
 *------------------------------------------------------------------------------*/
#define DACC_CfgModeReg(pDACC, mode)  { \
             (pDACC)->DACC_MR = (mode);\
        }

#define DACC_GetModeReg(pDACC)                ((pDACC)->DACC_MR)

#define DACC_SoftReset(pDACC)                 ((pDACC)->DACC_CR = DACC_CR_SWRST)

#define DACC_EnableIt(pDACC, mode)            {\
            assert( ((mode)&0xFFF00000)== 0 ) ;\
            (pDACC)->DACC_IER = (mode);\
        }

#define DACC_DisableIt(pDACC, mode)           {\
            assert( ((mode)&0xFFF00000)== 0 ) ;\
            (pDACC)->DACC_IDR = (mode);\
        }

#define DACC_EnableDataReadyIt(pDACC)         ((pDACC)->DACC_IER = AT91C_DACC_DRDY)

#define DACC_GetStatus(pDACC)                 ((pDACC)->DACC_ISR)

#define DACC_GetInterruptMaskStatus(pDACC)    ((pDACC)->DACC_IMR)


/*------------------------------------------------------------------------------
 *         Exported functions
 *------------------------------------------------------------------------------*/
extern void DACC_Initialize( Dacc*    pDACC,
                             uint8_t  idDACC,
                             uint8_t  trgEn,
                             uint8_t  trgSel,
                             uint8_t  dacEn,
                             uint8_t  word,
                             uint32_t startup,
                             uint32_t clkDiv
                            );


extern uint32_t DACC_WriteBuffer( Dacc* pDACC, uint16_t* pwBuffer, uint32_t dwSize ) ;

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _DACC_ */
