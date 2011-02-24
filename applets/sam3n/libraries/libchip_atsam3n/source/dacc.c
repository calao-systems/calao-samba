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

/** \addtogroup dacc_module Working with DACC
 * The DACC driver provides the interface to configure and use the DACC peripheral.\n
 *
 * The DACC(Digital-to-Analog Converter Controller) converts digital input to analog output.
 * The data to be converted are sent through a PDC channel. It offers an
 * analog outputs.
 *
 * To Enable a DACC conversion,the user has to follow these few steps:
 * <ul>
 * <li> Select an appropriate reference voltage on ADVREF   </li>
 * <li> Configure the DACC according to its requirements and special needs,which could be
        broken down into several parts:
 * -#   Enable DACC in free running mode by clearing TRGEN in DACC_MR;
 * -#   Configure Startup Time and Internal Trigger Period through setting STARTUP and CLKDIV fields
 *      in DACC_MR.
 * -#   Using PDC to transfer the converted data to DACC .
   </li>
 * </ul>
 *
 * For more accurate information, please look at the DACC section of the
 * Datasheet.
 *
 * Related files :\n
 * \ref DACC.c\n
 * \ref DACC.h\n
*/
/*@{*/
/*@}*/
/**
 * \file
 *
 * Implementation of Digital-to-Analog Converter Controller (DACC).
 *
 */
/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <chip.h>

#include <stdint.h>

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
  * \brief Initialize the DACC controller
  * \param pDACC Pointer to an DACC instance.
  * \param idDACC identifier of DAC peripheral
  * \param trgEn trigger mode, free running mode or external Hardware trigger
  * \param trgSel hardware trigger selection
  * \param dacEn  enable DAC
  * \param word transfer size,word or half word
  * \param startup value of the start up time (in DACCClock) (see datasheet)
  * \param clkDiv value of the DAC clock divider for internal trigger
*/
extern void DACC_Initialize( Dacc*    pDACC,
                             uint8_t  idDACC,
                             uint8_t  trgEn,
                             uint8_t  trgSel,
                             uint8_t  dacEn,
                             uint8_t  word,
                             uint32_t startup,
                             uint32_t clkDiv
                            )
{
    /* Enable peripheral clock*/
    PMC->PMC_PCER = 1 << idDACC;

    /*  Reset the controller */
    DACC_SoftReset(pDACC);

    /*  Write to the MR register */
    DACC_CfgModeReg( pDACC,
          ( trgEn    & DACC_MR_TRGEN )
        | ( trgSel   & DACC_MR_TRGSEL )
        | ( dacEn    & DACC_MR_DACEN )
        | ( word     & DACC_MR_WORD )
        | ( startup  & DACC_MR_STARTUP )
        | ( clkDiv   & DACC_MR_CLKDIV ) );
}

/**
  * \brief Write converted data through PDC channel
  * \param pDACC the pointer of DACC peripheral
  * \param pBuffer the destination buffer
  * \param size the size of the buffer
*/
extern uint32_t DACC_WriteBuffer( Dacc* pDACC, uint16_t *pwBuffer, uint32_t dwSize )
{

    /* Check if the first PDC bank is free*/
    if ( (pDACC->DACC_TCR == 0) && (pDACC->DACC_TNCR == 0) )
	{
        pDACC->DACC_TPR = (uint32_t)pwBuffer ;
        pDACC->DACC_TCR = dwSize ;
        pDACC->DACC_PTCR = DACC_PTCR_TXTEN ;

        return 1 ;
    }
    /* Check if the second PDC bank is free*/
    else
	{
	    if (pDACC->DACC_TNCR == 0)
	    {
            pDACC->DACC_TNPR = (uint32_t)pwBuffer ;
            pDACC->DACC_TNCR = dwSize ;

            return 1 ;
        }
        else
		{
            return 0 ;
		}
    }

}


