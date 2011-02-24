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
 * \file
 *
 * Implementation of LCD driver, Include LCD initialization,
 * LCD on/off and LCD backlight control.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include <board.h>

#include <stdint.h>

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Initializes the LCD controller.
 * Configure SMC to access LCD controller at 64MHz MCK.
 */
void LCDD_Initialize(void)
{
    /* Initialize LCD controller */
    LCD_Initialize();

    /* Set LCD backlight */
    LCDD_SetBacklight(2);
}

/**
 * \brief Turn on the LCD.
 */
void LCDD_On(void)
{
    LCD_On();
}

/**
 * \brief Turn off the LCD.
 */
void LCDD_Off(void)
{
    LCD_Off();
}

/**
 * \brief Set the backlight of the LCD (AAT3193).
 *
 * \param level   Backlight brightness level [1..16], 1 means maximum brightness.
 */
void LCDD_SetBacklight (uint32_t level)
{
    uint32_t i;
    const Pin pPins[] = {PIN_LCD_BACKLIGTH};

    /* Ensure valid level */
    level = (level < 1) ? 1 : level;
    level = (level > 16) ? 16 : level;

    /* Enable pins */
    PIO_Configure(pPins, PIO_LISTSIZE(pPins));

    /* Switch off backlight */
    PIO_Clear(pPins);
    i = 600 * (BOARD_MCK / 1000000);    /* wait for at least 500us */
    while(i--);

    /* Set new backlight level */
    for (i = 0; i < level; i++) {
        PIO_Clear(pPins);
        PIO_Clear(pPins);
        PIO_Clear(pPins);

        PIO_Set(pPins);
        PIO_Set(pPins);
        PIO_Set(pPins);
    }
}
