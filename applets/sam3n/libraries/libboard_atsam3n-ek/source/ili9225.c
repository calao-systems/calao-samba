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
 * Implementation of ILI9225 driver.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdio.h>
#include <board.h>

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/* LCD Register Select pin definition */
const Pin pinLcdRs = PIN_LCD_RS;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Initialize SPI transfer with LCD.
 */
static void SpiInit(void)
{
    /* Configure Spi driver */
    SPI_Configure(SPI, ID_SPI, SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_MR_PCS);

    /* Configure Spi Chip select: SPI Mode 0 */
    SPI_ConfigureNPCS(SPI, BOARD_LCD_NPCS,
        SPI_CSR_NCPHA |
        SPI_CSR_BITS_8_BIT |
        SPI_DLYBS(BOARD_LCD_DLYBS, BOARD_MCK) |
        SPI_DLYBCT(BOARD_LCD_DLYBCT, BOARD_MCK) |
        SPI_SCBR(BOARD_LCD_SPCK, BOARD_MCK));

    /* Enables a SPI peripheral. */
    SPI_Enable(SPI);
}

/**
 * \brief Send command to LCD controller.
 *
 * \param cmd   command.
 */
static void WriteCmd(uint8_t cmd)
{
    /* Configure Spi Chip select: SPI Mode 0, 8bits */
    SPI_ConfigureNPCS(SPI, BOARD_LCD_NPCS,
        SPI_CSR_NCPHA |
        SPI_CSR_BITS_8_BIT |
        SPI_DLYBS(BOARD_LCD_DLYBS, BOARD_MCK) |
        SPI_DLYBCT(BOARD_LCD_DLYBCT, BOARD_MCK) |
        SPI_SCBR(BOARD_LCD_SPCK, BOARD_MCK));

    SPI_EnableCs(SPI, BOARD_LCD_NPCS);
    PIO_Clear(&pinLcdRs);
    SPI_Write(SPI, BOARD_LCD_NPCS, cmd);
    while (!SPI_IsFinished(SPI));
}

/**
 * \brief Send one data to LCD controller.
 *
 * \param data   data.
 */

static void WriteData(uint16_t data)
{
    /* Configure Spi Chip select: SPI Mode 0, 16bits */
    SPI_ConfigureNPCS(SPI, BOARD_LCD_NPCS,
        SPI_CSR_NCPHA |
        SPI_CSR_BITS_16_BIT |
        SPI_DLYBS(BOARD_LCD_DLYBS, BOARD_MCK) |
        SPI_DLYBCT(BOARD_LCD_DLYBCT, BOARD_MCK) |
        SPI_SCBR(BOARD_LCD_SPCK, BOARD_MCK));

    SPI_EnableCs(SPI, BOARD_LCD_NPCS);
    PIO_Set(&pinLcdRs);
    SPI_WriteBuffer(SPI, &data, sizeof(data)/2);
    while (!SPI_IsFinished(SPI));
}

/**
 * \brief Write mutiple data in buffer to LCD controller.
 *
 * \param pBuf  data buffer.
 * \param size  size in bytes
 */
static void WriteBuffer(void *pBuf, uint32_t size)
{
    /* Configure Spi Chip select: SPI Mode 0, 16bits */
    SPI_ConfigureNPCS(SPI, BOARD_LCD_NPCS,
        SPI_CSR_NCPHA |
        SPI_CSR_BITS_16_BIT |
        SPI_DLYBS(BOARD_LCD_DLYBS, BOARD_MCK) |
        SPI_DLYBCT(BOARD_LCD_DLYBCT, BOARD_MCK) |
        SPI_SCBR(BOARD_LCD_SPCK, BOARD_MCK));

    SPI_EnableCs(SPI, BOARD_LCD_NPCS);
    PIO_Set(&pinLcdRs);
    SPI_WriteBuffer(SPI, pBuf, size/2);
    while (!SPI_IsFinished(SPI));
}

/**
 * \brief Write data to LCD Register.
 *
 * \param reg   Register address.
 * \param data  Data to be written.
 */
static void WriteReg(uint8_t reg, uint16_t data)
{
    WriteCmd(reg);
    WriteData(data);
}

/*----------------------------------------------------------------------------
 *        Export functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Prepare to write GRAM data.
 */
extern void LCD_WriteRAM_Prepare(void)
{
    WriteCmd(0x22);
}

/**
 * \brief Write one pixel to LCD GRAM.
 *
 * \param color  RGB color.
 */
extern void LCD_WriteRAM(LcdColor_t color)
{
    WriteData(color);
}

/**
 * \brief Write multiple pixels in buffer to LCD GRAM.
 *
 * \param pBuf  color buffer of pixels.
 * \param size  size in bytes
 */
extern void LCD_WriteRAMBuffer(void *pBuf, uint32_t size)
{
    WriteBuffer(pBuf, size);
}

/**
 * \brief Initialize the LCD controller.
 */
extern void LCD_Initialize(void)
{
    const Pin pinsLcd[] = {PINS_LCD};
    const Pin pinLcdRst = PIN_LCD_RSTN;

    /* Initialize LCD Pins */
    PIO_Configure(pinsLcd, PIO_LISTSIZE(pinsLcd));

    /* Reset LCD module */
    PIO_Set(&pinLcdRst);
    Wait(2);
    PIO_Clear(&pinLcdRst);
    Wait(20);
    PIO_Set(&pinLcdRst);
    Wait(50);

    /* Initialize SPI interface for LCD */
    SpiInit();

    /* Turn off LCD */
    LCD_Off();

    /*======== LCD module initial code ========*/

    /* Start Initial Sequence */
    WriteReg(0x01, 0x011c);              /* Set SS ¡¢SM  GS and NL  bit */
    WriteReg(0x02, 0x0100);              /* Set 1 line inversion */
    WriteReg(0x03, 0x1030);              /* Entry Mode  set GRAM     write direction and BGR=1 */
    WriteReg(0x08, 0x0808);              /* Set BP and FP */
    WriteReg(0x0C, 0x0001);              /* RGB Input Interface Control:16-bit RGB interface */
    WriteReg(0x0F, 0x0A01);              /* set frame rate: 83Hz */
    WriteReg(0x20, BOARD_LCD_WIDTH);     /* set GRAM Address */
    WriteReg(0x21, BOARD_LCD_HEIGHT);    /* set GRAM Address */

    /* power on sequence */
    WriteReg(0x10, 0x0A00);              /* set asp DSTB,STB */
    WriteReg(0x11, 0x1038);              /* SET APON PON AON VCI1EN VC */
    Wait(50);
    WriteReg(0x12, 0x1121);              /* INTERNAL REFERENCE VOLTATE =VCI */
    WriteReg(0x13, 0x06CE);              /* SET GVDD */
    WriteReg(0x14, 0x676F);              /* SET VCOMH/VCOML VOLTAGE */

    /* SET GRAM AREA */
    WriteReg(0x30, 0x0000);
    WriteReg(0x31, 0x00DB);
    WriteReg(0x32, 0x0000);
    WriteReg(0x33, 0x0000);
    WriteReg(0x34, 0x00DB);
    WriteReg(0x35, 0x0000);
    WriteReg(0x36, BOARD_LCD_WIDTH);
    WriteReg(0x37, 0x0000);
    WriteReg(0x38, BOARD_LCD_HEIGHT);
    WriteReg(0x39, 0x0000);

    /* Set GAMMA CRUVE */
    WriteReg(0x50, 0x0000);
    WriteReg(0x51, 0x060A);
    WriteReg(0x52, 0x0D0A);
    WriteReg(0x53, 0x0303);
    WriteReg(0x54, 0x0A0D);
    WriteReg(0x55, 0x0A06);
    WriteReg(0x56, 0x0000);
    WriteReg(0x57, 0x0303);
    WriteReg(0x58, 0x0000);
    WriteReg(0x59, 0x0000);
}

/**
 * \brief Turn on the LCD.
 */
void LCD_On(void)
{
    WriteReg(0x07, 0x1017);
}

/**
 * \brief Turn off the LCD.
 */
void LCD_Off(void)
{
    WriteReg(0x07, 0x00);
}

/**
 * \brief Set cursor of LCD srceen.
 *
 * \param x  X-coordinate of upper-left corner on LCD.
 * \param y  Y-coordinate of upper-left corner on LCD.
 */
void LCD_SetCursor(uint16_t x, uint16_t y)
{
    WriteReg(0x20, x); /* column */
    WriteReg(0x21, y); /* row */
}

/**
 * \brief Set LCD display window.
 *
 * \param x      X-coordinate of upper-left corner on LCD.
 * \param y      Y-coordinate of upper-left corner on LCD.
 * \param width  Window width.
 * \param height Window height.
 */
void LCD_SetWindow( uint32_t x, uint32_t y, uint32_t width, uint32_t height )
{
    /* Set Horizontal Address End Position */
    WriteReg( 0x36, (uint16_t)x+width-1 ) ;

    /* Set Horizontal Address Start Position */
    WriteReg( 0x37, (uint16_t)x ) ;

    /* Set Vertical Address End Position */
    WriteReg( 0x38, (uint16_t)y+height-1 ) ;

    /* Set Vertical Address Start Position */
    WriteReg( 0x39, (uint16_t)y ) ;
}

