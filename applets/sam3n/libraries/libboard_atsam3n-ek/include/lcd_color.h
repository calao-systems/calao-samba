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

#ifndef LCD_COLOR_H
#define LCD_COLOR_H

/**
 * \file
 *
 * RGB 16-bits color table definition.
 *
 */

/*----------------------------------------------------------------------------
 *        Types
 *----------------------------------------------------------------------------*/
#include <stdint.h>

/*----------------------------------------------------------------------------
 *        Types
 *----------------------------------------------------------------------------*/

/** LCD color data type */
typedef uint16_t LcdColor_t;

/*----------------------------------------------------------------------------
 *        Macros
 *----------------------------------------------------------------------------*/

/** Convert 24-bits color to 16-bits color */
#define RGB24ToRGB16(color) (((color >> 8) & 0xF800) | \
    ((color >> 5) & 0x7E0) | \
    ((color >> 3) & 0x1F))

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/*
 * RGB 16 Bpp
 * RGB 565
 * R7R6R5R4R3 G7G6G5G4G3G2 B7B6B5B4B3
 */
#define COLOR_BLACK          RGB24ToRGB16(0x000000)
#define COLOR_WHITE          RGB24ToRGB16(0xFFFFFF)

#define COLOR_BLUE           RGB24ToRGB16(0x0000FF)
#define COLOR_GREEN          RGB24ToRGB16(0x00FF00)
#define COLOR_RED            RGB24ToRGB16(0xFF0000)

#define COLOR_NAVY           RGB24ToRGB16(0x000080)
#define COLOR_DARKBLUE       RGB24ToRGB16(0x00008B)
#define COLOR_DARKGREEN      RGB24ToRGB16(0x006400)
#define COLOR_DARKCYAN       RGB24ToRGB16(0x008B8B)
#define COLOR_CYAN           RGB24ToRGB16(0x00FFFF)
#define COLOR_TURQUOISE      RGB24ToRGB16(0x40E0D0)
#define COLOR_INDIGO         RGB24ToRGB16(0x4B0082)
#define COLOR_DARKRED        RGB24ToRGB16(0x800000)
#define COLOR_OLIVE          RGB24ToRGB16(0x808000)
#define COLOR_GRAY           RGB24ToRGB16(0x808080)
#define COLOR_SKYBLUE        RGB24ToRGB16(0x87CEEB)
#define COLOR_BLUEVIOLET     RGB24ToRGB16(0x8A2BE2)
#define COLOR_LIGHTGREEN     RGB24ToRGB16(0x90EE90)
#define COLOR_DARKVIOLET     RGB24ToRGB16(0x9400D3)
#define COLOR_YELLOWGREEN    RGB24ToRGB16(0x9ACD32)
#define COLOR_BROWN          RGB24ToRGB16(0xA52A2A)
#define COLOR_DARKGRAY       RGB24ToRGB16(0xA9A9A9)
#define COLOR_SIENNA         RGB24ToRGB16(0xA0522D)
#define COLOR_LIGHTBLUE      RGB24ToRGB16(0xADD8E6)
#define COLOR_GREENYELLOW    RGB24ToRGB16(0xADFF2F)
#define COLOR_SILVER         RGB24ToRGB16(0xC0C0C0)
#define COLOR_LIGHTGREY      RGB24ToRGB16(0xD3D3D3)
#define COLOR_LIGHTCYAN      RGB24ToRGB16(0xE0FFFF)
#define COLOR_VIOLET         RGB24ToRGB16(0xEE82EE)
#define COLOR_AZUR           RGB24ToRGB16(0xF0FFFF)
#define COLOR_BEIGE          RGB24ToRGB16(0xF5F5DC)
#define COLOR_MAGENTA        RGB24ToRGB16(0xFF00FF)
#define COLOR_TOMATO         RGB24ToRGB16(0xFF6347)
#define COLOR_GOLD           RGB24ToRGB16(0xFFD700)
#define COLOR_ORANGE         RGB24ToRGB16(0xFFA500)
#define COLOR_SNOW           RGB24ToRGB16(0xFFFAFA)
#define COLOR_YELLOW         RGB24ToRGB16(0xFFFF00)

#endif /* #define LCD_COLOR_H */
