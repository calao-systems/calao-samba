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

/** \addtogroup pwm_module Working with PWM
 * The PWM driver provides the interface to configure and use the PWM
 * peripheral.
 *
 * The PWM macrocell controls square output waveforms of several channels.
 * Characteristics of output waveforms such as period, duty-cycle,
 * dead-time can be configured.\n
 *
 * Before enabling the channels, they must have been configured first.
 * The main settings include:
 * <ul>
 * <li>Configuration of the clock generator.</li>
 * <li>Selection of the clock for each channel.</li>
 * <li>Configuration of output waveform characteristics, such as period, duty-cycle etc.</li>
 * </ul>
 *
 * After the channels is enabled, the user must use respective update registers
 * to change the wave characteristics to prevent unexpected output waveform.
 * i.e. PWM_CDTYUPDx register should be used if user want to change duty-cycle
 * when the channel is enabled.
 *
 * For more accurate information, please look at the PWM section of the
 * Datasheet.
 *
 * Related files :\n
 * \ref pwmc.c\n
 * \ref pwmc.h.\n
 */
/*@{*/
/*@}*/

/**
 * \file
 *
 * Implementation of the Pulse Width Modulation Controller (PWM) peripheral.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <chip.h>

#include <stdint.h>
#include <assert.h>

/*----------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Finds a prescaler/divisor couple to generate the desired frequency
 * from MCK.
 *
 * Returns the value to enter in PWM_MR or 0 if the configuration cannot be
 * met.
 *
 * \param frequency  Desired frequency in Hz.
 * \param mck  Master clock frequency in Hz.
 */
static uint16_t FindClockConfiguration(
    uint32_t frequency,
    uint32_t mck)
{
    uint32_t divisors[11] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024};
    uint8_t divisor = 0;
    uint32_t prescaler;

    assert(frequency < mck);

    /* Find prescaler and divisor values */
    prescaler = (mck / divisors[divisor]) / frequency;
    while ((prescaler > 255) && (divisor < 11)) {

        divisor++;
        prescaler = (mck / divisors[divisor]) / frequency;
    }

    /* Return result */
    if ( divisor < 11 )
    {
        TRACE_DEBUG( "Found divisor=%u and prescaler=%u for freq=%uHz\n\r", divisors[divisor], prescaler, frequency ) ;

        return prescaler | (divisor << 8) ;
    }
    else
    {
        return 0 ;
    }
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Configures PWM a channel with the given parameters, basic configure function.
 *
 * The PWM controller must have been clocked in the PMC prior to calling this
 * function.
 * Beware: this function disables the channel. It waits until disable is effective.
 *
 * \param channel  Channel number.
 * \param prescaler  Channel prescaler.
 * \param alignment  Channel alignment.
 * \param polarity  Channel polarity.
 */
void PWMC_ConfigureChannel(
    uint8_t channel,
    uint32_t prescaler,
    uint32_t alignment,
    uint32_t polarity)
{
    assert(prescaler < PWM_CMR_CPRE_CLKB);
    assert((alignment & (uint32_t)~PWM_CMR_CALG) == 0);
    assert((polarity & (uint32_t)~PWM_CMR_CPOL) == 0);

    /* Disable channel (effective at the end of the current period) */
    if ((PWM->PWM_SR & (1 << channel)) != 0) {
        PWM->PWM_DIS = 1 << channel;
        while ((PWM->PWM_SR & (1 << channel)) != 0);
    }

    /* Configure channel */
    PWM->PWM_CH_NUM[channel].PWM_CMR = prescaler | alignment | polarity;
}

/**
 * \brief Configures PWM clocks A & B to run at the given frequencies.
 *
 * This function finds the best MCK divisor and prescaler values automatically.
 *
 * \param clka  Desired clock A frequency (0 if not used).
 * \param clkb  Desired clock B frequency (0 if not used).
 * \param mck  Master clock frequency.
 */
void PWMC_ConfigureClocks(uint32_t clka, uint32_t clkb, uint32_t mck)
{
    uint32_t mode = 0;
    uint32_t result;

    /* Clock A */
    if (clka != 0) {

        result = FindClockConfiguration(clka, mck);
        assert( result != 0 ) ;
        mode |= result;
    }

    /* Clock B */
    if (clkb != 0) {

        result = FindClockConfiguration(clkb, mck);
        assert( result != 0 ) ;
        mode |= (result << 16);
    }

    /* Configure clocks */
    TRACE_DEBUG( "Setting PWM_MR = 0x%08X\n\r", mode ) ;
    PWM->PWM_MR = mode;
}

/**
 * \brief Sets the period value used by a PWM channel.
 *
 * This function writes directly to the CPRD register if the channel is disabled;
 * otherwise, it uses the update register PWM_CUPD.
 *
 * \param channel Channel number.
 * \param period  Period value.
 */
void PWMC_SetPeriod(uint8_t channel, uint16_t period)
{
    /* If channel is disabled, write to CPRD */
    if ((PWM->PWM_SR & (1 << channel)) == 0) {

        PWM->PWM_CH_NUM[channel].PWM_CPRD = period;
    }
    /* Otherwise use update register */
    else {

        PWM->PWM_CH_NUM[channel].PWM_CMR |= (uint32_t)PWM_CMR_CPD;
        PWM->PWM_CH_NUM[channel].PWM_CUPD = period;
    }
}

/**
 * \brief Sets the duty cycle used by a PWM channel.
 * This function writes directly to the CDTY register if the channel is disabled;
 * otherwise it uses the update register PWM_CUPD.
 * Note that the duty cycle must always be inferior or equal to the channel
 * period.
 *
 * \param channel  Channel number.
 * \param duty     Duty cycle value.
 */
void PWMC_SetDutyCycle(uint8_t channel, uint16_t duty)
{
    assert(duty <= PWM->PWM_CH_NUM[channel].PWM_CPRD);

    /* If channel is disabled, write to CDTY */
    if ((PWM->PWM_SR & (1 << channel)) == 0) {

        PWM->PWM_CH_NUM[channel].PWM_CDTY = duty;
    }
    /* Otherwise use update register */
    else {

        PWM->PWM_CH_NUM[channel].PWM_CMR &= ~(uint32_t)PWM_CMR_CPD;
        PWM->PWM_CH_NUM[channel].PWM_CUPD = duty;
    }
}

/**
 * \brief Enables the given PWM channel.
 *
 * This does NOT enable the corresponding pin;this must be done in the user code.
 *
 * \param channel  Channel number.
 */
void PWMC_EnableChannel(uint8_t channel)
{
    PWM->PWM_ENA = 1 << channel;
}

/**
 * \brief Disables the given PWM channel.
 *
 * Beware, channel will be effectively disabled at the end of the current period.
 * Application can check channel is disabled using the following wait loop:
 * while ((PWM->PWM_SR & (1 << channel)) != 0);
 *
 * \param channel  Channel number.
 */
void PWMC_DisableChannel(uint8_t channel)
{
    PWM->PWM_DIS = 1 << channel;
}

/**
 * \brief Enables the period interrupt for the given PWM channel.
 *
 * \param channel  Channel number.
 */
void PWMC_EnableChannelIt(uint8_t channel)
{
    PWM->PWM_IER = 1 << channel;
}

/**
 * \brief Disables the period interrupt for the given PWM channel.
 *
 * \param channel  Channel number.
 */
void PWMC_DisableChannelIt(uint8_t channel)
{
    PWM->PWM_IDR = 1 << channel;
}

