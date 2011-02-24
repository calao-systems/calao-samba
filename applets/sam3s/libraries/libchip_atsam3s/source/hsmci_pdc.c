/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2009, Atmel Corporation
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
 * Implementation of High Speed MultiMedia Card Interface (HSMCI) controller,
 * using PDC to transfer data.
 *
 */
/*---------------------------------------------------------------------------
 *         Headers
 *---------------------------------------------------------------------------*/

#include <chip.h>

#include <assert.h>

/*---------------------------------------------------------------------------
 *         Local constants
 *---------------------------------------------------------------------------*/

/** Bit mask for status register errors. */
#define STATUS_ERRORS ((uint32_t)(HSMCI_SR_UNRE  \
                       | HSMCI_SR_OVRE \
                       | HSMCI_SR_ACKRCVE \
                       /*| HSMCI_SR_BLKOVRE*/ \
                       | HSMCI_SR_CSTOE \
                       | HSMCI_SR_DTOE \
                       | HSMCI_SR_DCRCE \
                       | HSMCI_SR_RTOE \
                       | HSMCI_SR_RENDE \
                       | HSMCI_SR_RCRCE \
                       | HSMCI_SR_RDIRE \
                       | HSMCI_SR_RINDE))

/** Bit mask for response errors */
#define STATUS_ERRORS_RESP ((uint32_t)(HSMCI_SR_CSTOE \
                            | HSMCI_SR_RTOE \
                            | HSMCI_SR_RENDE \
                            | HSMCI_SR_RCRCE \
                            | HSMCI_SR_RDIRE \
                            | HSMCI_SR_RINDE))

/** Bit mask for data errors */
#define STATUS_ERRORS_DATA ((uint32_t)(HSMCI_SR_UNRE \
                            | HSMCI_SR_OVRE \
                            /*| HSMCI_SR_BLKOVRE*/ \
                            /*| HSMCI_SR_CSTOE*/ \
                            | HSMCI_SR_DTOE \
                            | HSMCI_SR_DCRCE))

/*---------------------------------------------------------------------------
 *         Local macros
 *---------------------------------------------------------------------------*/

/** Enable MCI */
#define MCI_ENABLE(pMciHw)     (pMciHw->HSMCI_CR = HSMCI_CR_MCIEN)
/** Disable MCI */
#define MCI_DISABLE(pMciHw)    (pMciHw->HSMCI_CR = HSMCI_CR_MCIDIS)
/** Reset MCI */
#define MCI_RESET(pMciHw)      (pMciHw->HSMCI_CR = HSMCI_CR_SWRST)

/** Return halfword(16-bit) count from byte count */
#define toHWCOUNT(byteCnt) (((byteCnt)&0x1) ? (((byteCnt)/2)+1) : ((byteCnt)/2))
/** Return word(32-bit) count from byte count */
#define toWCOUNT(byteCnt)  (((byteCnt)&0x3) ? (((byteCnt)/4)+1) : ((byteCnt)/4))

/*---------------------------------------------------------------------------
 *         Internal functions
 *---------------------------------------------------------------------------*/

/**
 * Enable MCI peripheral access clock
 */
static uint8_t PERIPH_ENABLE(id)
{
    if (PMC_IsPeriphEnabled(id)) return 0;
    PMC_EnablePeripheral(id);
    return 1;
}

/**
 * Disable MCI peripheral access clock
 */
static inline void PERIPH_DISABLE(id)
{
    PMC_DisablePeripheral(id);
}

/*---------------------------------------------------------------------------
 *         Exported functions
 *---------------------------------------------------------------------------*/

/**
 * Initializes a MCI driver instance and the underlying peripheral.
 * \param pMci    Pointer to a MCI driver instance.
 * \param pMciHw  Pointer to a MCI peripheral.
 * \param mciId   MCI peripheral identifier.
 */
extern void MCI_Init( Mcid *pMci, Hsmci *pMciHw, uint8_t mciId, uint32_t dwMCk ) ;
extern void MCI_Init( Mcid *pMci, Hsmci *pMciHw, uint32_t dwMCk, uint32_t dwId )
{
    uint16_t clkDiv;

    /* Initialize the MCI driver structure */
    pMci->pMciHw    = pMciHw;
    pMci->mciId     = dwId ;
    pMci->semaphore = 1;
    pMci->pCommand  = NULL;

    /* Enable the MCI peripheral */
    PERIPH_ENABLE( dwId ) ;

    /* Reset the MCI */
    pMciHw->HSMCI_CR = HSMCI_CR_SWRST;

    /* Disable the MCI */
    pMciHw->HSMCI_CR = HSMCI_CR_MCIDIS | HSMCI_CR_PWSDIS;

    /* Disable all the interrupts */
    pMciHw->HSMCI_IDR = 0xFFFFFFFF;

    /* Set the Data Timeout Register */
    pMciHw->HSMCI_DTOR = HSMCI_DTOR_DTOCYC
                       | HSMCI_DTOR_DTOMUL;
    /* CSTOR ? */
    pMciHw->HSMCI_CSTOR = HSMCI_CSTOR_CSTOCYC
                        | HSMCI_CSTOR_CSTOMUL;

    /* Set the Mode Register: 400KHz for MCK = 48MHz (CLKDIV = 58) */
    clkDiv = (dwMCk / (MCI_INITIAL_SPEED << 1)) - 1;
    pMciHw->HSMCI_MR = (clkDiv | (HSMCI_MR_PWSDIV & (0x7 << 8)));

    /* Set the SDCard Register 1-bit, slot A */
    pMciHw->HSMCI_SDCR = ((0 << 0) & HSMCI_SDCR_SDCSEL)
                       | ((0 << 6) & HSMCI_SDCR_SDCBUS);

    /* Enable the MCI and the Power Saving */
    pMciHw->HSMCI_CR = HSMCI_CR_MCIEN ;

    /* Configure MCI */
    pMciHw->HSMCI_CFG = HSMCI_CFG_FIFOMODE
                      | ((1 << 4) & HSMCI_CFG_FERRCTRL);

    /* Disable the MCI peripheral clock. */
    PERIPH_DISABLE( dwId ) ;
}

/**
 * Reset MCI HW interface and disable it.
 * \param keepSettings Keep old register settings, including
 *                     _MR, _SDCR, _DTOR, _CSTOR, _DMA and _CFG.
 */
void MCI_Reset(Mcid *pMci, uint8_t keepSettings)
{
    Hsmci *pMciHw = pMci->pMciHw;
    uint8_t mciDis;

    assert(pMci);
    assert(pMci->pMciHw);

    mciDis = PERIPH_ENABLE(pMci->mciId);
    if (keepSettings)
    {
        uint32_t mr, sdcr, dtor, cstor;
        uint32_t cfg;
        mr    = pMciHw->HSMCI_MR;
        sdcr  = pMciHw->HSMCI_SDCR;
        dtor  = pMciHw->HSMCI_DTOR;
        cstor = pMciHw->HSMCI_CSTOR;
        cfg   = pMciHw->HSMCI_CFG;
        MCI_RESET(pMciHw);
        MCI_DISABLE(pMciHw);
        pMciHw->HSMCI_MR    = mr;
        pMciHw->HSMCI_SDCR  = sdcr;
        pMciHw->HSMCI_DTOR  = dtor;
        pMciHw->HSMCI_CSTOR = cstor;
        pMciHw->HSMCI_CFG   = cfg;
    }
    else
    {
        MCI_RESET(pMciHw);
        MCI_DISABLE(pMciHw);
    }
    if (mciDis) PERIPH_DISABLE(pMci->mciId);
}

/**
 * Configure the  MCI CLKDIV in the MCI_MR register. The max. for MCI clock is
 * MCK/2 and corresponds to CLKDIV = 0
 * \param pMci  Pointer to the low level MCI driver.
 * \param mciSpeed  MCI clock speed in Hz, 0 will not change current speed.
 * \param mck       MCK to generate MCI Clock, in Hz
 * \return The actual speed used, 0 for fail.
 */
uint32_t MCI_SetSpeed( Mcid* pMci, uint32_t mciSpeed, uint32_t mck )
{
    Hsmci *pMciHw = pMci->pMciHw;
    uint32_t mciMr;
    uint32_t clkdiv;
    uint8_t  mciDis;

    assert(pMci);
    assert(pMciHw);

    mciDis = PERIPH_ENABLE(pMci->mciId);
    mciMr = pMciHw->HSMCI_MR & (~(uint32_t)HSMCI_MR_CLKDIV);
    /* Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
     * divided by (2*(CLKDIV+1))
     * mciSpeed = MCK / (2*(CLKDIV+1)) */
    if (mciSpeed > 0) {
        clkdiv = (mck / 2 / mciSpeed);
        /* Speed should not bigger than expired one */
        if (mciSpeed < mck/2/clkdiv)
        {
            clkdiv ++;
        }
        if ( clkdiv > 0 )
        {
            clkdiv -= 1;
        }
        assert( (clkdiv & 0xFFFFFF00) == 0 ) ; /* "mciSpeed too small" */
    }
    else
    {
	    clkdiv = 0 ;
    }
    /* Actual MCI speed */
    mciSpeed = mck / 2 / (clkdiv + 1);
    /* Modify MR */
    pMciHw->HSMCI_MR = mciMr | clkdiv;
    if (mciDis) PERIPH_DISABLE(pMci->mciId);

    return (mciSpeed);
}

/**
 * Configure the MCI_CFG to enable the HS mode
 * \param pMci     Pointer to the low level MCI driver.
 * \param hsEnable 1 to enable, 0 to disable HS mode.
 */
uint8_t MCI_EnableHsMode(Mcid* pMci, uint8_t hsEnable)
{
    Hsmci *pMciHw = pMci->pMciHw;
    uint32_t cfgr;
    uint8_t mciDis;
    uint8_t rc = 0;

    assert(pMci);
    assert(pMci->pMciHw);

    mciDis = PERIPH_ENABLE(pMci->mciId);
    cfgr = pMciHw->HSMCI_CFG;
    if ( hsEnable == 1 )
    {
        cfgr |=  HSMCI_CFG_HSMODE;
    }
    else
    {
        if (hsEnable == 0)
        {
            cfgr &= ~(uint32_t)HSMCI_CFG_HSMODE;
        }
    }
    else
    {
        rc = ((cfgr & HSMCI_CFG_HSMODE) != 0);
    }

    pMciHw->HSMCI_CFG = cfgr;
    if (mciDis)
    {
        PERIPH_DISABLE(pMci->mciId);
    }

    return rc;
}

/**
 * Configure the  MCI SDCBUS in the MCI_SDCR register. Only two modes available
 *
 * \param pMci  Pointer to the low level MCI driver.
 * \param busWidth  MCI bus width mode. 00: 1-bit, 10: 4-bit, 11: 8-bit.
 */
uint8_t MCI_SetBusWidth(Mcid*pMci, uint8_t busWidth)
{
    Hsmci *pMciHw = pMci->pMciHw;
    uint32_t mciSdcr;
    uint8_t mciDis;

    assert(pMci);
    assert(pMci->pMciHw);

    switch ( busWidth )
    {
        case SDMMC_BUS_1_BIT:
            busWidth = 0;
        break;

        case SDMMC_BUS_4_BIT:
            busWidth = 2;
        break;

        default:
            return SDMMC_ERROR_PARAM;
    }

    mciDis = PERIPH_ENABLE(pMci->mciId);
    mciSdcr = (pMciHw->HSMCI_SDCR & ~(uint32_t)(HSMCI_SDCR_SDCBUS));
    pMciHw->HSMCI_SDCR = mciSdcr | (busWidth << 6);

    if (mciDis)
    {
        PERIPH_DISABLE(pMci->mciId);
    }

    return 0;
}

/**
 * Starts a MCI  transfer. This is a non blocking function. It will return
 * as soon as the transfer is started.
 * Return 0 if successful; otherwise returns MCI_ERROR_LOCK if the driver is
 * already in use.
 * \param pMci  Pointer to an MCI driver instance.
 * \param pCommand  Pointer to the command to execute.
 */
uint8_t MCI_SendCommand(Mcid*pMci, MciCmd *pCommand)
{
    Hsmci* pMciHw = pMci->pMciHw;
    uint32_t mciIer = 0, mciMr;
    uint32_t transSize;

    assert(pMci);
    assert(pMciHw);
    assert(pCommand);

    /* Try to acquire the MCI semaphore */
    if (pMci->semaphore == 0)
    {
        return SDMMC_ERROR_BUSY;
    }
    pMci->semaphore--;

    /* Command is now being executed */
    pMci->pCommand = pCommand;
    pCommand->state = SDMMC_CMD_PENDING;

    /* Enable the MCI peripheral clock */
    PERIPH_ENABLE(pMci->mciId);

#if 1
    /* Wait for NOTBUSY (DATA bus IDLE) */
    if (pCommand->tranType == MCI_STOP_TRANSFER)
    {
        volatile uint32_t busyWait = 0x00001000;

        while ( busyWait-- )
        {
          uint32_t sr = pMciHw->HSMCI_SR;

          if ((sr & HSMCI_SR_NOTBUSY) && ((sr & HSMCI_SR_DTIP) == 0))
          {
              break;
          }
        }
    }
#endif

    /* Disable MCI */
    MCI_DISABLE(pMciHw);

    /* Prepare Default Mode register value */
    mciMr = pMciHw->HSMCI_MR & (~(HSMCI_MR_WRPROOF
                                  |HSMCI_MR_RDPROOF
                                  |HSMCI_MR_BLKLEN
                                  |HSMCI_MR_PDCMODE
                                  |HSMCI_MR_FBYTE));

    /* Stop transfer: idle the bus */
    if (pCommand->tranType == MCI_STOP_TRANSFER)
    {
        pMciHw->HSMCI_MR = mciMr;
        mciIer = HSMCI_IER_XFRDONE | STATUS_ERRORS_RESP;
        //mciIer = HSMCI_IER_CMDRDY | STATUS_ERRORS_RESP;
    }
    /* No data transfer: stop at the end of the command */
    else if (pCommand->tranType == MCI_NO_TRANSFER)
    {
        pMciHw->HSMCI_MR = mciMr;
        mciIer = HSMCI_IER_XFRDONE | STATUS_ERRORS_RESP;
        /* R3 response, no CRC check */
        if (pCommand->resType == 3)
        {
            mciIer &= ~((uint32_t)HSMCI_IER_RCRCE);
        }
    }
    /* Data command but no following:
       Start an infinite block transfer */
    else
        if (pCommand->nbBlock == 0)
    {
        /* Block size... Data length 0: no PDC */
        pMciHw->HSMCI_MR = mciMr | HSMCI_MR_WRPROOF
                                 | HSMCI_MR_RDPROOF;
        pMciHw->HSMCI_BLKR = (pCommand->blockSize << 16);
        mciIer = HSMCI_IER_CMDRDY | STATUS_ERRORS_RESP;
    }
    /* Command with DATA stage */
    else
    {
        /* Check data size */
        if (pCommand->nbBlock == 0)
        {
            TRACE_ERROR("MCICmd: At least one block required\n\r");
            return SDMMC_ERROR_PARAM;
        }

        /* Number of blocks to transfer */
        if (   pCommand->tranType == MCI_START_WRITE
            || pCommand->tranType == MCI_START_READ)
        {
            /* Set number of blocks */
            pMciHw->HSMCI_BLKR = (pCommand->blockSize << 16)
                               | (pCommand->nbBlock   <<  0);
        }

        /* Force byte transfer */
        transSize = (pCommand->nbBlock * pCommand->blockSize);
        if (pCommand->blockSize & 0x3)
        {
            mciMr |= HSMCI_MR_FBYTE;
        }
        else
        {
            transSize = toWCOUNT(transSize);
        }

        /* Set block size & enable PDC */
        pMciHw->HSMCI_MR = mciMr | HSMCI_MR_WRPROOF
                                 | HSMCI_MR_RDPROOF
                                 | HSMCI_MR_PDCMODE
                                 | (pCommand->blockSize << 16);

        /* Update PDC for write */
        if (   pCommand->tranType == MCI_START_WRITE
            || pCommand->tranType == MCI_WRITE )
        {
            pMciHw->HSMCI_TPR = (uint32_t)pCommand->pData;
            pMciHw->HSMCI_TCR = transSize;
            pMciHw->HSMCI_TNCR = 0;

            if (pCommand->tranType == MCI_START_WRITE)
            {
                mciIer = (uint32_t)(HSMCI_IER_XFRDONE | STATUS_ERRORS_DATA);
            }
            else
            {
                mciIer = (uint32_t)(HSMCI_IER_TXBUFE | STATUS_ERRORS_DATA);
            }
        }
        /* Update PDC for read */
        else
        {
            pMciHw->HSMCI_RPR = (uint32_t)pCommand->pData;
            pMciHw->HSMCI_RCR = transSize;
            pMciHw->HSMCI_RNCR = 0;
            pMciHw->HSMCI_PTCR = HSMCI_PTCR_RXTEN;
            if (pCommand->tranType == MCI_START_READ)
            {
                mciIer = (uint32_t)(HSMCI_IER_XFRDONE | STATUS_ERRORS_DATA);
            }
            else
            {
                mciIer = (uint32_t)(HSMCI_IER_RXBUFF | STATUS_ERRORS_DATA);
            }
        }
    }

    /* Enable MCI */
    MCI_ENABLE(pMciHw);

    /* Send the command */
    if ( (pCommand->tranType != MCI_WRITE && pCommand->tranType != MCI_READ) || (pCommand->blockSize == 0) )
    {
        pMciHw->HSMCI_ARGR = pCommand->arg;
        pMciHw->HSMCI_CMDR = pCommand->cmd;
    }

    /* Start sending PDC */
    if ( pCommand->blockSize > 0 && (pCommand->tranType == MCI_START_WRITE || pCommand->tranType == MCI_WRITE) )
    {
        pMciHw->HSMCI_PTCR = HSMCI_PTCR_TXTEN;
    }

    /* Ignore CRC error for R3 & R4 */
    if (pCommand->resType == 3 || pCommand->resType == 4)
    {
        mciIer &= ~(uint32_t)(HSMCI_IER_RCRCE);
    }

    /* Ignore data error on stop transfer command :) */
    if (pCommand->tranType == MCI_STOP_TRANSFER)
    {
        mciIer &= ~(STATUS_ERRORS_DATA);
    }

    /* Interrupt enable shall be done after PDC TXTEN and RXTEN */
    pMciHw->HSMCI_IER = mciIer;

    return 0;
}

/**
 * Processes pending events on the given MCI driver.
 * \param pMci  Pointer to a MCI driver instance.
 */
void MCI_Handler(Mcid*pMci)
{
    Hsmci *pMciHw = pMci->pMciHw;
    MciCmd *pCommand = pMci->pCommand;
    volatile uint32_t SR;
    volatile uint32_t maskedSR;
    uint32_t mask;
    uint8_t i;

    assert(pMci);
    assert(pMciHw);
    assert(pCommand);

    /* Read the status register */
    SR = pMciHw->HSMCI_SR;
    mask = pMciHw->HSMCI_IMR;
    maskedSR = SR & mask;

    //TRACE_INFO_WP("i%dS %x\n\r", (pCommand->cmd & HSMCI_CMDR_CMDNB), SR);
    //TRACE_INFO_WP("i%dM %x\n\r", (pCommand->cmd & HSMCI_CMDR_CMDNB), maskedSR);

    /* Check if an error has occured */
    if ((maskedSR & STATUS_ERRORS) != 0)
    {
        pCommand->state = SDMMC_CMD_ERROR;
        /* Check error code */
        if ((maskedSR & STATUS_ERRORS) == HSMCI_SR_RTOE)
        {
            pCommand->status = SDMMC_ERROR_NORESPONSE;
        }
        else
        {
            TRACE_INFO("iERR %x: C%x(%d), R%d\n\r",
                maskedSR, pCommand->cmd,
                (pCommand->cmd & HSMCI_CMDR_CMDNB), pCommand->resType);
            pCommand->status = SDMMC_ERROR;
        }
    }
    mask &= ~STATUS_ERRORS;

    /* Check if a command has been completed */
    if (maskedSR & HSMCI_SR_CMDRDY)
    {
        /* Command OK, disable interrupt */
        pMciHw->HSMCI_IDR = HSMCI_IDR_CMDRDY;

        /* Check BUSY */
        if ( pCommand->busyCheck && (pCommand->tranType == MCI_NO_TRANSFER || pCommand->tranType == MCI_STOP_TRANSFER) )
        {
            /* Check XFRDONE to confirm no bus action */
            if (SR & HSMCI_SR_NOTBUSY)
            {
                mask &= ~(uint32_t)HSMCI_IMR_CMDRDY;
            }
            else
            {
                pMciHw->HSMCI_IER = HSMCI_SR_NOTBUSY;
            }
        }
        else
        {
            mask &= ~(uint32_t)HSMCI_IMR_CMDRDY;
        }
    }

    /* Check if transfer stopped */
    if (maskedSR & HSMCI_SR_XFRDONE) {
        mask &= ~(uint32_t)HSMCI_IMR_XFRDONE;
    }

    /* Check if not busy */
    if (maskedSR & HSMCI_SR_NOTBUSY)
    {
        mask &= ~(uint32_t)HSMCI_IMR_NOTBUSY;
    }

    /* Check if data read done */
    if (maskedSR & HSMCI_SR_RXBUFF)
    {
        mask &= ~(uint32_t)HSMCI_IMR_RXBUFF;
    }

    /* Check if TX ready */
    if (maskedSR & HSMCI_SR_TXRDY)
    {
        mask &= ~(uint32_t)HSMCI_IMR_TXRDY;
    }

    /* Check if FIFO empty (TX & All data sent) */
    if (maskedSR & HSMCI_SR_FIFOEMPTY)
    {
        /* Disable FIFO EMPTY */
        pMciHw->HSMCI_IDR = HSMCI_SR_FIFOEMPTY;
        /* Byte transfer no BLKE */
        if (pMciHw->HSMCI_MR & HSMCI_MR_FBYTE)
        {
            mask &= ~(uint32_t)HSMCI_IMR_FIFOEMPTY;
        }
        /* End command until no data on bus */
        else
        {
            if ((SR & HSMCI_SR_BLKE) == 0)
            {
                pMciHw->HSMCI_IER = HSMCI_IER_BLKE;
            }
            else
            {
                mask &= ~(uint32_t)HSMCI_IMR_FIFOEMPTY;
            }
        }
    }

    /* Check if BLKE (TX & no data on bus) */
    if (maskedSR & HSMCI_SR_BLKE)
    {
#if 0
        /* Check BUSY */
        if (pCommand->busyCheck)
        {
            /* Check XFRDONE to confirm data written */
            if (SR & HSMCI_SR_NOTBUSY)
            {
                mask &= ~(uint32_t)HSMCI_SR_BLKE;
            }
            else
            {
                pMciHw->HSMCI_IDR = HSMCI_SR_BLKE;
                pMciHw->HSMCI_IER = HSMCI_SR_NOTBUSY;
            }
        }
#else
#   if 1
        if (SR & HSMCI_SR_TXRDY )
        {
            mask &= ~(uint32_t)HSMCI_SR_BLKE;
        }
        else
        {
            pMciHw->HSMCI_IDR = HSMCI_SR_BLKE;
            pMciHw->HSMCI_IER = HSMCI_SR_TXRDY;
        }
#   else
        mask &= ~(uint32_t)HSMCI_SR_BLKE;
#   endif
#endif
    }

    /* Check if data write finished */
    if (maskedSR & HSMCI_SR_TXBUFE)
    {
        /* Data OK, disable interrupt */
        pMciHw->HSMCI_IDR = HSMCI_IDR_TXBUFE;

        /* End command until no data on bus */
        if ((SR & HSMCI_SR_FIFOEMPTY) == 0)
        {
            pMciHw->HSMCI_IER = HSMCI_IER_FIFOEMPTY;
        }
        else
        {
            mask &= ~(uint32_t)HSMCI_SR_TXBUFE;
        }
    }

    /* All non-error mask done, complete the command */
    if (0 == mask || pCommand->state != SDMMC_CMD_PENDING)
    {
        /* Store the card response in the provided buffer */
        if (pCommand->pResp)
        {
            uint8_t resSize;
            switch (pCommand->resType)
            {
                case 1:
                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                    resSize = 1;
                break;

                case 2:
                    resSize = 4;
                break;

                default:
                    resSize = 0;
                break;
            }

            for (i=0; i < resSize; i++)
            {
                pCommand->pResp[i] = pMciHw->HSMCI_RSPR[0];
            }
        }

        /* If no error occured, the transfer is successful */
        if (pCommand->state == SDMMC_CMD_PENDING)
        {
            pCommand->state = 0;
        }
        /* Error ? Reset to remove transfer statuses */
        else
        {
            MCI_Reset(pMci, 1);
        }

        /* Disable PDC */
        pMciHw->HSMCI_PTCR = HSMCI_PTCR_RXTDIS | HSMCI_PTCR_TXTDIS;

        /* Disable interrupts */
        pMciHw->HSMCI_IDR = pMciHw->HSMCI_IMR;

        /* Disable peripheral */
        PERIPH_DISABLE(pMci->mciId);

        /* Release the semaphore */
        pMci->semaphore++;

        /* Invoke the callback associated with the current command (if any) */
        if ( pCommand->callback )
        {
            (pCommand->callback)(pCommand->status, pCommand->pArg);
        }
    }
}

/**
 * Returns 1 if the given MCI transfer is complete; otherwise returns 0.
 * \param pMci  Pointer to a Mcid instance.
 */
uint8_t MCI_IsCommandComplete( Mcid *pMci )
{
    MciCmd *pCommand = pMci->pCommand;

    if ( pMci->semaphore == 0 )
    {
        return 0;
    }

    if ( pCommand )
    {
        if ( pCommand->state == SDMMC_CMD_PENDING )
        {
            return 0;
        }
    }

    return 1;
}
