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
 * not using PDC nor DMA to transfer data.
 *
 */

/*---------------------------------------------------------------------------
 *         Headers
 *---------------------------------------------------------------------------*/

#include <chip.h>

#include <assert.h>

/*---------------------------------------------------------------------------
 *         Compile options
 *---------------------------------------------------------------------------*/

/** Enable writing with FIFO (Not available now) */
//#define HSMCI_WRITE_FIFO
/** Enable reading with FIFO (Not available now) */
//#define HSMCI_READ_FIFO

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

/*---------------------------------------------------------------------------
 *         Local types
 *---------------------------------------------------------------------------*/

/** Transfer parameters */
typedef struct _MciTransfer {
    uint8_t  *pData;        /** Pointer to data buffer */
    uint32_t size;          /** Total number of bytes to transfer */
    uint32_t transferred;   /** Number of bytes finished */
    uint16_t fifoed;        /** bytes in FIFO */
    uint8_t  isRx;          /** 1 for RX */
    uint8_t  reserved;
} MciTransfer;

/*---------------------------------------------------------------------------
 *         Local variables
 *---------------------------------------------------------------------------*/

/** Tx/Rx parameters */
MciTransfer hsmciXfr;

/*---------------------------------------------------------------------------
 *         Internal functions
 *---------------------------------------------------------------------------*/

/**
 * Enable MCI peripheral access clock
 */
static uint32_t PERIPH_ENABLE( uint32_t dwId )
{
    if ( PMC_IsPeriphEnabled( dwId ) )
    {
        return 0 ;
    }

    PMC_EnablePeripheral( dwId ) ;

    return 1 ;
}

/**
 * Disable MCI peripheral access clock
 */
static inline void PERIPH_DISABLE( uint32_t dwId )
{
    PMC_DisablePeripheral( dwId ) ;
}
#ifdef HSMCI_WRITE_FIFO
/**
 * Write data to FIFO
 * \param pHw  Pointer to HSMCI register base.
 * \param pXfr Pointer to transfer instance.
 * \param size Size of data in bytes.
 */
static uint8_t FIFO_Write(Hsmci *pHw, MciTransfer *pXfr, uint32_t size)
{
    volatile uint32_t *pFIFO = (volatile uint32_t*)((uint32_t)pHw + 0x200);
    register uint32_t c4, c1;
    uint32_t *pData = (uint32_t*)&pXfr->pData[pXfr->transferred];

    if (size > pXfr->size - pXfr->transferred)
        size = pXfr->size - pXfr->transferred;

    if (size == 0)
        return 0;

    pXfr->fifoed += size;

    /* convert bytes to DW */
    if (size & 0x3) size += 4;
    size /= 4;

    c4 = size >> 2;
    c1 = size & 0x3;
    for (;c4; c4 --) {
        *pFIFO ++ = *pData ++;
        *pFIFO ++ = *pData ++;
        *pFIFO ++ = *pData ++;
        *pFIFO ++ = *pData ++;
    }
    for (;c1; c1 --) {
        *pFIFO ++ = *pData ++;
    }

    return 0;
}
#endif
#ifdef HSMCI_READ_FIFO
/**
 * Read data from FIFO
 * \param pHw  Pointer to HSMCI register base.
 * \param pXfr Pointer to transfer instance.
 * \param size Size of data in DW.
 */
static uint8_t FIFO_Read(Hsmci *pHw, MciTransfer *pXfr, uint32_t size)
{
    volatile uint32_t *pFIFO = (volatile uint32_t*)((uint32_t)pHw + 0x200);
    register uint32_t c4, c1;
    uint32_t *pData = (uint32_t*)&pXfr->pData[pXfr->transferred];

    if (size > pXfr->size - pXfr->transferred)
        size = pXfr->size - pXfr->transferred;

    if (size == 0)
        return 0;

    pXfr->fifoed -= size;
    pXfr->transferred += size;

    /* convert bytes to DW */
    if (size & 0x3) size += 4;
    size /= 4;

    c4 = size >> 2;
    c1 = size & 0x3;
    for (;c4; c4 --) {
        *pData ++ = *pFIFO ++;
        *pData ++ = *pFIFO ++;
        *pData ++ = *pFIFO ++;
        *pData ++ = *pFIFO ++;
    }
    for (;c1; c1 --) {
        *pData ++ = *pFIFO ++;
    }

    return 0;
}
#endif
/*---------------------------------------------------------------------------
 *         Exported functions
 *---------------------------------------------------------------------------*/

/**
 * Initializes a MCI driver instance and the underlying peripheral.
 * \param pMci    Pointer to a MCI driver instance.
 * \param pMciHw  Pointer to a MCI peripheral.
 * \param mciId   MCI peripheral identifier.
 */
void MCI_Init( Mcid *pMci, Hsmci *pMciHw, uint8_t mciId, uint32_t dwMCk )
{
    unsigned short clkDiv;

    /* Initialize the MCI driver structure */
    pMci->pMciHw    = pMciHw;
    pMci->mciId     = mciId;
    pMci->semaphore = 1;
    pMci->pCommand  = NULL;

    /* Enable the MCI peripheral */
    PERIPH_ENABLE(mciId);

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
    clkDiv = (dwMCk / (MCI_INITIAL_SPEED * 2)) - 1;
    pMciHw->HSMCI_MR = (clkDiv | (HSMCI_MR_PWSDIV & (0x7 << 8)));

    /* Set the SDCard Register 1-bit, slot A */
    pMciHw->HSMCI_SDCR = ((0 << 0) & HSMCI_SDCR_SDCSEL)
                       | ((0 << 6) & HSMCI_SDCR_SDCBUS);

    /* Enable the MCI and the Power Saving */
    pMciHw->HSMCI_CR = HSMCI_CR_MCIEN;

    /* Configure MCI */
    pMciHw->HSMCI_CFG = HSMCI_CFG_FIFOMODE
                      | ((1 << 4) & HSMCI_CFG_FERRCTRL);

    /* Disable the MCI peripheral clock. */
    PERIPH_DISABLE(mciId);
}

/**
 * Configure the  MCI CLKDIV in the MCI_MR register. The max. for MCI clock is
 * MCK/2 and corresponds to CLKDIV = 0
 * \param pMci  Pointer to the low level MCI driver.
 * \param mciSpeed  MCI clock speed in Hz, 0 will not change current speed.
 * \param mck       MCK to generate MCI Clock, in Hz
 * \return The actual speed used, 0 for fail.
 */
uint32_t MCI_SetSpeed(Mcid* pMci,
                      uint32_t mciSpeed,
                      uint32_t mck)
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
    if (mciSpeed > 0)
    {
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
    if ( mciDis )
	{
        PERIPH_DISABLE( pMci->mciId ) ;
    }

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
    if (hsEnable == 1)      cfgr |=  HSMCI_CFG_HSMODE;
    else if (hsEnable == 0) cfgr &= ~(uint32_t)HSMCI_CFG_HSMODE;
    else rc = ((cfgr & HSMCI_CFG_HSMODE) != 0);
    pMciHw->HSMCI_CFG = cfgr;
    if (mciDis) PERIPH_DISABLE(pMci->mciId);

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

    switch(busWidth) {
        case SDMMC_BUS_1_BIT: busWidth = 0; break;
        case SDMMC_BUS_4_BIT: busWidth = 2; break;
        default:
            return SDMMC_ERROR_PARAM;
    }
    mciDis = PERIPH_ENABLE(pMci->mciId);
    mciSdcr = (pMciHw->HSMCI_SDCR & ~(uint32_t)(HSMCI_SDCR_SDCBUS));
    pMciHw->HSMCI_SDCR = mciSdcr | (busWidth << 6);
    if (mciDis) PERIPH_DISABLE(pMci->mciId);

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

    assert(pMci);
    assert(pMciHw);
    assert(pCommand);

    /* Try to acquire the MCI semaphore */
    if (pMci->semaphore == 0) {

        return SDMMC_ERROR_BUSY;
    }
    pMci->semaphore--;

    /* Command is now being executed */
    pMci->pCommand = pCommand;
    pCommand->state = SDMMC_CMD_PENDING;

    /* Enable the MCI peripheral clock */
    PERIPH_ENABLE(pMci->mciId);

    /* Disable MCI */
    MCI_DISABLE(pMciHw);

    /* Prepare Default Mode register value */
    mciMr = pMciHw->HSMCI_MR & (~(HSMCI_MR_WRPROOF
                                  |HSMCI_MR_RDPROOF
                                  |HSMCI_MR_BLKLEN
                                  |HSMCI_MR_PDCMODE
                                  |HSMCI_MR_FBYTE));

    /* Stop transfer: idle the bus */
    if (pCommand->tranType == MCI_STOP_TRANSFER) {
        pMciHw->HSMCI_MR = mciMr;
        mciIer = HSMCI_IER_XFRDONE | STATUS_ERRORS_RESP;
    }
    /* No data transfer: stop at the end of the command */
    else if (pCommand->tranType == MCI_NO_TRANSFER) {
        pMciHw->HSMCI_MR = mciMr;
        mciIer = HSMCI_IER_XFRDONE | STATUS_ERRORS_RESP;
        /* R3 response, no CRC check */
        if (pCommand->resType == 3) {
            mciIer &= ~((uint32_t)HSMCI_IER_RCRCE);
        }
    }
    /* Data command but no following:
       Start an infinite block transfer */
    else if (pCommand->nbBlock == 0) {
        /* Block size... Data length 0: no PDC */
        pMciHw->HSMCI_MR = mciMr | HSMCI_MR_WRPROOF
                                 | HSMCI_MR_RDPROOF;
        pMciHw->HSMCI_BLKR = (pCommand->blockSize << 16);
        mciIer = HSMCI_IER_CMDRDY | STATUS_ERRORS_RESP;
    }
    /* Command with DATA stage */
    else {
        /* Check data size */
        if (pCommand->nbBlock == 0) {
            TRACE_ERROR("MCICmd: At least one block required\n\r");
            return SDMMC_ERROR_PARAM;
        }
        /* Number of blocks to transfer */
        {
            /* Set number of blocks */
            pMciHw->HSMCI_BLKR = (pCommand->blockSize << 16)
                               | (pCommand->nbBlock   <<  0);
        }

        /* Force byte transfer */
        if (pCommand->blockSize & 0x3) {
            mciMr |= HSMCI_MR_FBYTE;
        }
        /* Set block size but disable PDC */
        pMciHw->HSMCI_MR = mciMr | HSMCI_MR_WRPROOF
                                 | HSMCI_MR_RDPROOF
                                 | (pCommand->blockSize << 16);
        /* Update transfer struct */
        hsmciXfr.pData = pCommand->pData;
        hsmciXfr.size  = pCommand->nbBlock * pCommand->blockSize;
        hsmciXfr.fifoed      = 0;
        hsmciXfr.transferred = 0;

        /* Update Tx struct for write */
        if (   pCommand->tranType == MCI_START_WRITE
            || pCommand->tranType == MCI_WRITE ) {
            hsmciXfr.isRx  = 0;
          #ifdef HSMCI_WRITE_FIFO
            mciIer = (uint32_t)(HSMCI_IER_BLKE | STATUS_ERRORS_DATA);
          #else
            pMciHw->HSMCI_TDR =
                *(uint32_t*)&hsmciXfr.pData[hsmciXfr.transferred];
            mciIer = (uint32_t)(HSMCI_IER_TXRDY | STATUS_ERRORS_DATA);
          #endif
        }
        /* Update Rx struct for read */
        else {
            hsmciXfr.isRx  = 1;
            /* RX byte by byte */
            mciIer = (uint32_t)(HSMCI_IER_RXRDY | STATUS_ERRORS_DATA);
        }
    }
    /* Enable MCI */
    MCI_ENABLE(pMciHw);

    /* Send the command */
    if((pCommand->tranType != MCI_WRITE && pCommand->tranType != MCI_READ)
        || (pCommand->blockSize == 0)) {

        pMciHw->HSMCI_ARGR = pCommand->arg;
        pMciHw->HSMCI_CMDR = pCommand->cmd;
    }

    /* Start sending FIFO */
    if (pCommand->blockSize > 0
        && (pCommand->tranType == MCI_START_WRITE
            || pCommand->tranType == MCI_WRITE) ) {
#ifdef HSMCI_WRITE_FIFO
        /* Prepare 2 blocks for write */
        FIFO_Write(pMciHw, &hsmciXfr, pCommand->blockSize * 2);
#else
        //pMciHw->HSMCI_TDR = *(uint32_t*)&hsmciXfr.pData[hsmciXfr.transferred];
#endif
    }

    /* Ignore data error ? */
    mciIer &= ~(STATUS_ERRORS_DATA);

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
    volatile uint32_t mask;
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
    if ((maskedSR & STATUS_ERRORS) != 0) {
        pCommand->state = SDMMC_CMD_ERROR;
        /* Check error code */
        if ((maskedSR & STATUS_ERRORS) == HSMCI_SR_RTOE)
        {
            pCommand->status = SDMMC_ERROR_NORESPONSE;
        }
        else
        {
            TRACE_INFO( "iERR %x: C%x(%d), R%d\n\r", maskedSR, pCommand->cmd, (pCommand->cmd & HSMCI_CMDR_CMDNB), pCommand->resType ) ;
            /* If the command is SEND_OP_COND the CRC error flag is always
               present (cf: R3,R4 response, no CRC) */
            if (   ((pCommand->resType == 3)||(pCommand->resType == 4))
                && (maskedSR & STATUS_ERRORS) == HSMCI_SR_RCRCE)
            {
            }
            else
            {
                pCommand->status = SDMMC_ERROR;
            }
        }
    }
    mask &= ~STATUS_ERRORS;

    /* Check if there is data (RX) */
    if (maskedSR & HSMCI_SR_RXRDY) {
        *(uint32_t*)&hsmciXfr.pData[hsmciXfr.transferred] = pMciHw->HSMCI_RDR;
        hsmciXfr.transferred += 4;
        if (hsmciXfr.transferred >= hsmciXfr.size) {
            mask &= ~(uint32_t)HSMCI_SR_RXRDY;
        }
    }
  #ifndef HSMCI_WRITE_FIFO
    /* Check if data can be sent (TX) */
    if (maskedSR & HSMCI_SR_TXRDY) {
        hsmciXfr.transferred += 4;
        if (hsmciXfr.transferred >= hsmciXfr.size) {
            pMciHw->HSMCI_IDR = HSMCI_SR_TXRDY;
            mask &= ~(uint32_t)HSMCI_SR_TXRDY;
        }
        else {
            pMciHw->HSMCI_TDR =
                *(uint32_t*)&hsmciXfr.pData[hsmciXfr.transferred];
        }
    }
    /* Check if BLKE (TX done) */
    if (maskedSR & HSMCI_SR_BLKE) {
        mask &= ~(uint32_t)HSMCI_SR_BLKE;
    }
  #else
    /* Check if BLKE (TX one block) */
    if (maskedSR & HSMCI_SR_BLKE) {
        /* Write: */
        {
            hsmciXfr.fifoed -= pCommand->blockSize;
            hsmciXfr.transferred += pCommand->blockSize;
            /* Transfer finished */
            if (hsmciXfr.transferred >= hsmciXfr.size) {
                mask &= ~(uint32_t)HSMCI_SR_BLKE;
            }
            /* Send next block */
            else {
                FIFO_Write(pMciHw, &hsmciXfr, pCommand->blockSize);
            }
        }
    }
    /* Check if FIFO empty (TX & All data sent) */
    if (maskedSR & HSMCI_SR_FIFOEMPTY) {
        /* End command until no data on bus */
        if ((SR & HSMCI_SR_BLKE) == 0) {
            pMciHw->HSMCI_IER = HSMCI_IER_BLKE;
        }
        else {
            mask &= ~(uint32_t)HSMCI_IMR_FIFOEMPTY;
        }
    }
  #endif
    /* Check if a command has been completed */
    if (maskedSR & HSMCI_SR_CMDRDY) {
        /* Command OK, disable interrupt */
        pMciHw->HSMCI_IDR = HSMCI_IDR_CMDRDY;

        /* Check BUSY */
        if (pCommand->busyCheck
            && (pCommand->tranType == MCI_NO_TRANSFER
                || pCommand->tranType == MCI_STOP_TRANSFER) ) {
            /* Check XFRDONE to confirm no bus action */
            if (SR & HSMCI_SR_NOTBUSY)
                mask &= ~(uint32_t)HSMCI_IMR_CMDRDY;
            else {
                pMciHw->HSMCI_IER = HSMCI_SR_NOTBUSY;
            }
        } else
        {
            mask &= ~(uint32_t)HSMCI_IMR_CMDRDY;
        }
    }

    /* Check if transfer stopped */
    if (maskedSR & HSMCI_SR_XFRDONE) {
        mask &= ~(uint32_t)HSMCI_IMR_XFRDONE;
    }

    /* Check if not busy */
    if (maskedSR & HSMCI_SR_NOTBUSY) {
        mask &= ~(uint32_t)HSMCI_IMR_NOTBUSY;
    }

    /* All non-error mask done, complete the command */
    if (0 == mask || pCommand->state != SDMMC_CMD_PENDING) {
        /* Store the card response in the provided buffer */
        if (pCommand->pResp) {
            uint8_t resSize;
            switch (pCommand->resType) {
            case 1: case 3: case 4: case 5: case 6: case 7:
                     resSize = 1;           break;
            case 2:  resSize = 4;           break;
            default: resSize = 0;           break;
            }
            for (i=0; i < resSize; i++) {
                pCommand->pResp[i] = pMciHw->HSMCI_RSPR[0];
            }
        }

        /* If no error occured, the transfer is successful */
        if (pCommand->state == SDMMC_CMD_PENDING)
            pCommand->state = 0;

        /* Disable PDC */
        pMciHw->HSMCI_PTCR = HSMCI_PTCR_RXTDIS | HSMCI_PTCR_TXTDIS;

        /* Disable interrupts */
        pMciHw->HSMCI_IDR = pMciHw->HSMCI_IMR;

        /* Disable MCI */
        //MCI_DISABLE(pMciHw);

        /* Disable peripheral */
        PERIPH_DISABLE(pMci->mciId);

        /* Release the semaphore */
        pMci->semaphore++;

        /* Invoke the callback associated with the current command (if any) */
        if (pCommand->callback) {
            (pCommand->callback)(pCommand->status, pCommand->pArg);
        }
    }
}

/**
 * Returns 1 if the given MCI transfer is complete; otherwise returns 0.
 * \param pMci  Pointer to a Mcid instance.
 */
uint8_t MCI_IsCommandComplete(Mcid *pMci)
{
    MciCmd *pCommand = pMci->pCommand;

    if (pCommand) {
        if (pCommand->state == SDMMC_CMD_PENDING)
            return 0;
    }
    return 1;
}
