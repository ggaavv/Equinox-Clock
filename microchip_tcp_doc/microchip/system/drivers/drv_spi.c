/*******************************************************************************
  SPI access routines for PIC32

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   SPI.c
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
#include "Compiler.h"
#include "system/drivers/drv_spi.h"
#include "system/system_services.h"

#if defined(__PIC32__)
    #include <plib.h>
#elif defined (__C30__)
    // C30 include
#endif

#if defined (__C30__)

// SPI look-up table for the operating frequency. This table is valid for peripheral clock of 40MHz.
// The table may require corresponding updates based on the different peripheral clock.
#if defined (__PIC24F__)
const SpiClockTable SpiClkTbl[] = 	{
										{8000000, FREQ_10MHZ},
										{4000000, FREQ_4_MHZ},
										{2667000, FREQ_2_667MHZ},
										{2000000, FREQ_2_MHZ},
										{1000000, FREQ_1_MHZ},
										{ 667000, FREQ_667KHZ},
										{ 500000, FREQ_500KHZ},
										{ 250000, FREQ_250KHZ},
										{ 167000, FREQ_167KHZ},
										{ 125000, FREQ_125KHZ},
										{  63000, FREQ_63KHZ},
										{  42000, FREQ_42KHZ},
										{  31000, FREQ_31KHZ},
									};
#else
const SpiClockTable SpiClkTbl[] = 	{
										{10000000, FREQ_10MHZ},
										{ 6660000, FREQ_6_66MHZ},
										{ 5000000, FREQ_5MHZ},
										{ 2500000, FREQ_2_5MHZ},
										{ 1660000, FREQ_1_66MHZ},
										{ 1250000, FREQ_1_25MHZ},
										{  625000, FREQ_625KHZ},
										{  416000, FREQ_416KHZ},
										{  312000, FREQ_312KHZ},
										{  156000, FREQ_156KHZ},
										{  104000, FREQ_104KHZ},
										{   78000, FREQ_78KHZ}
									};

#endif
#endif // (__C30__)


static void WaitForDataByte(SpiChannel chn)
{
#if defined (__C32__)
	while (!SpiChnTxBuffEmpty(chn) || !SpiChnDataRdy(chn)); 
#elif defined (__C30__)
	switch(chn)
	{
		case 1:
			while ((SPI1STATbits.SPITBF == 1) || (SPI1STATbits.SPIRBF == 0));
		break;
		case 2:
			while ((SPI2STATbits.SPITBF == 1) || (SPI2STATbits.SPIRBF == 0));
		break;
	}

#endif

}


/****************************************************************************
  Function:
    bool DRV_SPI_Initialize(SpiChannel chn, SpiOpenFlags oFlags, unsigned int fpbDiv)

  Summary:
    This function initializes the SPI channel and also sets the brg register.

  Description:
    This function initializes the SPI channel and also sets the brg register.
 	The SPI baudrate BR is given by: BR=Fpb/(2*(SPIBRG+1))
 	The input parametes fpbDiv specifies the Fpb divisor term (2*(SPIBRG+1)),
 	so the BRG is calculated as SPIBRG=fpbDiv/2-1.

    
  Precondition:
    None

  Parameters:
	chn 	- the channel to set
	oFlags	- a SpiOpenFlags or __SPIxCONbits_t structure that sets the module behavior
	fpbDiv	- Fpb divisor to extract the baud rate: BR=Fpb/fpbDiv.


  Returns:
     true if success
     false otherwise

  Remarks:
    - The baud rate is always obtained by dividing the Fpb to an even number
      between 2 and 1024.
	- When selecting the number of bits per character, SPI_OPEN_MODE32 has the highest priority.
	  If SPI_OPEN_MODE32 is not set, then SPI_OPEN_MODE16 selects the character width.
 	- The SPI_OPEN_SSEN is taken into account even in master mode. If it is set the library
      will properly se the SS pin as an digital output.
  ***************************************************************************/	
bool DRV_SPI_Initialize(SpiChannel chn, SpiOpenFlags oFlags, unsigned int fpbDiv)
{
#if defined (__C32__)
    SpiChnOpen(chn, oFlags, fpbDiv);
#elif defined (__C30__) 
    volatile uint16_t con1 = 0; 
    uint16_t con2 = 0;
    uint16_t con3 = 0;
    uint8_t i;

    if((SYS_CLK_PeripheralClockGet()/fpbDiv) > 10000000ul)
    {
        SYS_ASSERT(false, "Requested SPI frequency is not supported!");
        return false;
        // the SPI clock is selected more than 10MHz.
        // Select the frequency as per the data sheet of the particular 16bit device.	
    }

    for(i = 0; i < SPI_CLK_TBL_ELEMENT_COUNT; i++)
    {
        if((SYS_CLK_PeripheralClockGet()/fpbDiv) <= SpiClkTbl[i].clock)
        {
            con1 = SpiClkTbl[i].scale;
            break;
        }
    }
	
	con1 |= oFlags;
	con3 |= SPI_EN;

    switch(chn)
    {
        case 1:
			OpenSPI1(con1,con2,con3); 
            break;
            
        case 2:
            SPI2STAT &= 0x7FFF;
            OpenSPI2(con1,con2,con3);
            break;

        default:
            SYS_ASSERT(false, "Requested SPI channel is not supported!");
            return false;
    }
#endif

    return true;
}

/*****************************************************************************
  Function:
	void DRV_SPI_TxRx(void)

  Summary:
	Transmits and receives SPI bytes

  Description:
	Transmits and receives N bytes of SPI data.

  Precondition:
	None

  Parameters:
	pTxBuf - pointer to SPI tx data
	txLen   - number of bytes to Tx
	pRxBuf - pointer to where SPI rx data will be stored
	rxLen   - number of SPI rx bytes caller wants copied to p_rxBuf

  Returns:
  	None
  	
  Remarks:
	Will clock out the larger of txLen or rxLen, and pad if necessary.
*****************************************************************************/
void DRV_SPI_TxRx(SpiChannel chn,
			   uint8_t   *pTxBuf, 
               uint16_t  txLen, 
               uint8_t   *pRxBuf,
               uint16_t  rxLen)
{
	uint16_t byteCount;
	uint16_t i;
	uint8_t  rxTrash;

    /* total number of byte to clock is whichever is larger, txLen or rxLen */
    byteCount = (txLen >= rxLen)?txLen:rxLen;
    
    for (i = 0; i < byteCount; ++i)
    {
        /* if still have bytes to transmit from tx buffer */
        if ((txLen > 0) && (pTxBuf != 0))
        {
			#if defined (__C32__)
            	SpiChnWriteC(chn, *pTxBuf++);
			#elif defined (__C30__)
				switch(chn)
				{
					case 1:
						WriteSPI1(*pTxBuf++);
						break;
					case 2:
						WriteSPI2(*pTxBuf++);
						break;
				}
			#endif
            --txLen;
        }
        /* else done writing bytes out from tx buffer */
        else
        { 
			#if defined (__C32__)

	            SpiChnWriteC(chn, 0); /* clock out a "don't care" byte */
			#elif defined (__C30__)

				switch(chn)
				{
					case 1:
						WriteSPI1(0x00);
						break;
					case 2:
						WriteSPI2(0x00);
						break;
				}
			#endif	
        }  

        /* wait until tx/rx byte to completely clock out */
        WaitForDataByte(chn);
        
        /* if still have bytes to read into rx buffer */
        if ((rxLen > 0) && (pRxBuf != 0))
        {
			#if defined (__C32__)

	            *pRxBuf++ = SpiChnReadC(chn);
			#elif defined (__C30__)

				switch(chn)
				{
					case 1:
						*pRxBuf++ = ReadSPI1();
						break;
					case 2:
						*pRxBuf++ = ReadSPI2();
						break;

				}
			#endif
            --rxLen;
        }
        /* else done reading bytes into rx buffer */ 
        else
        {
			#if defined (__C32__)

	            rxTrash = SpiChnReadC(chn); /* read and throw away byte */
			#elif defined (__C30__)

				switch(chn)
				{
					case 1:
						rxTrash = ReadSPI1();
						break;
					case 2:
						rxTrash = ReadSPI2();
						break;

				}
			#endif
        }    
    }  /* end for loop */  
}


