/*******************************************************************************
  SPI access routines for C32

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   drv_spi.h
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

#ifndef _DRV_SPI_H_
#define _DRV_SPI_H_

#include <stdint.h>
#include <stdbool.h>


#if defined(__C32__)
    #include <peripheral/ports.h>
    #include <peripheral/spi.h>
#endif

// Defines to calculate the SPI operating frequency
#if defined (__C30__)

	#define SPI_EN 			0x8000

	#define SPI_SMP_END		0x0200
	#define SPI_CLK_EDGE	0x0100
	#define SPI_CLK_POL		0x0040
	#define SPI_MSTEN		0x0020

#if defined (__PIC24F__)

	#define PRI_PRE_1_1		0x0003
	#define PRI_PRE_4_1		0x0002
	#define PRI_PRE_16_1 	0x0001	
	#define PRI_PRE_64_1    0x0000

	#define SEC_PRE_1_1		0x001C
	#define SEC_PRE_2_1		0x0018
	#define SEC_PRE_4_1 	0x0014	
	#define SEC_PRE_6_1     0x0010
	#define SEC_PRE_8_1     0x0000
 
// At present the below macros are calculated using the maximum Peripheral bus clock, i,e 40MHz
// As per the data sheet of 16bit devices the maximum spi frequency is limited to 10MHz
	#define FREQ_10MHZ 			( PRI_PRE_1_1 | SEC_PRE_2_1 )//10MHz
	#define FREQ_4_MHZ 			( PRI_PRE_1_1 | SEC_PRE_4_1 )//4MHz
	#define FREQ_2_667MHZ 		( PRI_PRE_1_1 | SEC_PRE_6_1 )//2.667MHz
	#define FREQ_2_MHZ 			( PRI_PRE_1_1 | SEC_PRE_8_1 )//2MHz
	#define FREQ_1_MHZ 			( PRI_PRE_4_1 | SEC_PRE_4_1 )//1MHz
	#define FREQ_667KHZ 		( PRI_PRE_4_1 | SEC_PRE_6_1 )//667MHz
	#define FREQ_500KHZ 		( PRI_PRE_4_1 | SEC_PRE_8_1 )//500KHz
	#define FREQ_250KHZ 		( PRI_PRE_16_1 | SEC_PRE_4_1 )//250KHz
	#define FREQ_167KHZ 		( PRI_PRE_16_1 | SEC_PRE_6_1 )//167KHz
	#define FREQ_125KHZ 		( PRI_PRE_16_1 | SEC_PRE_8_1 )//125KHz
	#define FREQ_63KHZ 			( PRI_PRE_64_1 | SEC_PRE_4_1 )//63KHz
	#define FREQ_42KHZ 			( PRI_PRE_64_1 | SEC_PRE_6_1 )//42KHz
	#define FREQ_31KHZ 			( PRI_PRE_64_1 | SEC_PRE_8_1 )//31KHz

#else

	#define PRI_PRE_1_1		0x0003
	#define PRI_PRE_4_1		0x0002
	#define PRI_PRE_16_1 	0x0001	
	#define PRI_PRE_64_1    0x0000

	#define SEC_PRE_1_1		0x001C
	#define SEC_PRE_2_1		0x0018
	#define SEC_PRE_4_1 	0x0014	
	#define SEC_PRE_6_1     0x0010
	#define SEC_PRE_8_1     0x0000
 
// At present the below macros are calculated using the maximum Peripheral bus clock, i,e 40MHz
// As per the data sheet of 16bit devices the maximum spi frequency is limited to 10MHz
	#define FREQ_10MHZ 		( PRI_PRE_4_1 | SEC_PRE_1_1 )//10MHz
	#define FREQ_6_66MHZ 	( PRI_PRE_1_1 | SEC_PRE_6_1 )//6.66MHz
	#define FREQ_5MHZ 		( PRI_PRE_1_1 | SEC_PRE_8_1 )//5MHz
	#define FREQ_2_5MHZ 	( PRI_PRE_4_1 | SEC_PRE_4_1 )//2.5MHz
	#define FREQ_1_66MHZ 	( PRI_PRE_4_1 | SEC_PRE_6_1 )//1.66MHz
	#define FREQ_1_25MHZ 	( PRI_PRE_4_1 | SEC_PRE_8_1 )//1.25MHz
	#define FREQ_625KHZ 	( PRI_PRE_16_1 | SEC_PRE_4_1 )//635KHz
	#define FREQ_416KHZ 	( PRI_PRE_16_1 | SEC_PRE_6_1 )//426KHz
	#define FREQ_312KHZ 	( PRI_PRE_16_1 | SEC_PRE_8_1 )//312KHz
	#define FREQ_156KHZ 	( PRI_PRE_64_1 | SEC_PRE_4_1 )//156KHz
	#define FREQ_104KHZ 	( PRI_PRE_64_1 | SEC_PRE_6_1 )//104KHz
	#define FREQ_78KHZ 		( PRI_PRE_64_1 | SEC_PRE_8_1 )//78KHz

#endif
// SPI Clock table for 16 bit devices only


typedef struct
{
	uint32_t clock;
	uint16_t scale;	
}
SpiClockTable;


#define SPI_CLK_TBL_ELEMENT_COUNT sizeof(SpiClkTbl)/sizeof(SpiClockTable)

// open flags
typedef enum
{
	// master opening mode
	SPI_OPEN_MSTEN = 		0,	// set the Master mode
	SPI_OPEN_SMP_END = 		1,	// Master Sample Phase for the input bit at the end of the data out time. Otherwise data is sampled in the middle.
	SPI_OPEN_MSSEN = 		2,	// enable the driving of the Slave Select (SS) output pin by the Master
	SPI_OPEN_MSSEN_HIGH = 	3,	// Master driven SS output active high. Otherwise low.
	
	// slave opening mode
	SPI_OPEN_SLVEN =		4,				// set the Slave mode
	SPI_OPEN_SSEN = 		5,	// enable the SS input pin. 
	
	// clocking opening mode
	SPI_OPEN_CKP_HIGH = 	6,	// set the clock polarity to (idle-high, active-low). Otherwise is (idle-low, active-high).
	SPI_OPEN_CKE_REV = 		7,	// set the Clock Edge reversed: transmit from active to idle. Otherwise transmit when clock goes from idle to active

	// data characters opening mode
	SPI_OPEN_MODE8 =		8,				// set 8 bits/char
	SPI_OPEN_MODE16 = 		9,	// set 16 bits/char
	SPI_OPEN_MODE32 = 		10,	// set 32 bits/char
	
	// framed mode opening mode
	SPI_OPEN_FRMEN = 		11,	// Enable the Framed SPI support. Otherwise the Framed SPI is disabled.
	SPI_OPEN_FSP_IN =		12,	// Frame Sync Pulse (FSP) direction set to input (Frame Slave).
       									// Otherwise the FSP is output and the SPI channel operates as a Frame Master.
	SPI_OPEN_FSP_HIGH = 	13,	// FSP polarity set active high. Otherwise the FSP is active low.
	SPI_OPEN_FSP_CLK1 =		14,	// Set the FSP to coincide with the 1st bit clock.
       									// Otherwise the FSP precedes the 1st bit clock
	SPI_OPEN_FSP_WIDE = 	15,	// set the FSP one character wide. Otherwise the FSP is one clock wide.
	
	SPI_OPEN_FRM_CNT1 =		16,	// set the number of characters per frame (Frame Counter) to 1 (default)
	SPI_OPEN_FRM_CNT2 =		17,	// set the Frame Counter to 2 
	SPI_OPEN_FRM_CNT4 =		18,	// set the Frame Counter to 4
	SPI_OPEN_FRM_CNT8 =		19,	// set the Frame Counter to 8
	SPI_OPEN_FRM_CNT16 =	20,	// set the Frame Counter to 16
	SPI_OPEN_FRM_CNT32 =	21,	// set the Frame Counter to 32

	// enhanced buffer (FIFO) opening mode
	SPI_OPEN_ENHBUF = 		22,	// enable the enhanced buffer mode

	SPI_OPEN_TBE_NOT_FULL =	23,	// Tx Buffer event issued when Tx buffer not full (at least one slot empty)
	SPI_OPEN_TBE_HALF_EMPTY =	24,	// Tx Buffer event issued when Tx buffer >= 1/2 empty
	SPI_OPEN_TBE_EMPTY =		25,	// Tx Buffer event issued when Tx buffer completely empty
	SPI_OPEN_TBE_SR_EMPTY =	26,	// Tx Buffer event issued when the last character is shifted out of the internal Shift Register
											// and the transmit is complete
	
	SPI_OPEN_RBF_FULL =		27,	// Rx Buffer event issued when RX buffer is full
	SPI_OPEN_RBF_HALF_FULL =	28,	// Rx Buffer event issued when RX buffer is >= 1/2 full
	SPI_OPEN_RBF_NOT_EMPTY =	29,	// Rx Buffer event issued when RX buffer is not empty
	SPI_OPEN_RBF_EMPTY =		30,	// Rx Buffer event issued when RX buffer is empty (the last character in the buffer is read).

	// general opening mode
	SPI_OPEN_DISSDO = 		31,	// disable the usage of the SDO pin by the SPI
	SPI_OPEN_SIDL = 		32,	// enable the Halt in the CPU Idle mode. Otherwise the SPI will be still active when the CPU is in Idle mode. 
	SPI_OPEN_FRZ = 		33,	// Debug mode only: enable the Freeze operation while in Debug. Otherwise continue to operate.
	SPI_OPEN_ON = 		34,	// turn ON the SPI (not used in SpiChnOpen)
}SpiOpenFlags;	// open flags that can be used with SpiChnOpen. Defined in the processor header file.


typedef enum
{
	DRV_SPI_CHANNEL1	= 1,
	DRV_SPI_CHANNEL2	= 2
#if defined (__PIC32MX__)
	DRV_SPI_CHANNEL3	= 3,
	DRV_SPI_CHANNEL4	= 4	
#endif
}SpiChannel;

#endif
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
bool DRV_SPI_Initialize(SpiChannel chn, SpiOpenFlags oFlags, unsigned int fpbDiv);

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
void DRV_SPI_TxRx(SpiChannel chn, uint8_t   *pTxBuf, uint16_t  txLen, uint8_t   *pRxBuf, uint16_t  rxLen);


#endif

