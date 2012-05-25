/*******************************************************************************
  Medium Access Control (MAC) Layer for Microchip ENC28J60

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Provides access to ENC28J60 Ethernet controller
    - Reference: ENC28J60 Data sheet, IEEE 802.3 Standard
*******************************************************************************/

/*******************************************************************************
FileName:   ENC28J60.c
Copyright 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND,
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
#define __ENC28J60_C

#include "tcpip_private.h"

// Make sure that this hardware profile has an ENC28J60 in it
#if defined(TCPIP_IF_ENC28J60)
#include "tcpip/enc28j60.h"
#include "system/drivers/drv_spi.h"

#define Enc28InitChipSelect()	 {ENC28_CS_TRIS = 0;}
#define Enc28EnableChipSelect()  {ENC28_CS_IO	= 0;}
#define Enc28DisableChipSelect() {ENC28_CS_IO	= 1;}
#if defined(ENC28_RST_IO)
#define Enc28InitChipReset()	 {ENC28_RST_TRIS = 0;}
#define Enc28EnableChipReset()	 {ENC28_RST_IO   = 0;}
#define Enc28DisableChipReset()	 {ENC28_RST_IO	 = 1;}
#endif

// Prototypes of functions intended for MAC layer use only.
static void BankSel(uint16_t Register);
static REG ReadETHReg(uint8_t Address);
static REG ReadMACReg(uint8_t Address);
static void WriteReg(uint8_t Address, uint8_t Data);
static void BFCReg(uint8_t Address, uint8_t Data);
static void BFSReg(uint8_t Address, uint8_t Data);
static PHYREG ReadPHYReg(uint8_t Register);
static void WritePHYReg(uint8_t Register, uint16_t Data);

static void ENC28J60_SendSystemReset(void);

//static void GetRegs(void);
//void Get8KBRAM(void);

// Internal MAC level variables and flags.
static TCPIP_UINT16_VAL NextPacketLocation;
static TCPIP_UINT16_VAL CurrentPacketLocation;
static bool WasDiscarded;
static uint8_t ENCRevID;

/******************************************************************************
 * Macro:        	void SetLEDConfig(uint16_t NewConfig)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *
 * Input:           NewConfig - xxx0: Pulse stretching disabled
 *								xxx2: Pulse stretch to 40ms (default)
 *								xxx6: Pulse stretch to 73ms
 *								xxxA: Pulse stretch to 139ms
 *
 *								xx1x: LEDB - TX
 *								xx2x: LEDB - RX (default)
 *								xx3x: LEDB - collisions
 *								xx4x: LEDB - link
 *								xx5x: LEDB - duplex
 *								xx7x: LEDB - TX and RX
 *								xx8x: LEDB - on
 *								xx9x: LEDB - off
 *								xxAx: LEDB - blink fast
 *								xxBx: LEDB - blink slow
 *								xxCx: LEDB - link and RX
 *								xxDx: LEDB - link and TX and RX
 *								xxEx: LEDB - duplex and collisions
 *
 *								x1xx: LEDA - TX
 *								x2xx: LEDA - RX
 *								x3xx: LEDA - collisions
 *								x4xx: LEDA - link (default)
 *								x5xx: LEDA - duplex
 *								x7xx: LEDA - TX and RX
 *								x8xx: LEDA - on
 *								x9xx: LEDA - off
 *								xAxx: LEDA - blink fast
 *								xBxx: LEDA - blink slow
 *								xCxx: LEDA - link and RX
 *								xDxx: LEDA - link and TX and RX
 *								xExx: LEDA - duplex and collisions
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Writes the value of NewConfig into the PHLCON PHY register.
 *					The LED pins will beginning outputting the new
 *					configuration immediately.
 *
 * Note:
 *****************************************************************************/
#define ENC28J60_SetLEDConfig(NewConfig)		WritePHYReg(PHLCON, NewConfig)


/******************************************************************************
 * Macro:        	uint16_t GetLEDConfig(void)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *
 * Input:           None
 *
 * Output:          uint16_t -	xxx0: Pulse stretching disabled
 *							xxx2: Pulse stretch to 40ms (default)
 *							xxx6: Pulse stretch to 73ms
 *							xxxA: Pulse stretch to 139ms
 *
 *							xx1x: LEDB - TX
 *							xx2x: LEDB - RX (default)
 *							xx3x: LEDB - collisions
 *							xx4x: LEDB - link
 *							xx5x: LEDB - duplex
 *							xx7x: LEDB - TX and RX
 *							xx8x: LEDB - on
 *							xx9x: LEDB - off
 *							xxAx: LEDB - blink fast
 *							xxBx: LEDB - blink slow
 *							xxCx: LEDB - link and RX
 *							xxDx: LEDB - link and TX and RX
 *							xxEx: LEDB - duplex and collisions
 *
 * 							x1xx: LEDA - TX
 *							x2xx: LEDA - RX
 *							x3xx: LEDA - collisions
 *							x4xx: LEDA - link (default)
 *							x5xx: LEDA - duplex
 *							x7xx: LEDA - TX and RX
 *							x8xx: LEDA - on
 *							x9xx: LEDA - off
 *							xAxx: LEDA - blink fast
 *							xBxx: LEDA - blink slow
 *							xCxx: LEDA - link and RX
 *							xDxx: LEDA - link and TX and RX
 *							xExx: LEDA - duplex and collisions
 *
 * Side Effects:    None
 *
 * Overview:        Returns the current value of the PHLCON register.
 *
 * Note:            None
 *****************************************************************************/
#define ENC28J60_GetLEDConfig()		ReadPHYReg(PHLCON).Val


//NOTE: All code in this module expects Bank 0 to be currently selected.  If code ever changes the bank, it must restore it to Bank 0 before returning.

/******************************************************************************
 * Function:        void ENC28J60_MACInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        MACInit sets up the PIC's SPI module and all the
 *                  registers in the ENC28J60 so that normal operation can
 *                  begin.
 *
 * Note:            None
 *****************************************************************************/
TCPIP_MAC_RES ENC28J60_MACInit(NET_CONFIG* pNetIf)
{
    uint8_t i;
	SYS_TICK startTick;

    // Set up the SPI module on the PIC for communications with the ENC28J60
	Enc28InitChipSelect();
    Enc28DisableChipSelect();

    // If the RESET pin is connected, take the chip out of reset
#if defined(ENC28_RST_IO)
	Enc28InitChipReset();
    Enc28DisableChipReset();
#endif

    // Set up SPI
	// Delay SDI input sampling (PIC perspective) by 1/2 SPI clock
	#if defined(__PIC32MX__)
	if(!DRV_SPI_Initialize(ENC28_SPI_CHN, 
					SPI_OPEN_MSTEN|SPI_OPEN_MODE8, 
					SYS_CLK_PeripheralClockGet()/ENC_MAX_SPI_FREQ))
	#elif defined(__C30__)
	if(!DRV_SPI_Initialize(ENC28_SPI_CHN, 
					SPI_SMP_END|SPI_CLK_POL|SPI_MSTEN, 
					SYS_CLK_PeripheralClockGet()/ENC_MAX_SPI_FREQ))
	#endif
    {
        return TCPIP_MAC_RES_INIT_FAIL;
    }

    // RESET the entire ENC28J60, clearing all registers
    // Also wait for CLKRDY to become set.
    // Bit 3 in ESTAT is an unimplemented bit.  If it reads out as '1' that
    // means the part is in RESET or there is something wrong with the SPI
    // connection.  This loop makes sure that we can communicate with the
    // ENC28J60 before proceeding.
    startTick = SYS_TICK_Get();
    do
    {
        ENC28J60_SendSystemReset();
        i = ReadETHReg(ESTAT).Val;
		// Init Time out error
		if(SYS_TICK_Get() - startTick > 1*SYS_TICK_TicksPerSecondGet())
		{
			return TCPIP_MAC_RES_PHY_INIT_FAIL;
		}		
    } while((i & 0x08) || (~i & ESTAT_CLKRDY));

    // Start up in Bank 0 and configure the receive buffer boundary pointers
    // and the buffer write protect pointer (receive buffer read pointer)
    WasDiscarded = true;
    NextPacketLocation.Val = RXSTART;

    WriteReg(ERXSTL, LOW(RXSTART));
    WriteReg(ERXSTH, HIGH(RXSTART));
    WriteReg(ERXRDPTL, LOW(RXSTOP));    // Write low byte first
    WriteReg(ERXRDPTH, HIGH(RXSTOP));   // Write high byte last
    WriteReg(ERXNDL, LOW(RXSTOP));
    WriteReg(ERXNDH, HIGH(RXSTOP));
    WriteReg(ETXSTL, LOW(TXSTART));
    WriteReg(ETXSTH, HIGH(TXSTART));

    // Write a permanant per packet control byte of 0x00
    WriteReg(EWRPTL, LOW(TXSTART));
    WriteReg(EWRPTH, HIGH(TXSTART));
    ENC28J60_MACPut(0x00);


    // Enter Bank 1 and configure Receive Filters
    // (No need to reconfigure - Unicast OR Broadcast with CRC checking is
    // acceptable)
    // Write ERXFCON_CRCEN only to ERXFCON to enter promiscuous mode

    // Promiscious mode example:
    BankSel(ERXFCON);
    WriteReg((uint8_t)ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_MCEN | ERXFCON_BCEN);

    // Enter Bank 2 and configure the MAC
    BankSel(MACON1);

    // Enable the receive portion of the MAC
    WriteReg((uint8_t)MACON1, MACON1_TXPAUS | MACON1_RXPAUS | MACON1_MARXEN);

    // Pad packets to 60 bytes, add CRC, and check Type/Length field.
#if defined(FULL_DUPLEX)
    WriteReg((uint8_t)MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN | MACON3_FULDPX);
    WriteReg((uint8_t)MABBIPG, 0x15);
#else
    WriteReg((uint8_t)MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN);
    WriteReg((uint8_t)MABBIPG, 0x12);
#endif

    // Allow infinite deferals if the medium is continuously busy
    // (do not time out a transmission if the half duplex medium is
    // completely saturated with other people's data)
    WriteReg((uint8_t)MACON4, MACON4_DEFER);

    // Late collisions occur beyond 63+8 bytes (8 bytes for preamble/start of frame delimiter)
    // 55 is all that is needed for IEEE 802.3, but ENC28J60 B5 errata for improper link pulse
    // collisions will occur less often with a larger number.
    WriteReg((uint8_t)MACLCON2, 63);

    // Set non-back-to-back inter-packet gap to 9.6us.  The back-to-back
    // inter-packet gap (MABBIPG) is set by MACSetDuplex() which is called
    // later.
    WriteReg((uint8_t)MAIPGL, 0x12);
    WriteReg((uint8_t)MAIPGH, 0x0C);

    // Set the maximum packet size which the controller will accept
    WriteReg((uint8_t)MAMXFLL, LOW(6+6+2+1500+4));  // 1518 is the IEEE 802.3 specified limit
    WriteReg((uint8_t)MAMXFLH, HIGH(6+6+2+1500+4)); // 1518 is the IEEE 802.3 specified limit

    // Enter Bank 3 and initialize physical MAC address registers
    BankSel(MAADR1);
    WriteReg((uint8_t)MAADR1, pNetIf->MyMACAddr.v[0]);
    WriteReg((uint8_t)MAADR2, pNetIf->MyMACAddr.v[1]);
    WriteReg((uint8_t)MAADR3, pNetIf->MyMACAddr.v[2]);
    WriteReg((uint8_t)MAADR4, pNetIf->MyMACAddr.v[3]);
    WriteReg((uint8_t)MAADR5, pNetIf->MyMACAddr.v[4]);
    WriteReg((uint8_t)MAADR6, pNetIf->MyMACAddr.v[5]);

    // Disable the CLKOUT output to reduce EMI generation
    WriteReg((uint8_t)ECOCON, 0x00);   // Output off (0V)
    //WriteReg((uint8_t)ECOCON, 0x01); // 25.000MHz
    //WriteReg((uint8_t)ECOCON, 0x03); // 8.3333MHz (*4 with PLL is 33.3333MHz)

    // Get the Rev ID so that we can implement the correct errata workarounds
    ENCRevID = ReadETHReg((uint8_t)EREVID).Val;

    // Disable half duplex loopback in PHY.  Bank bits changed to Bank 2 as a
    // side effect.
    WritePHYReg(PHCON2, PHCON2_HDLDIS);

    // Configure LEDA to display LINK status, LEDB to display TX/RX activity
    ENC28J60_SetLEDConfig(0x3472);

    // Set the MAC and PHY into the proper duplex state
#if defined(FULL_DUPLEX)
    WritePHYReg(PHCON1, PHCON1_PDPXMD);
#elif defined(HALF_DUPLEX)
    WritePHYReg(PHCON1, 0x0000);
#else
    // Use the external LEDB polarity to determine weather full or half duplex
    // communication mode should be set.
    {
        REG Register;
        PHYREG PhyReg;

        // Read the PHY duplex mode
        PhyReg = ReadPHYReg(PHCON1);
        DuplexState = PhyReg.PHCON1bits.PDPXMD;

        // Set the MAC to the proper duplex mode
        BankSel(MACON3);
        Register = ReadMACReg((uint8_t)MACON3);
        Register.MACON3bits.FULDPX = PhyReg.PHCON1bits.PDPXMD;
        WriteReg((uint8_t)MACON3, Register.Val);

        // Set the back-to-back inter-packet gap time to IEEE specified
        // requirements.  The meaning of the MABBIPG value changes with the duplex
        // state, so it must be updated in this function.
        // In full duplex, 0x15 represents 9.6us; 0x12 is 9.6us in half duplex
        WriteReg((uint8_t)MABBIPG, PhyReg.PHCON1bits.PDPXMD ? 0x15 : 0x12);
    }
#endif

    BankSel(ERDPTL);        // Return to default Bank 0

    // Enable packet reception
    BFSReg(ECON1, ECON1_RXEN);

	return TCPIP_MAC_RES_OK;
}//end MACInit


/******************************************************************************
 * Function:        bool ENC28J60_MACIsLinked(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          true:  link is up
 *                  false: link is down
 *
 * Side Effects:    None
 *
 * Overview:        Returns the PHSTAT2.LSTAT bit.
 *
 * Note:            None
 *****************************************************************************/
bool ENC28J60_MACIsLinked(void)
{
    // LLSTAT is a latching low link status bit.  Therefore, if the link
    // goes down and comes back up before a higher level stack program calls
    // MACIsLinked(), MACIsLinked() will still return false.  The next
    // call to MACIsLinked() will return true (unless the link goes down
    // again).
    return ReadPHYReg(PHSTAT2).PHSTAT2bits.LSTAT;
}


/******************************************************************************
 * Function:        bool ENC28J60_MACCheckLink(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          true: If the PHY reports that a link partner is present
 *                        and the link has been up continuously since the last
 *                        call to MACIsLinked()
 *                  false: If the PHY reports no link partner, or the link went
 *                         down momentarily since the last call to MACIsLinked()
 *
 * Side Effects:    None
 *
 * Overview:        Returns the PHSTAT1.LLSTAT bit.
 *
 * Note:            None
 *****************************************************************************/
bool ENC28J60_MACCheckLink(void)
{
    // LLSTAT is a latching low link status bit.  Therefore, if the link
    // goes down and comes back up before a higher level stack program calls
    // MACIsLinked(), MACIsLinked() will still return false.  The next
    // call to MACIsLinked() will return true (unless the link goes down
    // again).
    return ReadPHYReg(PHSTAT1).PHSTAT1bits.LLSTAT;
}


/******************************************************************************
 * Function:        bool ENC28J60_MACIsTxReady(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          true: If no Ethernet transmission is in progress
 *                  false: If a previous transmission was started, and it has
 *                         not completed yet.  While false, the data in the
 *                         transmit buffer and the TXST/TXND pointers must not
 *                         be changed.
 *
 * Side Effects:    None
 *
 * Overview:        Returns the ECON1.TXRTS bit
 *
 * Note:            None
 *****************************************************************************/
bool ENC28J60_MACIsTxReady(void)
{
    return !ReadETHReg(ECON1).ECON1bits.TXRTS;
}


/******************************************************************************
 * Function:        void ENC28J60_MACDiscardRx(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Marks the last received packet (obtained using
 *                  MACGetHeader())as being processed and frees the buffer
 *                  memory associated with it
 *
 * Note:            Is is safe to call this function multiple times between
 *                  MACGetHeader() calls.  Extra packets won't be thrown away
 *                  until MACGetHeader() makes it available.
 *****************************************************************************/
void ENC28J60_MACDiscardRx(void)
{
    TCPIP_UINT16_VAL NewRXRDLocation;

    // Make sure the current packet was not already discarded
    if(WasDiscarded)
        return;
    WasDiscarded = true;

    // Decrement the next packet pointer before writing it into
    // the ERXRDPT registers.  This is a silicon errata workaround.
    // RX buffer wrapping must be taken into account if the
    // NextPacketLocation is precisely RXSTART.
    NewRXRDLocation.Val = NextPacketLocation.Val - 1;
    if(NewRXRDLocation.Val > RXSTOP)
    {
        NewRXRDLocation.Val = (uint16_t)RXSTOP;
    }

    // Decrement the RX packet counter register, EPKTCNT
    BFSReg(ECON2, ECON2_PKTDEC);

    // Move the receive read pointer to unwrite-protect the memory used by the
    // last packet.  The writing order is important: set the low byte first,
    // high byte last.
    WriteReg(ERXRDPTL, NewRXRDLocation.v[0]);
    WriteReg(ERXRDPTH, NewRXRDLocation.v[1]);
}


/******************************************************************************
 * Function:        uint16_t ENC28J60_MACGetFreeRxSize(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          A uint16_t estimate of how much RX buffer space is free at
 *                  the present time.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 *****************************************************************************/
uint16_t ENC28J60_MACGetFreeRxSize(void)
{
    TCPIP_UINT16_VAL ReadPT, WritePT;

    // Read the Ethernet hardware buffer write pointer.  Because packets can be
    // received at any time, it can change between reading the low and high
    // bytes.  A loop is necessary to make certain a proper low/high byte pair
    // is read.
    BankSel(EPKTCNT);
    do {
        // Save EPKTCNT in a temporary location
        ReadPT.v[0] = ReadETHReg((uint8_t)EPKTCNT).Val;

        BankSel(ERXWRPTL);
        WritePT.v[0] = ReadETHReg(ERXWRPTL).Val;
        WritePT.v[1] = ReadETHReg(ERXWRPTH).Val;

        BankSel(EPKTCNT);
    } while(ReadETHReg((uint8_t)EPKTCNT).Val != ReadPT.v[0]);

    // Determine where the write protection pointer is
    BankSel(ERXRDPTL);
    ReadPT.v[0] = ReadETHReg(ERXRDPTL).Val;
    ReadPT.v[1] = ReadETHReg(ERXRDPTH).Val;

    // Calculate the difference between the pointers, taking care to account
    // for buffer wrapping conditions
    if(WritePT.Val > ReadPT.Val)
    {
        return (RXSTOP - RXSTART) - (WritePT.Val - ReadPT.Val);
    }
    else if(WritePT.Val == ReadPT.Val)
    {
        return (uint16_t)(RXSIZE - 1);
    }
    else
    {
        return ReadPT.Val - WritePT.Val - 1;
    }
}

/******************************************************************************
 * Function:        bool ENC28J60_MACGetHeader(MAC_ADDR *remote, uint8_t* type)
 *
 * PreCondition:    None
 *
 * Input:           *remote: Location to store the Source MAC address of the
 *                           received frame.
 *                  *type: Location of a uint16_t to store the constant
 *                         ETHERTYPE_UNKNOWN, ETHERTYPE_IPVx, or ETHERTYPE_ARP, 
 *                         representing the contents of the Ethernet type
 *                         field.
 *
 * Output:          true: If a packet was waiting in the RX buffer.  The
 *                        remote, and type values are updated.
 *                  false: If a packet was not pending.  remote and type are
 *                         not changed.
 *
 * Side Effects:    Last packet is discarded if MACDiscardRx() hasn't already
 *                  been called.
 *
 * Overview:        None
 *
 * Note:            None
 *****************************************************************************/
bool ENC28J60_MACGetHeader(MAC_ADDR *remote, uint16_t* type)
{
    ENC_PREAMBLE header;
    uint8_t PacketCount;

    // Test if at least one packet has been received and is waiting
    BankSel(EPKTCNT);
    PacketCount = ReadETHReg((uint8_t)EPKTCNT).Val;
    BankSel(ERDPTL);
    if(PacketCount == 0u)
        return false;

    // Make absolutely certain that any previous packet was discarded
    if(WasDiscarded == false)
    {
        ENC28J60_MACDiscardRx();
        return false;
    }

    // Set the SPI read pointer to the beginning of the next unprocessed packet
    CurrentPacketLocation.Val = NextPacketLocation.Val;
    WriteReg(ERDPTL, CurrentPacketLocation.v[0]);
    WriteReg(ERDPTH, CurrentPacketLocation.v[1]);

    // Obtain the MAC header from the Ethernet buffer
    ENC28J60_MACGetArray((uint8_t*)&header, sizeof(header));

    // The EtherType field, like most items transmitted on the Ethernet medium
    // are in big endian.
    header.Type.Val = swaps(header.Type.Val);

    // Validate the data returned from the ENC28J60.  Random data corruption,
    // such as if a single SPI bit error occurs while communicating or a
    // momentary power glitch could cause this to occur in rare circumstances.
    if(header.NextPacketPointer > RXSTOP || ((TCPIP_UINT8_VAL*)(&header.NextPacketPointer))->bits.b0 ||
       header.StatusVector.bits.Zero ||
       header.StatusVector.bits.CRCError ||
       header.StatusVector.bits.ByteCount > 1518u ||
       !header.StatusVector.bits.ReceiveOk)
    {
        Reset();
    }

    // Save the location where the hardware will write the next packet to
    NextPacketLocation.Val = header.NextPacketPointer;

    // Return the Ethernet frame's Source MAC address field to the caller
    // This parameter is useful for replying to requests without requiring an
    // ARP cycle.
    memcpy((void*)remote->v, (void*)header.SourceMACAddr.v, sizeof(*remote));

    // Return a simplified version of the EtherType field to the caller
    *type = ETHERTYPE_UNKNOWN;
    if( (header.Type.Val == ETHERTYPE_IPV4) || (header.Type.Val == ETHERTYPE_IPV6) || 
        (header.Type.Val == ETHERTYPE_ARP) )
    {
        *type = header.Type.Val;
    }

    // Mark this packet as discardable
    WasDiscarded = false;
    return true;
}


/******************************************************************************
 * Function:        void ENC28J60_MACPutHeader(MAC_ADDR *remote, uint16_t type, uint16_t dataLen)
 *
 * PreCondition:    MACIsTxReady() must return true.
 *
 * Input:           *remote: Pointer to memory which contains the destination
 *                           MAC address (6 bytes)
 *                  type - packet type: ETHERTYPE_IPV4/6, ETHERTYPE_ARP
 *                  dataLen: Length of the Ethernet data payload
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            Because of the dataLen parameter, it is probably
 *                  advantagous to call this function immediately before
 *                  transmitting a packet rather than initially when the
 *                  packet is first created.  The order in which the packet
 *                  is constructed (header first or data first) is not
 *                  important.
 *****************************************************************************/
void ENC28J60_MACPutHeader(NET_CONFIG* pNetIf, MAC_ADDR *remote, uint16_t type, uint16_t dataLen)
{
    // Set the SPI write pointer to the beginning of the transmit buffer (post per packet control byte)
    WriteReg(EWRPTL, LOW(TXSTART+1));
    WriteReg(EWRPTH, HIGH(TXSTART+1));

    // Calculate where to put the TXND pointer
    dataLen += (uint16_t)sizeof(ETHER_HEADER) + TXSTART;

    // Write the TXND pointer into the registers, given the dataLen given
    WriteReg(ETXNDL, ((TCPIP_UINT16_VAL*)&dataLen)->v[0]);
    WriteReg(ETXNDH, ((TCPIP_UINT16_VAL*)&dataLen)->v[1]);

    // Set the per-packet control byte and write the Ethernet destination
    // address
    ENC28J60_MACPutArray((uint8_t*)remote, sizeof(*remote));

    // Write our MAC address in the Ethernet source field
    ENC28J60_MACPutArray((uint8_t*)&pNetIf->MyMACAddr, sizeof(pNetIf->MyMACAddr));

    // Write the appropriate Ethernet Type uint16_t for the protocol being used
    ENC28J60_MACPut((type >> 8) & 0xFF);
    ENC28J60_MACPut(type & 0xFF);
}

/******************************************************************************
 * Function:        void ENC28J60_MACFlush(void)
 *
 * PreCondition:    A packet has been created by calling MACPut() and
 *                  MACPutHeader().
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        MACFlush causes the current TX packet to be sent out on
 *                  the Ethernet medium.  The hardware MAC will take control
 *                  and handle CRC generation, collision retransmission and
 *                  other details.
 *
 * Note:            After transmission completes (MACIsTxReady() returns true),
 *                  the packet can be modified and transmitted again by calling
 *                  MACFlush() again.  Until MACPutHeader() or MACPut() is
 *                  called (in the TX data area), the data in the TX buffer
 *                  will not be corrupted.
 *****************************************************************************/
void ENC28J60_MACFlush(void)
{
    // Reset transmit logic if a TX Error has previously occured
    // This is a silicon errata workaround
    BFSReg(ECON1, ECON1_TXRST);
    BFCReg(ECON1, ECON1_TXRST);
    BFCReg(EIR, EIR_TXERIF | EIR_TXIF);

    // Start the transmission
    // After transmission completes (MACIsTxReady() returns true), the packet
    // can be modified and transmitted again by calling MACFlush() again.
    // Until MACPutHeader() is called, the data in the TX buffer will not be
    // corrupted.
    BFSReg(ECON1, ECON1_TXRTS);

    // Revision B5 and B7 silicon errata workaround
    if(ENCRevID == 0x05u || ENCRevID == 0x06u)
    {
        uint16_t AttemptCounter = 0x0000;
        while(!(ReadETHReg(EIR).Val & (EIR_TXERIF | EIR_TXIF)) && (++AttemptCounter < 1000u));
        if(ReadETHReg(EIR).EIRbits.TXERIF || (AttemptCounter >= 1000u))
        {
            TCPIP_UINT16_VAL ReadPtrSave;
            TCPIP_UINT16_VAL TXEnd;
            TXSTATUS TXStatus;
            uint8_t i;

            // Cancel the previous transmission if it has become stuck set
            BFCReg(ECON1, ECON1_TXRTS);

            // Save the current read pointer (controlled by application)
            ReadPtrSave.v[0] = ReadETHReg(ERDPTL).Val;
            ReadPtrSave.v[1] = ReadETHReg(ERDPTH).Val;

            // Get the location of the transmit status vector
            TXEnd.v[0] = ReadETHReg(ETXNDL).Val;
            TXEnd.v[1] = ReadETHReg(ETXNDH).Val;
            TXEnd.Val++;

            // Read the transmit status vector
            WriteReg(ERDPTL, TXEnd.v[0]);
            WriteReg(ERDPTH, TXEnd.v[1]);
            ENC28J60_MACGetArray((uint8_t*)&TXStatus, sizeof(TXStatus));

            // Implement retransmission if a late collision occured (this can
            // happen on B5 when certain link pulses arrive at the same time
            // as the transmission)
            for(i = 0; i < 16u; i++)
            {
                if(ReadETHReg(EIR).EIRbits.TXERIF && TXStatus.bits.LateCollision)
                {
                    // Reset the TX logic
                    BFSReg(ECON1, ECON1_TXRST);
                    BFCReg(ECON1, ECON1_TXRST);
                    BFCReg(EIR, EIR_TXERIF | EIR_TXIF);

                    // Transmit the packet again
                    BFSReg(ECON1, ECON1_TXRTS);
                    while(!(ReadETHReg(EIR).Val & (EIR_TXERIF | EIR_TXIF)));

                    // Cancel the previous transmission if it has become stuck set
                    BFCReg(ECON1, ECON1_TXRTS);

                    // Read transmit status vector
                    WriteReg(ERDPTL, TXEnd.v[0]);
                    WriteReg(ERDPTH, TXEnd.v[1]);
                    ENC28J60_MACGetArray((uint8_t*)&TXStatus, sizeof(TXStatus));
                }
                else
                {
                    break;
                }
            }

            // Restore the current read pointer
            WriteReg(ERDPTL, ReadPtrSave.v[0]);
            WriteReg(ERDPTH, ReadPtrSave.v[1]);
        }
    }
}


/******************************************************************************
 * Function:        void ENC28J60_MACSetReadPtrInRx(uint16_t offset)
 *
 * PreCondition:    A packet has been obtained by calling MACGetHeader() and
 *                  getting a true result.
 *
 * Input:           offset: uint16_t specifying how many bytes beyond the Ethernet
 *                          header's type field to relocate the SPI read
 *                          pointer.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        SPI read pointer are updated.  All calls to
 *                  MACGet() and MACGetArray() will use these new values.
 *
 * Note:            RXSTOP must be statically defined as being > RXSTART for
 *                  this function to work correctly.  In other words, do not
 *                  define an RX buffer which spans the 0x1FFF->0x0000 memory
 *                  boundary.
 *****************************************************************************/
void ENC28J60_MACSetReadPtrInRx(uint16_t offset)
{
    TCPIP_UINT16_VAL ReadPT;

    // Determine the address of the beginning of the entire packet
    // and adjust the address to the desired location
    ReadPT.Val = CurrentPacketLocation.Val + sizeof(ENC_PREAMBLE) + offset;

    // Since the receive buffer is circular, adjust if a wraparound is needed
    if(ReadPT.Val > RXSTOP)
        ReadPT.Val -= RXSIZE;

    // Set the SPI read pointer to the new calculated value
    WriteReg(ERDPTL, ReadPT.v[0]);
    WriteReg(ERDPTH, ReadPT.v[1]);
}


/******************************************************************************
 * Function:        PTR_BASE ENC28J60_MACSetWritePtr(PTR_BASE Address)
 *
 * PreCondition:    None
 *
 * Input:           Address: Address to seek to
 *
 * Output:          uint16_t: Old EWRPT location
 *
 * Side Effects:    None
 *
 * Overview:        SPI write pointer is updated.  All calls to
 *                  MACPut() and MACPutArray() will use this new value.
 *
 * Note:            None
 *****************************************************************************/
PTR_BASE ENC28J60_MACSetWritePtr(PTR_BASE address)
{
    TCPIP_UINT16_VAL oldVal;

    oldVal.v[0] = ReadETHReg(EWRPTL).Val;
    oldVal.v[1] = ReadETHReg(EWRPTH).Val;

    // Set the SPI write pointer to the new calculated value
    WriteReg(EWRPTL, ((TCPIP_UINT16_VAL*)&address)->v[0]);
    WriteReg(EWRPTH, ((TCPIP_UINT16_VAL*)&address)->v[1]);

    return oldVal.Val;
}

/******************************************************************************
 * Function:        PTR_BASE ENC28J60_MACSetReadPtr(PTR_BASE Address)
 *
 * PreCondition:    None
 *
 * Input:           Address: Address to seek to
 *
 * Output:          uint16_t: Old ERDPT value
 *
 * Side Effects:    None
 *
 * Overview:        SPI write pointer is updated.  All calls to
 *                  MACPut() and MACPutArray() will use this new value.
 *
 * Note:            None
 *****************************************************************************/
PTR_BASE ENC28J60_MACSetReadPtr(PTR_BASE address)
{
    TCPIP_UINT16_VAL oldVal;

    oldVal.v[0] = ReadETHReg(ERDPTL).Val;
    oldVal.v[1] = ReadETHReg(ERDPTH).Val;

    // Set the SPI write pointer to the new calculated value
    WriteReg(ERDPTL, ((TCPIP_UINT16_VAL*)&address)->v[0]);
    WriteReg(ERDPTH, ((TCPIP_UINT16_VAL*)&address)->v[1]);

    return oldVal.Val;
}


/******************************************************************************
 * Function:        uint16_t ENC28J60_MACCalcRxChecksum(uint16_t offset, uint16_t len)
 *
 * PreCondition:    None
 *
 * Input:           offset  - Number of bytes beyond the beginning of the
 *                          Ethernet data (first byte after the type field)
 *                          where the checksum should begin
 *                  len     - Total number of bytes to include in the checksum
 *
 * Output:          16-bit checksum as defined by RFC 793.
 *
 * Side Effects:    None
 *
 * Overview:        This function performs a checksum calculation in the MAC
 *                  buffer itself
 *
 * Note:            None
 *****************************************************************************/
uint16_t ENC28J60_MACCalcRxChecksum(TCPIP_MAC_HANDLE hMac, uint16_t offset, uint16_t len)
{
    TCPIP_UINT16_VAL temp;
    TCPIP_UINT16_VAL RDSave;

    // Add the offset requested by firmware plus the Ethernet header
    temp.Val = CurrentPacketLocation.Val + sizeof(ENC_PREAMBLE) + offset;
    if(temp.Val > RXSTOP)       // Adjust value if a wrap is needed
    {
        temp.Val -= RXSIZE;
    }

    RDSave.v[0] = ReadETHReg(ERDPTL).Val;
    RDSave.v[1] = ReadETHReg(ERDPTH).Val;

    WriteReg(ERDPTL, temp.v[0]);
    WriteReg(ERDPTH, temp.v[1]);

    temp.Val = ENC28J60_CalcIPBufferChecksum(hMac, len);

    WriteReg(ERDPTL, RDSave.v[0]);
    WriteReg(ERDPTH, RDSave.v[1]);

    return temp.Val;
}


/******************************************************************************
 * Function:        uint16_t ENC28J60_CalcIPBufferChecksum(uint16_t len)
 *
 * PreCondition:    Read buffer pointer set to starting of checksum data
 *
 * Input:           len: Total number of bytes to calculate the checksum over.
 *                       The first byte included in the checksum is the byte
 *                       pointed to by ERDPT, which is updated by calls to
 *                       MACSetReadPtr(), MACGet(), MACGetArray(),
 *                       MACGetHeader(), etc.
 *
 * Output:          16-bit checksum as defined by RFC 793
 *
 * Side Effects:    None
 *
 * Overview:        This function performs a checksum calculation in the MAC
 *                  buffer itself.  The ENC28J60 has a hardware DMA module
 *                  which can calculate the checksum faster than software, so
 *                  this function replaces the CaclIPBufferChecksum() function
 *                  defined in the tcpip_helpers.c file.  Through the use of
 *                  preprocessor defines, this replacement is automatic.
 *
 * Note:            This function works either in the RX buffer area or the TX
 *                  buffer area.  No validation is done on the len parameter.
 *****************************************************************************/
uint16_t ENC28J60_CalcIPBufferChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len)
{
    TCPIP_UINT16_VAL Start;
    TCPIP_UINT32_VAL Checksum = {0x00000000ul};
    uint16_t ChunkLen;
    uint16_t DataBuffer[10];
    uint16_t *DataPtr;

    // Save the SPI read pointer starting address
    Start.v[0] = ReadETHReg(ERDPTL).Val;
    Start.v[1] = ReadETHReg(ERDPTH).Val;

    while(len)
    {
        // Obtain a chunk of data (less SPI overhead compared
        // to requesting one byte at a time)
        ChunkLen = len > sizeof(DataBuffer) ? sizeof(DataBuffer) : len;
        ENC28J60_MACGetArray((uint8_t*)DataBuffer, ChunkLen);

        len -= ChunkLen;

        // Take care of a last odd numbered data byte
        if(((TCPIP_UINT16_VAL*)&ChunkLen)->bits.b0)
        {
            ((uint8_t*)DataBuffer)[ChunkLen] = 0x00;
            ChunkLen++;
        }

        // Calculate the checksum over this chunk
        DataPtr = DataBuffer;
        while(ChunkLen)
        {
            Checksum.Val += *DataPtr++;
            ChunkLen -= 2;
        }
    }

    // Restore old read pointer location
    WriteReg(ERDPTL, Start.v[0]);
    WriteReg(ERDPTH, Start.v[1]);

    // Do an end-around carry (one's complement arrithmatic)
    Checksum.Val = (uint32_t)Checksum.w[0] + (uint32_t)Checksum.w[1];

    // Do another end-around carry in case if the prior add
    // caused a carry out
    Checksum.w[0] += Checksum.w[1];

    // Return the resulting checksum
    return ~Checksum.w[0];
}

PTR_BASE ENC28J60_MACGetReadPtrInRx(TCPIP_MAC_HANDLE hMac)
{
	uint16_t tmp = 0;
	tmp = (ReadETHReg(ERDPTH).Val<<8) | (ReadETHReg(ERDPTL).Val);
    return (PTR_BASE)tmp;
}

/******************************************************************************
 * Function:        void ENC28J60_MACMemCopyAsync(PTR_BASE destAddr, PTR_BASE sourceAddr, uint16_t len)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *
 * Input:           destAddr:   Destination address in the Ethernet memory to
 *                              copy to.  If (PTR_BASE)-1 is specified, the 
 *								current EWRPT value will be used instead.
 *                  sourceAddr: Source address to read from.  If (PTR_BASE)-1 is
 *                              specified, the current ERDPT value will be used
 *                              instead.
 *                  len:        Number of bytes to copy
 *
 * Output:          Byte read from the ENC28J60's RAM
 *
 * Side Effects:    None
 *
 * Overview:        Bytes are asynchrnously transfered within the buffer.  Call
 *                  MACIsMemCopyDone() to see when the transfer is complete.
 *
 * Note:            If a prior transfer is already in progress prior to
 *                  calling this function, this function will block until it
 *                  can start this transfer.
 *
 *                  If (PTR_BASE)-1 is used for the sourceAddr or destAddr
 *                  parameters, then that pointer will get updated with the
 *                  next address after the read or write.
 *****************************************************************************/
void ENC28J60_MACMemCopyAsync(PTR_BASE destAddr, PTR_BASE sourceAddr, uint16_t len)
{
    TCPIP_UINT16_VAL ReadSave, WriteSave;
    bool UpdateWritePointer = false;
    bool UpdateReadPointer = false;

    if(destAddr == (PTR_BASE)-1)
    {
        UpdateWritePointer = true;
        destAddr = ReadETHReg(EWRPTL).Val;
        ((uint8_t*)&destAddr)[1] = ReadETHReg(EWRPTH).Val;
    }
    if(sourceAddr == (PTR_BASE)-1)
    {
        UpdateReadPointer = true;
        sourceAddr = ReadETHReg(ERDPTL).Val;
        ((uint8_t*)&sourceAddr)[1] = ReadETHReg(ERDPTH).Val;
    }

    // Handle special conditions where len == 0 or len == 1
    // The DMA module is not capable of handling those corner cases
    if(len <= 1u)
    {
        if(!UpdateReadPointer)
        {
            ReadSave.v[0] = ReadETHReg(ERDPTL).Val;
            ReadSave.v[1] = ReadETHReg(ERDPTH).Val;
        }
        if(!UpdateWritePointer)
        {
            WriteSave.v[0] = ReadETHReg(EWRPTL).Val;
            WriteSave.v[1] = ReadETHReg(EWRPTH).Val;
        }
        WriteReg(ERDPTL, ((uint8_t*)&sourceAddr)[0]);
        WriteReg(ERDPTH, ((uint8_t*)&sourceAddr)[1]);
        WriteReg(EWRPTL, ((uint8_t*)&destAddr)[0]);
        WriteReg(EWRPTH, ((uint8_t*)&destAddr)[1]);
        while(len--)
            ENC28J60_MACPut(ENC28J60_MACGet());
        if(!UpdateReadPointer)
        {
            WriteReg(ERDPTL, ReadSave.v[0]);
            WriteReg(ERDPTH, ReadSave.v[1]);
        }
        if(!UpdateWritePointer)
        {
            WriteReg(EWRPTL, WriteSave.v[0]);
            WriteReg(EWRPTH, WriteSave.v[1]);
        }
    }
    else
    {
        if(UpdateWritePointer)
        {
            WriteSave.Val = destAddr + len;
            WriteReg(EWRPTL, WriteSave.v[0]);
            WriteReg(EWRPTH, WriteSave.v[1]);
        }
        len += sourceAddr - 1;
        while(ReadETHReg(ECON1).ECON1bits.DMAST);
        WriteReg(EDMASTL, ((uint8_t*)&sourceAddr)[0]);
        WriteReg(EDMASTH, ((uint8_t*)&sourceAddr)[1]);
        WriteReg(EDMADSTL, ((uint8_t*)&destAddr)[0]);
        WriteReg(EDMADSTH, ((uint8_t*)&destAddr)[1]);
        if((sourceAddr <= RXSTOP) && (len > RXSTOP)) //&& (sourceAddr >= RXSTART))
            len -= RXSIZE;
        WriteReg(EDMANDL, ((uint8_t*)&len)[0]);
        WriteReg(EDMANDH, ((uint8_t*)&len)[1]);
        BFCReg(ECON1, ECON1_CSUMEN);
        BFSReg(ECON1, ECON1_DMAST);
        if(UpdateReadPointer)
        {
            len++;
            if((sourceAddr <= RXSTOP) && (len > RXSTOP)) //&& (sourceAddr >= RXSTART))
                len -= RXSIZE;
            WriteReg(ERDPTL, ((uint8_t*)&len)[0]);
            WriteReg(ERDPTH, ((uint8_t*)&len)[1]);
        }
    }
}

bool ENC28J60_MACIsMemCopyDone(void)
{
    return !ReadETHReg(ECON1).ECON1bits.DMAST;
}


/******************************************************************************
 * Function:        uint8_t ENC28J60_MACGet()
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *                  ERDPT must point to the place to read from.
 *
 * Input:           None
 *
 * Output:          Byte read from the ENC28J60's RAM
 *
 * Side Effects:    None
 *
 * Overview:        MACGet returns the byte pointed to by ERDPT and
 *                  increments ERDPT so MACGet() can be called again.  The
 *                  increment will follow the receive buffer wrapping boundary.
 *
 * Note:            None
 *****************************************************************************/
uint8_t ENC28J60_MACGet(void)
{
    uint8_t Result;
	uint8_t txBuf[2] = {RBM, 0}, rxBuf[2] = {0, 0};

    Enc28EnableChipSelect();

    // Send the Read Buffer Memory command and read the second byte
	DRV_SPI_TxRx(ENC28_SPI_CHN, txBuf, 2, rxBuf, 2);
	// Get received byte(Note, The last byte is the needed result)
    Result = rxBuf[1];
    Enc28DisableChipSelect();

    return Result;
}//end MACGet


/******************************************************************************
 * Function:        uint16_t ENC28J60_MACGetArray(uint8_t *val, uint16_t len)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *                  ERDPT must point to the place to read from.
 *
 * Input:           *val: Pointer to storage location
 *                  len:  Number of bytes to read from the data buffer.
 *
 * Output:          Byte(s) of data read from the data buffer.
 *
 * Side Effects:    None
 *
 * Overview:        Burst reads several sequential bytes from the data buffer
 *                  and places them into local memory.  With SPI burst support,
 *                  it performs much faster than multiple MACGet() calls.
 *                  ERDPT is incremented after each byte, following the same
 *                  rules as MACGet().
 *
 * Note:            None
 *****************************************************************************/
uint16_t ENC28J60_MACGetArray(uint8_t *val, uint16_t len)
{
// Workaround needed on HPC Explorer (classic) board to prevent interference
// with I2C temperature sensor on the same SPI wires
    uint16_t i = 0;
	uint8_t txBuf[1] = {RBM}, rxBuf[1] = {0};

    // Start the burst operation
    Enc28EnableChipSelect();
	// Send the Read Buffer Memory opcode.
	DRV_SPI_TxRx(ENC28_SPI_CHN, txBuf, 1, rxBuf, 1);

	// Get received data stream
	i = len;
	DRV_SPI_TxRx(ENC28_SPI_CHN, txBuf, 0, val, len);

    // Terminate the burst operation
    Enc28DisableChipSelect();

    return i;
}//end MACGetArray


/******************************************************************************
 * Function:        void ENC28J60_MACPut(uint8_t val)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *                  EWRPT must point to the location to begin writing.
 *
 * Input:           Byte to write into the ENC28J60 buffer memory
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        MACPut outputs the Write Buffer Memory opcode/constant
 *                  (8 bits) and data to write (8 bits) over the SPI.
 *                  EWRPT is incremented after the write.
 *
 * Note:            None
 *****************************************************************************/
void ENC28J60_MACPut(uint8_t val)
{
	uint8_t txBuf[2] = {WBM, 0};
    //volatile uint8_t Dummy;

    Enc28EnableChipSelect();

    // Send the Write Buffer Memory and data, in on 16-bit write
    txBuf[1] = val;
	DRV_SPI_TxRx(ENC28_SPI_CHN, txBuf, 2, NULL, 0);

    Enc28DisableChipSelect();
}//end MACPut


/******************************************************************************
 * Function:        void ENC28J60_MACPutArray(uint8_t *val, uint16_t len)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *                  EWRPT must point to the location to begin writing.
 *
 * Input:           *val: Pointer to source of bytes to copy.
 *                  len:  Number of bytes to write to the data buffer.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        MACPutArray writes several sequential bytes to the
 *                  ENC28J60 RAM.  It performs faster than multiple MACPut()
 *                  calls.  EWRPT is incremented by len.
 *
 * Note:            None
 *****************************************************************************/
void ENC28J60_MACPutArray(uint8_t *val, uint16_t len)
{
// Workaround needed on HPC Explorer (classic) board to prevent interference
// with I2C temperature sensor on the same SPI wires    uint16_t i = 0;
	uint8_t txBuf[1] = {WBM};
    //volatile uint8_t Dummy;

    // Select the chip and send the proper opcode
    Enc28EnableChipSelect();
	DRV_SPI_TxRx(ENC28_SPI_CHN, txBuf, 1, NULL, 0);
	// Send the data
	DRV_SPI_TxRx(ENC28_SPI_CHN, val, len, NULL, 0);

    // Terminate the burst operation
    Enc28DisableChipSelect();
}//end MACPutArray


/******************************************************************************
 * Function:        static void ENC28J60_SendSystemReset(void)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        SendSystemReset sends the System Reset SPI command to
 *                  the Ethernet controller.  It resets all register contents
 *                  (except for ECOCON) and returns the device to the power
 *                  on default state.
 *
 * Note:            None
 *****************************************************************************/
static void ENC28J60_SendSystemReset(void)
{
	uint8_t txBuf[1] = {SR};

    // Note: The power save feature may prevent the reset from executing, so
    // we must make sure that the device is not in power save before issuing
    // a reset.
    BFCReg(ECON2, ECON2_PWRSV);

    // Give some opportunity for the regulator to reach normal regulation and
    // have all clocks running
    SYS_TICK_MsDelay(1);

    // Execute the System Reset command
    Enc28EnableChipSelect();
	DRV_SPI_TxRx(ENC28_SPI_CHN, txBuf, 1, NULL, 0);
    Enc28DisableChipSelect();

    // Wait for the oscillator start up timer and PHY to become ready
    SYS_TICK_MsDelay(1);
}//end SendSystemReset


/******************************************************************************
 * Function:        void ENC28J60_MACPowerDown(void)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        MACPowerDown puts the ENC28J60 in low power sleep mode. In
 *                  sleep mode, no packets can be transmitted or received.
 *                  All MAC and PHY registers should not be accessed.
 *
 * Note:            If a packet is being transmitted while this function is
 *                  called, this function will block until it is it complete.
 *                  If anything is being received, it will be completed.
 *****************************************************************************/
void ENC28J60_MACPowerDown(void)
{
    // Disable packet reception
    BFCReg(ECON1, ECON1_RXEN);

    // Make sure any last packet which was in-progress when RXEN was cleared
    // is completed
    while(ReadETHReg(ESTAT).ESTATbits.RXBUSY);

    // If a packet is being transmitted, wait for it to finish
    while(ReadETHReg(ECON1).ECON1bits.TXRTS);

    // Enter sleep mode
    BFSReg(ECON2, ECON2_PWRSV);
}//end MACPowerDown


/******************************************************************************
 * Function:        void ENC28J60_MACPowerUp(void)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        MACPowerUp returns the ENC28J60 back to normal operation
 *                  after a previous call to MACPowerDown().  Calling this
 *                  function when already powered up will have no effect.
 *
 * Note:            If a link partner is present, it will take 10s of
 *                  milliseconds before a new link will be established after
 *                  waking up.  While not linked, packets which are
 *                  transmitted will most likely be lost.  MACIsLinked() can
 *                  be called to determine if a link is established.
 *****************************************************************************/
void ENC28J60_ACPowerUp(void)
{
    // Leave power down mode
    BFCReg(ECON2, ECON2_PWRSV);

    // Wait for the 300us Oscillator Startup Timer (OST) to time out.  This
    // delay is required for the PHY module to return to an operational state.
    while(!ReadETHReg(ESTAT).ESTATbits.CLKRDY);

    // Enable packet reception
    BFSReg(ECON1, ECON1_RXEN);
}//end MACPowerUp


/******************************************************************************
 * Function:        void ENC28J60_SetCLKOUT(uint8_t NewConfig)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *
 * Input:           NewConfig - 0x00: CLKOUT disabled (pin driven low)
 *                              0x01: Divide by 1 (25 MHz)
 *                              0x02: Divide by 2 (12.5 MHz)
 *                              0x03: Divide by 3 (8.333333 MHz)
 *                              0x04: Divide by 4 (6.25 MHz, POR default)
 *                              0x05: Divide by 8 (3.125 MHz)
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Writes the value of NewConfig into the ECOCON register.
 *                  The CLKOUT pin will beginning outputting the new frequency
 *                  immediately.
 *
 * Note:
 *****************************************************************************/
void ENC28J60_SetCLKOUT(uint8_t NewConfig)
{
    BankSel(ECOCON);
    WriteReg((uint8_t)ECOCON, NewConfig);
    BankSel(ERDPTL);
}//end SetCLKOUT


/******************************************************************************
 * Function:        uint8_t ENC28J60_GetCLKOUT(void)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *
 * Input:           None
 *
 * Output:          uint8_t - 0x00: CLKOUT disabled (pin driven low)
 *                         0x01: Divide by 1 (25 MHz)
 *                         0x02: Divide by 2 (12.5 MHz)
 *                         0x03: Divide by 3 (8.333333 MHz)
 *                         0x04: Divide by 4 (6.25 MHz, POR default)
 *                         0x05: Divide by 8 (3.125 MHz)
 *                         0x06: Reserved
 *                         0x07: Reserved
 *
 * Side Effects:    None
 *
 * Overview:        Returns the current value of the ECOCON register.
 *
 * Note:            None
 *****************************************************************************/
uint8_t ENC28J60_GetCLKOUT(void)
{
    uint8_t i;

    BankSel(ECOCON);
    i = ReadETHReg((uint8_t)ECOCON).Val;
    BankSel(ERDPTL);
    return i;
}//end GetCLKOUT


/******************************************************************************
 * Function:        void ENC28J60_SetRXHashTableEntry(MAC_ADDR DestMACAddr)
 *
 * PreCondition:    SPI interface must be initialized (done in MACInit()).
 *
 * Input:           DestMACAddr: 6 byte group destination MAC address to allow 
 *								 through the Hash Table Filter.  If DestMACAddr 
 *								 is set to 00-00-00-00-00-00, then the hash 
 *								 table will be cleared of all entries and the 
 *								 filter will be disabled.
 *
 * Output:          Sets the appropriate bit in the EHT* registers to allow 
 *					packets sent to DestMACAddr to be received and enables the 
 *					Hash Table receive filter (if not already).
 *
 * Side Effects:    None
 *
 * Overview:        Calculates a CRC-32 using polynomial 0x4C11DB7 and then, 
 *					using bits 28:23 of the CRC, sets the appropriate bit in 
 *					the EHT0-EHT7 registers.
 *
 * Note:            This code is commented out to save code space on systems 
 *					that do not need this function.  Change the 
 *					"#if TCPIP_STACK_USE_ZEROCONF_MDNS_SD" line to "#if 1" to 
 *					uncomment it, assuming you aren't using the Zeroconf module, 
 *					which requires mutlicast support and enables this function 
 *					automatically.
 *
 *					There is no way to individually unset destination MAC 
 *					addresses from the hash table since it is possible to have 
 *					a hash collision and therefore multiple MAC addresses 
 *					relying on the same hash table bit.  The stack would have 
 *					to individually store each 6 byte MAC address to support 
 *					this feature, which would waste a lot of RAM and be 
 *					unnecessary in most applications.  As a simple compromise, 
 *					you can call SetRXHashTableEntry() using a 
 *					00-00-00-00-00-00 destination MAC address, which will clear 
 *					the entire hash table and disable the hash table filter.  
 *					This will allow you to then re-add the necessary 
 *					destination address(es).
 *
 *					This function is intended to be used when 
 *					ERXFCON.ANDOR == 0 (OR).
 *****************************************************************************/
#if defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
void ENC28J60_SetRXHashTableEntry(MAC_ADDR DestMACAddr)
{
    TCPIP_UINT32_VAL CRC = {0xFFFFFFFF};
    uint8_t HTRegister;
    uint8_t i, j;

	// Select proper bank for ERXFCON and EHT0-EHT7 register access
	BankSel(ERXFCON);

	// Clear the Hash Table bits and disable the Hash Table Filter if a special 
	// 00-00-00-00-00-00 destination MAC address is provided.
	if((DestMACAddr.v[0] | DestMACAddr.v[1] | DestMACAddr.v[2] | DestMACAddr.v[3] | DestMACAddr.v[4] | DestMACAddr.v[5]) == 0x00u)
	{
		// Disable the Hash Table receive filter and clear the hash table
		BFCReg((uint8_t)ERXFCON, ERXFCON_HTEN);
		for(i = (uint8_t)EHT0; i <= (uint8_t)EHT7; i++)
			WriteReg(i, 0x00);
	}
	else
	{
		// Calculate a CRC-32 over the 6 byte MAC address
		// using polynomial 0x4C11DB7
		for(i = 0; i < sizeof(MAC_ADDR); i++)
		{
			uint8_t  crcnext;

			// shift in 8 bits
			for(j = 0; j < 8; j++)
			{
				crcnext = 0;
				if(((TCPIP_UINT8_VAL*)&(CRC.v[3]))->bits.b7)
					crcnext = 1;
				crcnext ^= (((TCPIP_UINT8_VAL*)&DestMACAddr.v[i])->bits.b0);

				CRC.Val <<= 1;
				if(crcnext)
					CRC.Val ^= 0x4C11DB7;
				// next bit
				DestMACAddr.v[i] >>= 1;
			}
		}

		// CRC-32 calculated, now extract bits 28:23
		// Bits 25:23 define where within the Hash Table byte the bit needs to be set
		// Bits 28:26 define which of the 8 Hash Table bytes that bits 25:23 apply to
		i = CRC.v[3] & 0x1F;
		HTRegister = (i >> 2) + (uint8_t)EHT0;
		i = (i << 1) & 0x06;
		((TCPIP_UINT8_VAL*)&i)->bits.b0 = ((TCPIP_UINT8_VAL*)&CRC.v[2])->bits.b7;

		// Set the proper bit in the Hash Table
		BFSReg(HTRegister, 1<<i);

		// Ensure that the Hash Table receive filter is enabled
		BFSReg((uint8_t)ERXFCON, ERXFCON_HTEN);
	}

    BankSel(ERDPTL);            // Return to Bank 0
}
#endif

/******************************************************************************
 * Function:        REG ReadETHReg(uint8_t Address)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *                  Bank select bits must be set corresponding to the register
 *                  to read from.
 *
 * Input:           5 bit address of the ETH control register to read from.
 *                    The top 3 bits must be 0.
 *
 * Output:          Byte read from the Ethernet controller's ETH register.
 *
 * Side Effects:    None
 *
 * Overview:        ReadETHReg sends the 8 bit RCR opcode/Address byte over
 *                  the SPI and then retrives the register contents in the
 *                  next 8 SPI clocks.
 *
 * Note:            This routine cannot be used to access MAC/MII or PHY
 *                  registers.  Use ReadMACReg() or ReadPHYReg() for that
 *                  purpose.
 *****************************************************************************/
static REG ReadETHReg(uint8_t Address)
{
    REG r;
	uint8_t txBuf[2] = {0, 0}, rxBuf[2] = {0, 0};

    // Select the chip and send the Read Control Register opcode/address
    Enc28EnableChipSelect();
	txBuf[0] = RCR | Address;
	// First SPI transfer transmits opcode/address; 
	// Second SPI transfer gets the Register content;
	DRV_SPI_TxRx(ENC28_SPI_CHN, txBuf, 2, rxBuf, 2);
	r.Val = rxBuf[1];
    Enc28DisableChipSelect();

    return r;
}//end ReadETHReg


/******************************************************************************
 * Function:        REG ReadMACReg(uint8_t Address)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *                  Bank select bits must be set corresponding to the register
 *                  to read from.
 *
 * Input:           5 bit address of the MAC or MII register to read from.
 *                    The top 3 bits must be 0.
 *
 * Output:          Byte read from the Ethernet controller's MAC/MII register.
 *
 * Side Effects:    None
 *
 * Overview:        ReadMACReg sends the 8 bit RCR opcode/Address byte as well
 *                  as a dummy byte over the SPI and then retrives the
 *                  register contents in the last 8 SPI clocks.
 *
 * Note:            This routine cannot be used to access ETH or PHY
 *                  registers.  Use ReadETHReg() or ReadPHYReg() for that
 *                  purpose.
 *****************************************************************************/
static REG ReadMACReg(uint8_t Address)
{
    REG r;
	uint8_t txBuf[3] = {0, 0, 0}, rxBuf[3] = {0, 0, 0};

    Enc28EnableChipSelect();
	// First SPI transfer transmits opcode/address
	// Seoncd SPI transfer is dummy byte
	// Third SPI transfer gets the register content
	txBuf[0] = RCR | Address;
	DRV_SPI_TxRx(ENC28_SPI_CHN, txBuf, 3, rxBuf, 3);
	r.Val = rxBuf[2]; // Thrid byte is the register value
    Enc28DisableChipSelect();

    return r;
}//end ReadMACReg


/******************************************************************************
 * Function:        ReadPHYReg
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *
 * Input:           Address of the PHY register to read from.
 *
 * Output:          16 bits of data read from the PHY register.
 *
 * Side Effects:    None
 *
 * Overview:        ReadPHYReg performs an MII read operation.  While in
 *                  progress, it simply polls the MII BUSY bit wasting time
 *                  (10.24us).
 *
 * Note:            None
 *****************************************************************************/
static PHYREG ReadPHYReg(uint8_t Register)
{
    PHYREG Result;

    // Set the right address and start the register read operation
    BankSel(MIREGADR);
    WriteReg((uint8_t)MIREGADR, Register);
    WriteReg((uint8_t)MICMD, MICMD_MIIRD);

    // Loop to wait until the PHY register has been read through the MII
    // This requires 10.24us
    BankSel(MISTAT);
    while(ReadMACReg((uint8_t)MISTAT).MISTATbits.BUSY);

    // Stop reading
    BankSel(MIREGADR);
    WriteReg((uint8_t)MICMD, 0x00);

    // Obtain results and return
    Result.VAL.v[0] = ReadMACReg((uint8_t)MIRDL).Val;
    Result.VAL.v[1] = ReadMACReg((uint8_t)MIRDH).Val;

    BankSel(ERDPTL);    // Return to Bank 0
    return Result;
}//end ReadPHYReg


/******************************************************************************
 * Function:        void WriteReg(uint8_t Address, uint8_t Data)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *                  Bank select bits must be set corresponding to the register
 *                  to modify.
 *
 * Input:           5 bit address of the ETH, MAC, or MII register to modify.
 *                    The top 3 bits must be 0.
 *                  Byte to be written into the register.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        WriteReg sends the 8 bit WCR opcode/Address byte over the
 *                  SPI and then sends the data to write in the next 8 SPI
 *                  clocks.
 *
 * Note:            This routine is almost identical to the BFCReg() and
 *                  BFSReg() functions.  It is seperate to maximize speed.
 *                  Unlike the ReadETHReg/ReadMACReg functions, WriteReg()
 *                  can write to any ETH or MAC register.  Writing to PHY
 *                  registers must be accomplished with WritePHYReg().
 *****************************************************************************/
static void WriteReg(uint8_t Address, uint8_t Data)
{
    //volatile uint8_t Dummy;
	uint8_t txBuf[2] = {0, 0};
    
    Enc28EnableChipSelect();

	txBuf[0] = (WCR | Address);
	txBuf[1] = Data;
	DRV_SPI_TxRx(ENC28_SPI_CHN, txBuf, 2, NULL, 0);

	// For faster processors (dsPIC), delay for a few clock cycles to ensure 
	// the MAC/MII register write Chip Select hold time minimum of 210ns is met.
    SYS_TICK_NsDelay(210);
	Enc28DisableChipSelect();
}//end WriteReg


/******************************************************************************
 * Function:        void BFCReg(uint8_t Address, uint8_t Data)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *                  Bank select bits must be set corresponding to the register
 *                    to modify.
 *
 * Input:           5 bit address of the register to modify.  The top 3 bits
 *                    must be 0.
 *                  Byte to be used with the Bit Field Clear operation.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BFCReg sends the 8 bit BFC opcode/Address byte over the
 *                  SPI and then sends the data in the next 8 SPI clocks.
 *
 * Note:            This routine is almost identical to the WriteReg() and
 *                  BFSReg() functions.  It is separate to maximize speed.
 *                  BFCReg() must only be used on ETH registers.
 *****************************************************************************/
static void BFCReg(uint8_t Address, uint8_t Data)
{
	uint8_t txBuf[2] = {0, 0};

    Enc28EnableChipSelect();
	txBuf[0] = BFC | Address;
	txBuf[1] = Data;
	DRV_SPI_TxRx(ENC28_SPI_CHN, txBuf, 2, NULL, 0);	
    Enc28DisableChipSelect();
}//end BFCReg


/******************************************************************************
 * Function:        void BFSReg(uint8_t Address, uint8_t Data)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *                  Bank select bits must be set corresponding to the register
 *                  to modify.
 *
 * Input:           5 bit address of the register to modify.  The top 3 bits
 *                    must be 0.
 *                  Byte to be used with the Bit Field Set operation.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BFSReg sends the 8 bit BFC opcode/Address byte over the
 *                  SPI and then sends the data in the next 8 SPI clocks.
 *
 * Note:            This routine is almost identical to the WriteReg() and
 *                  BFCReg() functions.  It is separate to maximize speed.
 *                  BFSReg() must only be used on ETH registers.
 *****************************************************************************/
static void BFSReg(uint8_t Address, uint8_t Data)
{
	uint8_t txBuf[2] = {0, 0};

    Enc28EnableChipSelect();
	txBuf[0] = BFS | Address;
	txBuf[1] = Data;
	DRV_SPI_TxRx(ENC28_SPI_CHN, txBuf, 2, NULL, 0);	
    Enc28DisableChipSelect();
}//end BFSReg


/******************************************************************************
 * Function:        WritePHYReg
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *
 * Input:           Address of the PHY register to write to.
 *                  16 bits of data to write to PHY register.
 *
 * Output:          None
 *
 * Side Effects:    Alters bank bits to point to Bank 3
 *
 * Overview:        WritePHYReg performs an MII write operation.  While in
 *                  progress, it simply polls the MII BUSY bit wasting time.
 *
 * Note:            None
 *****************************************************************************/
static void WritePHYReg(uint8_t Register, uint16_t Data)
{
    // Write the register address
    BankSel(MIREGADR);
    WriteReg((uint8_t)MIREGADR, Register);

    // Write the data
    // Order is important: write low byte first, high byte last
    WriteReg((uint8_t)MIWRL, ((TCPIP_UINT16_VAL*)&Data)->v[0]);
    WriteReg((uint8_t)MIWRH, ((TCPIP_UINT16_VAL*)&Data)->v[1]);

    // Wait until the PHY register has been written
    BankSel(MISTAT);
    while(ReadMACReg((uint8_t)MISTAT).MISTATbits.BUSY);

    BankSel(ERDPTL);    // Return to Bank 0
}//end WritePHYReg


/******************************************************************************
 * Function:        BankSel
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *
 * Input:           Register address with the high byte containing the 2 bank
 *                    select 2 bits.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BankSel takes the high byte of a register address and
 *                  changes the bank select bits in ETHCON1 to match.
 *
 * Note:            None
 *****************************************************************************/
static void BankSel(uint16_t Register)
{
    BFCReg(ECON1, ECON1_BSEL1 | ECON1_BSEL0);
    BFSReg(ECON1, ((TCPIP_UINT16_VAL*)&Register)->v[1]);
}//end BankSel



//// GetRegs is a function for debugging purposes only.  It will read all
//// registers and store them in the PIC's RAM so they can be viewed with
//// the ICD2.
//REG Regs[4][32];
//void GetRegs(void)
//{
//  uint8_t i;
//
//  BankSel(0x000);
//  for(i=0; i<0x1A; i++)
//      Regs[0][i] = ReadETHReg(i);
//  for(i=0x1B; i<32; i++)
//      Regs[0][i] = ReadETHReg(i);
//
//  BankSel(0x100);
//  for(i=0; i<0x1A; i++)
//      Regs[1][i] = ReadETHReg(i);
//  for(i=0x1B; i<32; i++)
//      Regs[1][i] = ReadETHReg(i);
//
//  BankSel(0x200);
//  for(i=0; i<5; i++)
//      Regs[2][i] = ReadMACReg(i);
//  Regs[2][5] = ReadETHReg(i);
//  for(i=6; i<0x0F; i++)
//      Regs[2][i] = ReadMACReg(i);
//  Regs[2][0x0F] = ReadETHReg(i);
//  for(i=0x10; i<0x13; i++)
//      Regs[2][i] = ReadMACReg(i);
//  Regs[2][0x13] = ReadETHReg(i);
//  for(i=0x14; i<0x1A; i++)
//      Regs[2][i] = ReadMACReg(i);
//  for(i=0x1B; i<32; i++)
//      Regs[2][i] = ReadETHReg(i);
//
//  BankSel(0x300);
//  for(i=0; i<0x06; i++)
//      Regs[3][i] = ReadMACReg(i);
//  for(i=6; i<0x0A; i++)
//      Regs[3][i] = ReadETHReg(i);
//  Regs[3][0x0A] = ReadMACReg(i);
//  for(i=0x0B; i<0x1A; i++)
//      Regs[3][i] = ReadETHReg(i);
//  for(i=0x1B; i<32; i++)
//      Regs[3][i] = ReadETHReg(i);
//
//  Regs[0][0x1A].Val = 0;
//  Regs[1][0x1A].Val = 0;
//  Regs[2][0x1A].Val = 0;
//  Regs[3][0x1A].Val = 0;
//
//  BankSel(ERDPTL);
//
//  return;
//}

//// Get8KBMem is a function intended for debugging purposes.  It will read all
//// Ethernet RAM and output it in hex out
//void Get8KBMem(void)
//{
//  TCPIP_UINT16_VAL i;
//  uint8_t v;
//  TCPIP_UINT16_VAL RDSave;
//
//  RDSave.v[0] = ReadETHReg(ERDPTL).Val;
//  RDSave.v[1] = ReadETHReg(ERDPTH).Val;
//
//  for(i.Val = 0; i.Val < 8192; i.Val++)
//  {
//      WriteReg(ERDPTL, i.v[0]);
//      WriteReg(ERDPTH, i.v[1]);
//      v = MACGet();
//
//      SYS_OUT_MESSAGE('0');
//      SYS_OUT_MESSAGE('x');
//      SYS_OUT_MESSAGE(btohexa_high(v));
//      SYS_OUT_MESSAGE(btohexa_low(v));
//  }
//
//  WriteReg(ERDPTL, RDSave.v[0]);
//  WriteReg(ERDPTH, RDSave.v[1]);
//
//}

#endif //#if defined(ENC_CS_TRIS)
