/*******************************************************************************
Microchip TCP/IP Stack Network Configuration Header

  Summary:
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	network_profile.h
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

#ifndef __NETWORK_PROFILE_H_
#define __NETWORK_PROFILE_H_


// =======================================================================
//   Network Interfaces Configuration Options
// =======================================================================

/* Default Network Configuration
 *   These settings are only used if data is not found in EEPROM.
 *   To clear EEPROM, hold BUTTON0, reset the board, and continue
 *   holding until the LEDs flash.  Release, and reset again.
 */

// =======================================================================
//   multi-homed hosts Network Addressing Configuration
// =======================================================================

#ifdef __LANGUAGE_C__
typedef struct
{
    const char*     interface;  // valid names "ENCJ60", "ENCJ600", "97J60", "PIC32INT", "MRF24W"
    const char*     hostName;   // "MCHPBOARD"
    const char*     macAddr;    // "00:04:a3:00:00:00"
    const char*     ipAddr;     // "169.254.1.1"
    const char*     ipMask;     // "255.255.0.0"
    const char*     gateway;    // "169.254.1.1"
    const char*     priDNS;     // "169.254.1.1"
    const char*     secondDNS;  // "0.0.0.0"
    // MAC configuration parameters
    // Note: not all MACs support all the features
    const char*     powerMode;  // valid options:
                                    // "full" - up and running;
                                    // "low" - low power mode;
                                    // "down" - powered down, not started
                                    // Note: "low" is not supported right now!
    int             txBuffSize;     // size of the corresponding TX buffer
    int             nTxDescriptors; // number of TX descriptors
    int             rxBuffSize;     // size of the corresponding RX buffer
    int             nRxDescriptors; // number of RX descriptors
}TCPIP_NETWORK_CONFIG;


// one configuration per each supported interface
// default values will be used if no configuration provided
static TCPIP_NETWORK_CONFIG __attribute__((unused))  TCPIP_HOSTS_CONFIGURATION[] =
{
    {
        "ENCJ60",               // interface
        "MCHPBOARD_E1",		// hostName
        "00:04:a3:00:00:00",    // macAddr
        "169.254.1.2",          // ipAddr 
        "255.255.0.0",          // ipMask
        "169.254.1.1",          // gateway
        "169.254.1.1",          // priDNS
        "0.0.0.0",              // secondDNS
        // MAC configuration parameters
        // Note: not all MACs support all the features
        "full",                 // powerMode
        1500,                   // txBuffSize
        2,                      // nTxDescriptors
        1536,                   // rxBuffSize
        8,                      // nRxDescriptors
    }, 
    {
        "ENCJ600",            	// interface
        "MCHPBOARD_E2",         // hostName
        "00:04:a3:00:00:00",    // macAddr
        "192.168.2.105",        // ipAddr 
        "255.255.255.0",        // ipMask
        "192.168.2.1",          // gateway
        "192.168.2.1",          // priDNS
        "0.0.0.0",              // secondDNS
        // MAC configuration parameters
        // Note: not all MACs support all the features
        "full",                 // powerMode
        1514,                   // txBuffSize
        2,                      // nTxDescriptors
        1536,                   // rxBuffSize
        8,                      // nRxDescriptors
    },
    
};

#endif  // __LANGUAGE_C__




#endif  // __NETWORK_PROFILE_H_

