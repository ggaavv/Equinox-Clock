/*******************************************************************************
  Header file for tcpip_helpers_private

  Summary:
    SUMMARY
    
  Description:
    DESCRIPTION
*******************************************************************************/

/*******************************************************************************
FileName:  tcpip_helpers_private.h 
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

#ifndef __TCPIP_HELPERS_PRIVATE_H_
#define __TCPIP_HELPERS_PRIVATE_H_


char *strupr(char* s);

// Implement consistent ultoa() function
#if (defined(__PIC32MX__) && (__C32_VERSION__ < 112)) || (defined (__C30__) && (__C30_VERSION__ < 325)) || defined(__C30_LEGACY_LIBC__) || defined(__C32_LEGACY_LIBC__)
	// C32 < 1.12 and C30 < v3.25 need this 2 parameter stack implemented function
	void ultoa(uint32_t Value, uint8_t* Buffer);
#else
	// C30 v3.25+, and C32 v1.12+ already have a ultoa() stdlib 
	// library function, but it requires 3 parameters.
	#include <stdlib.h>
	#define ultoa(val,buf)	ultoa((char*)(buf),(val),10)
#endif

void 	uitoa(uint16_t Value, uint8_t* Buffer);
void 	UnencodeURL(uint8_t* URL);
uint16_t 	Base64Decode(uint8_t* cSourceData, uint16_t wSourceLen, uint8_t* cDestData, uint16_t wDestLen);
uint16_t	Base64Encode(uint8_t* cSourceData, uint16_t wSourceLen, uint8_t* cDestData, uint16_t wDestLen);
bool	TCPIP_HELPER_StringToIPAddress(const char* str, IP_ADDR* IPAddress);
bool    StringToMACAddress(const char* str, uint8_t macAddr[6]);
uint8_t	hexatob(uint16_t AsciiVal);
uint8_t	btohexa_high(uint8_t b);
uint8_t	btohexa_low(uint8_t b);
signed char stricmppgm2ram(uint8_t* a, const uint8_t* b);
char * 	strnchr(const char *searchString, size_t count, char c);
size_t  strncpy_m(char* destStr, size_t destSize, int nStrings, ...);

#if defined (TCPIP_STACK_USE_IPV6)
    unsigned char FindCommonPrefix (unsigned char * addr1, unsigned char * addr2, unsigned char bytes);
#endif

TCPIP_MAC_ID StringToMACId(const char* str);

TCPIP_STACK_POWER_MODE StringToTCPIPPowerMode(const char* str);


uint16_t    CalcIPChecksum(uint8_t* buffer, uint16_t len);
uint16_t    CalcMACBufferIPChecksum(TCPIP_MAC_HANDLE hMac, uint16_t len);


void FormatNetBIOSName(uint8_t Name[16]);


// Protocols understood by the ExtractURLFields() function.  IMPORTANT: If you 
// need to reorder these (change their constant values), you must also reorder 
// the constant arrays in ExtractURLFields().
typedef enum
{
	PROTOCOL_HTTP = 0u,
	PROTOCOL_HTTPS,
	PROTOCOL_MMS,
	PROTOCOL_RTSP
} PROTOCOLS;

uint8_t ExtractURLFields(uint8_t *vURL, PROTOCOLS *protocol, uint8_t *vUsername, uint16_t *wUsernameLen, uint8_t *vPassword, uint16_t *wPasswordLen, uint8_t *vHostname, uint16_t *wHostnameLen, uint16_t *wPort, uint8_t *vFilePath, uint16_t *wFilePathLen);
int16_t Replace(uint8_t *vExpression, const uint8_t *vFind, const uint8_t *vReplacement, uint16_t wMaxLen, bool bSearchCaseInsensitive);

#endif  // __TCPIP_HELPERS_PRIVATE_H_
