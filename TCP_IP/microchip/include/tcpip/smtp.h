/*******************************************************************************
  Simple Mail Transfer Protocol (SMTP) Client

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  SMTP.h 
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

#ifndef __SMTP_H
#define __SMTP_H

/****************************************************************************
  Section:
	Data Type Definitions
  ***************************************************************************/
	#define SMTP_SUCCESS		(0x0000u)	// Message was successfully sent
	#define SMTP_RESOLVE_ERROR	(0x8000u)	// DNS lookup for SMTP server failed
	#define SMTP_CONNECT_ERROR	(0x8001u)	// Connection to SMTP server failed
	
/****************************************************************************
  Function:
      typedef struct SMTP_POINTERS
    
  Summary:
    Configures the SMTP client to send a message
    
  Description:
    This structure of pointers configures the SMTP Client to send an e-mail
    message. Initially, all pointers will be null. Set <c>SMTPClient.[field
    name].szRAM</c> to use a string stored in RAM, or <c>SMTPClient.[field
    name].szROM</c> to use a string stored in const. (Where <c>[field name]</c>
    is one of the parameters below.)
    
  Parameters:
    Server -        the SMTP server to relay the message through
    Username -      the user name to use when logging into the SMTP server,
                    if any is required
    Password -      the password to supply when logging in, if any is required
    To -            the destination address for this message. May be a
                    comma\-separated list of addresss, and/or formatted.
    CC -            The CC addresses for this message, if any. May be a
                    comma\-separated list of addresss, and/or formatted.
    BCC -           The BCC addresses for this message, if any. May be a
                    comma\-separated list of addresss, and/or formatted.
    From -          The From address for this message. May be formatted.
    Subject -       The Subject header for this message.
    OtherHeaders -  Any additional headers for this message. Each additional 
                    header, including the last one, must be terminated with
                    a CRLF pair.
    Body -          When sending a message from memory, the location of the
    				body of this message in memory. Leave as NULL to build 
    				a message on\-the\-fly.
    UseSSL -        When TCPIP_STACK_USE_SSL_CLIENT is enabled, this flag causes
                    the SMTP client to make an SSL connection to the server.
    ServerPort -    (uint16_t value) Indicates the port on which to connect to the
                    remote SMTP server.

  Remarks:
    When formatting an e-mail address, the SMTP standard format for associating a
    printable name may be used. This format places the printable name in quotation
    marks, with the address following in pointed brackets, such as "John Smith"
    \<john.smith@domain.com\>                                                       
  ***************************************************************************/
typedef struct
{
	char* Server;
	char* Username;
	char* Password;
	char* To;
	char* CC;
	char* BCC;
	char* From;
	char* Subject;
	char* OtherHeaders;
	char* Body;

	#if defined(TCPIP_STACK_USE_SSL_CLIENT)
	bool UseSSL;
	#endif
	
	uint16_t ServerPort;
	
} SMTP_POINTERS;

/****************************************************************************
  Section:
	Global SMTP Variables
  ***************************************************************************/
	extern SMTP_POINTERS SMTPClient;
	
/****************************************************************************
  Section:
	SMTP Function Prototypes
  ***************************************************************************/

bool SMTPBeginUsage(void);
uint16_t SMTPEndUsage(void);
void SMTPTask(void);
void SMTPSendMail(void);
bool SMTPIsBusy(void);
uint16_t SMTPIsPutReady(void);
bool SMTPPut(char c);
uint16_t SMTPPutArray(uint8_t* Data, uint16_t Len);
uint16_t SMTPPutString(char* Data);
void SMTPFlush(void);
void SMTPPutDone(void);


#endif
