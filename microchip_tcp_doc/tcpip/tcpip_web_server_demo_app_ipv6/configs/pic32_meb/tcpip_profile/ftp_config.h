/*******************************************************************************
  File Transfer Protocol (FTP) Configuration file

  Summary:
    FTP configuration file
    
  Description:
    This file contains the FTP module configuration options
    
*******************************************************************************/

/*******************************************************************************
FileName:   ftp_config.h
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

#ifndef _FTP_CONFIG_H_
#define _FTP_CONFIG_H_


// Specifies the max length for user name
#define FTP_USER_NAME_LEN		(10)

// Socket to send commands to
#define FTP_COMMAND_PORT		(21u)

//Used to set FTPDataPort if FTP socket was invalid.
#define FTP_DATA_PORT			(20u)

// FTP timeout, seconds
#define FTP_TIMEOUT			(180ul)     

// Used to tell ParseFTPString() function when to stop.
#define FTP_MAX_ARGS			(7u)

// Determins max charactors to get from the FTP socket
#define FTP_MAX_CMD_STRING_LEN		(31u)

// Comment this line out to disable MPFS
#define FTP_PUT_ENABLED

// Maximum number of FTP connections allowed
#define FTP_MAX_CONNECTIONS		(1)


#endif  // _FTP_CONFIG_H_
