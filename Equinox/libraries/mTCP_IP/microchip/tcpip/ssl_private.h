/*******************************************************************************
  SSLv3 Module Private Headers

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:  ssl_private.h
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

#ifndef __SSL_PRIVATE_H_
#define __SSL_PRIVATE_H_

#include <common/hashes.h>
/****************************************************************************
  Section:
	SSL Commands and Responses
  ***************************************************************************/

	#define SSL_CHANGE_CIPHER_SPEC	20u		// Protocol code for Change Cipher Spec records
	#define SSL_ALERT				21u		// Protocol code for Alert records
	#define SSL_HANDSHAKE			22u		// Protocol code for Handshake records
	#define SSL_APPLICATION			23u		// Protocol code for Application data records
	
	// Describes the types of SSL messages (handshake and alerts)
	typedef enum
	{
		SSL_HELLO_REQUEST				= 0u,	// HelloRequest handshake message (not currently supported)
		SSL_CLIENT_HELLO				= 1u,	// ClientHello handshake message
		SSL_ANTIQUE_CLIENT_HELLO		= 18u,	// SSLv2 ClientHello handshake message (Supported for backwards compatibility.  This is an internally defined value.)
		SSL_SERVER_HELLO				= 2u,	// ServerHello handshake message
		SSL_CERTIFICATE					= 11u,	// ServerCertifiate handshake message
		SSL_SERVER_HELLO_DONE			= 14u,	// ServerHelloDone handshake message
		SSL_CLIENT_KEY_EXCHANGE			= 16u,	// ClientKeyExchange handshake message
		SSL_FINISHED					= 20u,	// Finished handshake message
		
		// Alert Messages
		SSL_ALERT_CLOSE_NOTIFY			= 0u  + 0x80,	// CloseNotify alert message (dummy value used internally)
		SSL_ALERT_UNEXPECTED_MESSAGE	= 10u + 0x80,	// UnexpectedMessage alert message (dummy value used internally)
		SSL_ALERT_BAD_RECORD_MAC		= 20u + 0x80,	// BadRecordMAC alert message (dummy value used internally)
		SSL_ALERT_HANDSHAKE_FAILURE		= 40u + 0x80,	// HandshakeFailure alert message (dummy value used internally)
		
		// No Message
		SSL_NO_MESSAGE					= 0xff			// No message is currently requested (internally used value)
		
	} SSL_MESSAGES;
	
	// Describes the two types of Alert records
	typedef enum
	{
		SSL_ALERT_WARNING 	= 1u,	// Alert message is a warning (session can be resumed)
		SSL_ALERT_FATAL		= 2u	// Alert message is fatal (session is non-resumable)
	} SSL_ALERT_LEVEL;

	// SSL Session Type Enumeration
	typedef enum
	{
		SSL_CLIENT,			// Local device is the SSL client
		SSL_SERVER			// Local device is the SSL host
	} SSL_SESSION_TYPE;	


/****************************************************************************
  Section:
	State Machines
  ***************************************************************************/
	
	// State machine for SSLRxServerHello
	typedef enum
	{
		RX_SERVER_CERT_START = 0u,
		RX_SERVER_CERT_FIND_KEY,
		RX_SERVER_CERT_FIND_N,
		RX_SERVER_CERT_READ_N,
		RX_SERVER_CERT_READ_E,
		RX_SERVER_CERT_CLEAR
	} SM_SSL_RX_SERVER_HELLO;


/****************************************************************************
  Section:
	Connection Struct Definitions
  ***************************************************************************/

	// Memory holder for general information associated with
	// an SSL connections.
	typedef struct
	{
		uint16_t wRxBytesRem;					// Bytes left to read in current record
		uint16_t wRxHsBytesRem;					// Bytes left to read in current Handshake submessage
		
		uint8_t rxProtocol;					// Protocol for message being read
		uint8_t rxHSType;						// Handshake message being received
		
		uint8_t idSession;						// ID for associated session
		uint8_t idMD5, idSHA1;					// ID for current hashes
		uint8_t idRxHash;						// ID for MAC hash (TX needs no persistence)
		uint8_t idRxBuffer, idTxBuffer;		// ID for current buffers (Sboxes)
		
		TCPIP_UINT32_VAL dwTemp;					// Used for state machine in RxCertificate
		
        union
        {
            struct
            {
                unsigned char bIsServer					: 1;	// We are the server
                unsigned char bClientHello				: 1;	// ClientHello has been sent/received
                unsigned char bServerHello				: 1;	// ServerHello has been sent/received
                unsigned char bServerCertificate		: 1;	// ServerCertificate has been sent/received
                unsigned char bServerHelloDone			: 1;	// ServerHelloDone has been sent/received
                unsigned char bClientKeyExchange		: 1;	// ClientKeyExchange has been sent/received
                unsigned char bRemoteChangeCipherSpec	: 1;	// Remote node has sent a ChangeCipherSpec message
                unsigned char bRemoteFinished			: 1;	// Remote node has sent a Finished message
                unsigned char bLocalChangeCipherSpec	: 1;	// We have sent a ChangeCipherSpec message
                unsigned char bLocalFinished			: 1;	// We have sent a Finished message
                unsigned char bExpectingMAC				: 1;	// We expect a MAC at end of message
                unsigned char bNewSession				: 1;	// true if a new session, false if resuming
                unsigned char bCloseNotify				: 1;	// Whether or not a CloseNotify has been sent/received
                unsigned char bDone						: 1;	// true if the connection is closed
                unsigned char bRSAInProgress			: 1;	// true when RSA op is in progress
                unsigned char bKeysValid				: 1;	// true if the session keys have been generated
            };
            uint16_t    w;
        }Flags;
		
		uint8_t requestedMessage;				// Currently requested message to send, or 0xff
        void * supplementaryBuffer;
        uint8_t supplementaryDataType;
	} SSL_STUB;

    typedef enum
    {
        SSL_SUPPLEMENTARY_DATA_NONE = 0,
        SSL_SUPPLEMENTARY_DATA_CERT_PUBLIC_KEY
    } SSL_SUPPLEMENTARY_DATA_TYPES;

    // To hash the public key information, we need to actually get
    // the public key information...
    // 1024 bit key at 8 bits/byte = 128 bytes needed for the public key.
    typedef struct
    {
      uint16_t pub_size_bytes;
      uint8_t pub_key[SSL_RSA_CLIENT_SIZE/8];
      uint8_t pub_e[3];
      uint8_t pub_guid;    // This is used as a TCP_SOCKET which is a uint8_t
    } SSL_PKEY_INFO;

	// Memory definition for SSL keys.  This area is split into Local and
	// Remote areas.  During the handshake, Local.random and Remote.random
	// hold the ServerRandom and ClientRandom values.  Once the session keys
	// are calculated, the Local.app and Remote.app contain the MAC
	// secret, record sequence number, and encryption context for the
	// ARCFOUR module.
	typedef struct
	{
		union
		{
			struct
			{
				uint8_t MACSecret[16];			// Server's MAC write secret
				uint32_t sequence;				// Server's write sequence number
				ARCFOUR_CTX cryptCtx;		// Server's write encryption context
				uint8_t reserved[6];			// Future expansion
			}app;
			uint8_t random[32];				// Server.random value
		} Local;
		
		union
		{
			struct
			{
				uint8_t MACSecret[16];			// Client's MAC write secret
				uint32_t sequence;				// Client's write sequence number
				ARCFOUR_CTX cryptCtx;		// Client's write encryption context
				uint8_t reserved[6];			// Future expansion
			}app;
			uint8_t random[32];				// Client.random value
		} Remote;		
	} SSL_KEYS;

	// Generic buffer space for SSL.  The hashRounds element is used
	// when this buffer is needed for handshake hash calculations, and
	// the full element is used as the Sbox for ARCFOUR calculations.
	typedef union
	{
		struct
		{
			HASH_SUM hash;
			uint8_t md5_hash[16];
			uint8_t sha_hash[20];
            #if SSL_RSA_CLIENT_SIZE > 1024
                uint8_t temp[(SSL_RSA_CLIENT_SIZE/4)-sizeof(HASH_SUM)-16-20];
            #else
				uint8_t temp[256-sizeof(HASH_SUM)-16-20];
            #endif
		} hashRounds;
        #if SSL_RSA_CLIENT_SIZE > 1024
            uint8_t full[(SSL_RSA_CLIENT_SIZE/4)];
        #else
			uint8_t full[256];
        #endif
	} SSL_BUFFER;
	
	// Storage space for SSL Session identifiers.  (The SessionID and MasterSecret)
	typedef struct
	{
		uint8_t sessionID[32];					// The SSL Session ID for this session
		uint8_t masterSecret[48];				// Associated Master Secret for this session
	} SSL_SESSION;

	// Stub value for an SSL_SESSION.  The tag associates this session with a 
	// remote node, either by matching to a remote IP address when we are
	// the client or the first 3 bytes of the session ID when we are the host.
	// When a session is free/expired, the tag is 0x00000000.  The lastUsed
	// value is the Tick count when the session was last used so that 
	// older sessions may be overwritten first.
	typedef struct
	{
		TCPIP_UINT32_VAL tag;						// Identifying tag for connection
											// When we're a client, this is the remote IP
											// When we're a host, this is 0x00 followed by first 3 bytes of session ID
											// When this stub is free/expired, this is 0x00
		SYS_TICK lastUsed;					// Tick count when session was last used
	} SSL_SESSION_STUB;

	
	#define SSL_STUB_SIZE		((uint32_t)sizeof(SSL_STUB))				// Amount of space needed by a single SSL stub
	#define SSL_STUB_SPACE		(SSL_STUB_SIZE*SSL_MAX_CONNECTIONS)		// Amount of space needed by all SSL stubs
	#define SSL_KEYS_SIZE		((uint32_t)sizeof(SSL_KEYS))				// Amount of space needed by a single SSL key
	#define SSL_KEYS_SPACE		(SSL_KEYS_SIZE*SSL_MAX_CONNECTIONS)		// Amount of space needed by all SSL key
	#define SSL_HASH_SIZE		((uint32_t)sizeof(HASH_SUM))				// Amount of space needed by a single SSL hash
	#define SSL_HASH_SPACE		((uint32_t)(SSL_HASH_SIZE*SSL_MAX_HASHES))	// Amount of space needed by all SSL hash
	#define SSL_BUFFER_SIZE		((uint32_t)sizeof(SSL_BUFFER))				// Amount of space needed by a single SSL buffer
	#define SSL_BUFFER_SPACE	(SSL_BUFFER_SIZE*SSL_MAX_BUFFERS)		// Amount of space needed by all SSL buffer
	#define SSL_SESSION_SIZE	((uint32_t)sizeof(SSL_SESSION))			// Amount of space needed by a single SSL session
	#define SSL_SESSION_SPACE	(SSL_SESSION_SIZE*SSL_MAX_SESSIONS)		// Amount of space needed by all SSL session
	
	// Total space needed by all SSL storage requirements
	#define RESERVED_SSL_MEMORY ((uint32_t)(SSL_STUB_SPACE + SSL_KEYS_SPACE + SSL_HASH_SPACE + SSL_BUFFER_SPACE + SSL_SESSION_SPACE))


#endif  // __SSL_PRIVATE_H_

