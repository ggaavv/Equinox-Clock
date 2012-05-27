/*******************************************************************************
  MRF24W Driver API Interface

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

/*******************************************************************************
FileName:  wf_api.h 
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

#ifndef __WF_API_H_
#define __WF_API_H_


#include <stdint.h>
#include <stdbool.h>

#include "system_profile.h"

/*----------------------------------------------------------------------------*/
/* This block of defines allows for code and data reduction by removing       */
/* WiFi driver code and or data that is not needed by the application.        */
/* Comment out those function blocks that are not needed.                     */
/*----------------------------------------------------------------------------*/
//#define USE_GRATUITOUS_ARP


/*
*********************************************************************************************************
*                                           DEFINES                               
*********************************************************************************************************
*/
/*-----------------------------*/
/* WiFi Driver Version Number  */
/*-----------------------------*/
#define WF_HOST_DRIVER_VERSION_NUMBER    "3.0.0"

/* API defines */
#define WF_MAX_NUM_CHANNELS             (14)
#define WF_MAX_NUM_CONNECTION_PROFILES  (8)
#define WF_CP_LIST_LENGTH               (8)
#define WF_MAX_SSID_LENGTH              (32)
#define WF_BSSID_LENGTH                 (6)
#define WF_RETRY_FOREVER                (255)
#define WF_CHANNEL_LIST_LENGTH          (14)
#define WF_MAX_SECURITY_KEY_LENGTH      (64)

#define WF_MIN_NUM_CPID                 (1)
#define WF_MAX_NUM_CPID                 (8)

#define WF_NO_CPID_ACTIVE               (0)

#define WF_RTS_THRESHOLD_MAX            (2347) /* maximum RTS threshold size in bytes */

#define WF_MAX_NUM_RATES                (8)

/* Key size defines */
#define WF_MIN_WPA_PASS_PHRASE_LENGTH         (8)
#define WF_MIN_WPA_PASS_PHRASE_LENGTH         (8)
#define WF_MAX_WPA_PASS_PHRASE_LENGTH        (64)
#define WF_MAX_WPA2_PASS_PHRASE_LENGTH       (64)
#define WF_WPA_KEY_LENGTH                    (32)
#define WF_WPA2_KEY_LENGTH                   (32)

/*------------------------------------------------------------------------------*/
/* These are error codes returned in the result field of a management response. */
/*------------------------------------------------------------------------------*/
#define WF_SUCCESS                                              ((uint16_t)1)
#define WF_ERROR_INVALID_SUBTYPE                                ((uint16_t)2)
#define WF_ERROR_OPERATION_CANCELLED                            ((uint16_t)3)
#define WF_ERROR_FRAME_END_OF_LINE_OCCURRED                     ((uint16_t)4)
#define WF_ERROR_FRAME_RETRY_LIMIT_EXCEEDED                     ((uint16_t)5)
#define WF_ERROR_EXPECTED_BSS_VALUE_NOT_IN_FRAME                ((uint16_t)6)
#define WF_ERROR_FRAME_SIZE_EXCEEDS_BUFFER_SIZE                 ((uint16_t)7)
#define WF_ERROR_FRAME_ENCRYPT_FAILED                           ((uint16_t)8)
#define WF_ERROR_INVALID_PARAM                                  ((uint16_t)9)
#define WF_ERROR_AUTH_REQ_ISSUED_WHILE_IN_AUTH_STATE            ((uint16_t)10)
#define WF_ERROR_ASSOC_REQ_ISSUED_WHILE_IN_ASSOC_STATE          ((uint16_t)11)
#define WF_ERROR_INSUFFICIENT_RESOURCES                         ((uint16_t)12)
#define WF_ERROR_TIMEOUT_OCCURRED                               ((uint16_t)13)
#define WF_ERROR_BAD_EXCHANGE_ENCOUNTERED_IN_FRAME_RECEPTION    ((uint16_t)14)
#define WF_ERROR_AUTH_REQUEST_REFUSED                           ((uint16_t)15)
#define WF_ERROR_ASSOCIATION_REQUEST_REFUSED                    ((uint16_t)16)
#define WF_ERROR_PRIOR_MGMT_REQUEST_IN_PROGRESS                 ((uint16_t)17)
#define WF_ERROR_NOT_IN_JOINED_STATE                            ((uint16_t)18)
#define WF_ERROR_NOT_IN_ASSOCIATED_STATE                        ((uint16_t)19)
#define WF_ERROR_NOT_IN_AUTHENTICATED_STATE                     ((uint16_t)20)
#define WF_ERROR_SUPPLICANT_FAILED                              ((uint16_t)21)
#define WF_ERROR_UNSUPPORTED_FEATURE                            ((uint16_t)22)
#define WF_ERROR_REQUEST_OUT_OF_SYNC                            ((uint16_t)23)
#define WF_ERROR_CP_INVALID_ELEMENT_TYPE                        ((uint16_t)24)
#define WF_ERROR_CP_INVALID_PROFILE_ID                          ((uint16_t)25)
#define WF_ERROR_CP_INVALID_DATA_LENGTH                         ((uint16_t)26)
#define WF_ERROR_CP_INVALID_SSID_LENGTH                         ((uint16_t)27)
#define WF_ERROR_CP_INVALID_SECURITY_TYPE                       ((uint16_t)28)
#define WF_ERROR_CP_INVALID_SECURITY_KEY_LENGTH                 ((uint16_t)29)
#define WF_ERROR_CP_INVALID_WEP_KEY_ID                          ((uint16_t)30)
#define WF_ERROR_CP_INVALID_NETWORK_TYPE                        ((uint16_t)31)
#define WF_ERROR_CP_INVALID_ADHOC_MODE                          ((uint16_t)32)
#define WF_ERROR_CP_INVALID_SCAN_TYPE                           ((uint16_t)33)
#define WF_ERROR_CP_INVALID_CP_LIST                             ((uint16_t)34)
#define WF_ERROR_CP_INVALID_CHANNEL_LIST_LENGTH                 ((uint16_t)35)  
#define WF_ERROR_NOT_CONNECTED                                  ((uint16_t)36)
#define WF_ERROR_ALREADY_CONNECTING                             ((uint16_t)37)
#define WF_ERROR_DISCONNECT_FAILED                              ((uint16_t)38)
#define WF_ERROR_NO_STORED_BSS_DESCRIPTOR                       ((uint16_t)39)
#define WF_ERROR_INVALID_MAX_POWER                              ((uint16_t)40)
#define WF_ERROR_CONNECTION_TERMINATED                          ((uint16_t)41)
#define WF_ERROR_HOST_SCAN_NOT_ALLOWED                          ((uint16_t)42)
#define WF_ERROR_INVALID_WPS_PIN                                ((uint16_t)44)


/*---------------------------------------------------------------------*/
/* Used for eventNotificationField bit mask in tWFCAElements structure */
/*---------------------------------------------------------------------*/
#define WF_NOTIFY_CONNECTION_ATTEMPT_SUCCESSFUL     ((uint8_t)(0x01))  
#define WF_NOTIFY_CONNECTION_ATTEMPT_FAILED         ((uint8_t)(0x02))  
#define WF_NOTIFY_CONNECTION_TEMPORARILY_LOST       ((uint8_t)(0x04))  
#define WF_NOTIFY_CONNECTION_PERMANENTLY_LOST       ((uint8_t)(0x08))  
#define WF_NOTIFY_CONNECTION_REESTABLISHED          ((uint8_t)(0x10))  
#define WF_NOTIFY_ALL_EVENTS                        ((uint8_t)(0x1f))

/*---------------------------------------------------------------------*/
/* Used for Tx mode selection */
/*---------------------------------------------------------------------*/
#define WF_TXMODE_G_RATES		0
#define WF_TXMODE_B_RATES		1
#define WF_TXMODE_LEGACY_RATES	2

/*------------------------------------------------------------------------------------------*/
/* Multicast Filter ID's                                                                    */
/* Infrastructure can use 2,3,4,5 and AdHoc can only use 4,5.  Use 4,5 which works for both */
/*------------------------------------------------------------------------------------------*/
#define WF_MULTICAST_FILTER_1       (4) 
#define WF_MULTICAST_FILTER_2       (5)
#define WF_MASK_DEAUTH_REASONCODE	((uint8_t)0x80)
#define WF_MASK_DISASSOC_REASONCODE	((uint8_t)0x40)

#define WF_SCAN_ALL ((uint8_t)(0xff))
#define WF_HWRSSIVAL_MAX	200	/* hw RSSI reference value to be used to derive real RSSI */
/*
*********************************************************************************************************
*                                           DATA TYPES                               
*********************************************************************************************************
*/

/*----------------------------------------------------------------------------*/
/* Events that can be invoked in WF_ProcessEvent().  Note that the            */
/* connection events are optional, all other events the app must be notified. */
/*----------------------------------------------------------------------------*/

#define WF_EVENT_CONNECTION_SUCCESSFUL          (1)   /* Connection attempt to network successful               */
#define WF_EVENT_CONNECTION_FAILED              (2)   /* Connection attempt failed                              */


#define WF_EVENT_CONNECTION_TEMPORARILY_LOST    (3)   /* Connection lost; MRF24W attempting to reconnect     */
#define WF_EVENT_CONNECTION_PERMANENTLY_LOST    (4)   /* Connection lost; MRF24W no longer trying to connect */  
#define WF_EVENT_CONNECTION_REESTABLISHED       (5)

#define WF_EVENT_FLASH_UPDATE_SUCCESSFUL        (6)   /* Update to FLASH successful                             */
#define WF_EVENT_FLASH_UPDATE_FAILED            (7)   /* Update to FLASH failed                                 */


#define WF_EVENT_KEY_CALCULATION_COMPLETE       (8)   /* Key calculation has completed                          */

#define WF_EVENT_SCAN_RESULTS_READY             (9)   /* scan results are ready                                 */ 
#define WF_EVENT_IE_RESULTS_READY               (10)  /* IE data ready                                          */


#define WF_EVENT_INVALID_WPS_PIN                (12)   /* Invalid WPS pin was entered                            */


typedef struct WFMacStatsStruct 
{
    /**
      Number of frames received with the Protected Frame subfield of the Frame 
      Control field set to zero and the value of dot11ExcludeUnencrypted causes 
      that frame to be discarded.
      */
    uint32_t MibWEPExcludeCtr;    
    uint32_t MibTxBytesCtr; // Total number of Tx bytes that have been transmitted

    /**
      Number of frames successfully transmitted that had the multicast bit set 
      in the destination MAC address.
      */
	uint32_t MibTxMulticastCtr;
    /**
      Number of Tx frames that failed due to the number of transmits exceeding 
      the retry count.
      */
	uint32_t MibTxFailedCtr;
	uint32_t MibTxRtryCtr; // Number of times a transmitted frame needed to be retried 
	uint32_t MibTxMultRtryCtr; // Number of times a frame was successfully transmitted after more than one retransmission.
	uint32_t MibTxSuccessCtr; // Number of Tx frames successfully transmitted.
    uint32_t MibRxDupCtr; // Number of frames received where the Sequence Control field indicates a duplicate.
	uint32_t MibRxCtsSuccCtr; // Number of CTS frames received in response to an RTS frame.
	uint32_t MibRxCtsFailCtr; // Number of times an RTS frame was not received in response to a CTS frame.
	uint32_t MibRxAckFailCtr; // Number of times an Ack was not received in response to a Tx frame.
	uint32_t MibRxBytesCtr; // Total number of Rx bytes received.
	uint32_t MibRxFragCtr; // Number of successful received frames (management or data)
	uint32_t MibRxMultCtr; // Number of frames received with the multicast bit set in the destination MAC address.
	uint32_t MibRxFCSErrCtr; // Number of frames received with an invalid Frame Checksum (FCS).

    /**
      Number of frames received where the Protected Frame subfield of the Frame Control Field is set to 
      one and the WEPOn value for the key mapped to the transmitter’s MAC address indicates the frame 
      should not have been encrypted.
      */
	uint32_t MibRxWEPUndecryptCtr;
	uint32_t MibRxFragAgedCtr; // Number of times that fragments ‘aged out’, or were not received in the allowable time.
	uint32_t MibRxMICFailureCtr; // Number of MIC failures that have occurred.
} tWFMacStats;


/*-------------------------------------------------------*/
/* Security Type defines                                 */
/* Used in WF_CPSet/GetSecurityType WF_CPSet/GetElements */
/*-------------------------------------------------------*/
#define WF_SECURITY_OPEN                         (0)
#define WF_SECURITY_WEP_40                       (1)
#define WF_SECURITY_WEP_104                      (2)
#define WF_SECURITY_WPA_WITH_KEY                 (3)
#define WF_SECURITY_WPA_WITH_PASS_PHRASE         (4)
#define WF_SECURITY_WPA2_WITH_KEY                (5)
#define WF_SECURITY_WPA2_WITH_PASS_PHRASE        (6)
#define WF_SECURITY_WPA_AUTO_WITH_KEY            (7)
#define WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE    (8)


/* Wep key types */
#define WF_SECURITY_WEP_SHAREDKEY  (0)
#define WF_SECURITY_WEP_OPENKEY    (1)


/*---------------------------------------------------------------------*/
/* Network Type defines                                                */
/* Used in WF_CPSet/GetNetworkType, WF_CPSetElements, WF_CPGetElements */
/*---------------------------------------------------------------------*/
#define WF_INFRASTRUCTURE                       (1)
#define WF_ADHOC                                (2)
#define WF_P2P                                  (3) 

/*--------------------------------------------------------*/
/* Ad Hoc behavior defines                                */
/* Used in WF_CPSet/GetAdhocBehavor, WF_CPSet/GetElements */
/*--------------------------------------------------------*/
#define WF_ADHOC_CONNECT_THEN_START             (0)
#define WF_ADHOC_CONNECT_ONLY                   (1)
#define WF_ADHOC_START_ONLY                     (2)

/*----------------------------------------------------*/
/* Scan type defines                                  */
/* Used in WF_CASet/GetScanType, WF_CASet/GetElements */
/*----------------------------------------------------*/
#define WF_ACTIVE_SCAN                          (1)
#define WF_PASSIVE_SCAN                         (2)                      

/*-----------------------------------------------------------------------------------------*/
/* Beacon Timeout and Deauth defines                                                       */
/* Used in WF_CASet/GetBeaconTimeoutAction, WF_CASet/GetDeauthAction, WF_CASet/GetElements */
/*-----------------------------------------------------------------------------------------*/
#define WF_DO_NOT_ATTEMPT_TO_RECONNECT          (0)
#define WF_ATTEMPT_TO_RECONNECT                 (1) 


#define WF_DISABLED                             (0)
#define WF_ENABLED                              (1)

/* eventInfo defines for WF_ProcessEvent(), case WF_EVENT_CONNECTION_FAILED */
/* Also value for index 3 of WF_CONNECTION_FAILED_EVENT_SUBTYPE */

#define WF_JOIN_FAILURE                         (2)
#define WF_AUTHENTICATION_FAILURE               (3)
#define WF_ASSOCIATION_FAILURE                  (4)
#define WF_WEP_HANDSHAKE_FAILURE                (5)
#define WF_PSK_CALCULATION_FAILURE              (6)
#define WF_PSK_HANDSHAKE_FAILURE                (7)
#define WF_ADHOC_JOIN_FAILURE                   (8)
#define WF_SECURITY_MISMATCH_FAILURE            (9)
#define WF_NO_SUITABLE_AP_FOUND_FAILURE         (10)
#define WF_RETRY_FOREVER_NOT_SUPPORTED_FAILURE  (11)
#define WF_LINK_LOST							(12)
#define WF_TKIP_MIC_FAILURE                     (13)
#define WF_RSN_MIXED_MODE_NOT_SUPPORTED         (14)
#define WF_RECV_DEAUTH							(15)
#define WF_RECV_DISASSOC						(16)
#define WF_WPS_FAILURE							(17)
#define WF_P2P_FAILURE							(18)

/* Reason Codes */
#define WF_UNSPECIFIED                          (1)
#define WF_REASON_PREV_AUTH_NOT_VALID           (2)
#define WF_DEAUTH_LEAVING                       (3)
#define WF_DISASSOC_DUE_TO_INACTIVITY           (4)
#define WF_DISASSOC_AP_BUSY                     (5)
#define WF_CLASS2_FRAME_FROM_NONAUTH_STA        (6)
#define WF_CLASS3_FRAME_FROM_NONASSOC_STA       (7)
#define WF_DISASSOC_STA_HAS_LEFT                (8)
#define WF_STA_REQ_ASSOC_WITHOUT_AUTH           (9)
#define WF_INVALID_IE                           (13)
#define WF_MIC_FAILURE                          (14)
#define WF_4WAY_HANDSHAKE_TIMEOUT               (15)
#define WF_GROUP_KEY_HANDSHAKE_TIMEOUT          (16)
#define WF_IE_DIFFERENT                         (17)
#define WF_INVALID_GROUP_CIPHER                 (18)
#define WF_INVALID_PAIRWISE_CIPHER              (19)
#define WF_INVALID_AKMP                         (20)
#define WF_UNSUPP_RSN_VERSION                   (21)
#define WF_INVALID_RSN_IE_CAP                   (22)
#define WF_IEEE8021X_FAILED                     (23)
#define WF_CIPHER_SUITE_REJECTED                (24)


/* eventInfo defines for WF_ProcessEvent(), case WF_EVENT_CONNECTION_TEMPORARILY_LOST */
#define WF_BEACON_TIMEOUT                       (1)
#define WF_DEAUTH_RECEIVED                      (2)
#define WF_DISASSOCIATE_RECEIVED                (3)

/* Status Codes */
#define WF_UNSPECIFIED_FAILURE          (1)
#define WF_CAPS_UNSUPPORTED             (10)
#define WF_REASSOC_NO_ASSOC             (11)
#define WF_ASSOC_DENIED_UNSPEC          (12)
#define WF_NOT_SUPPORTED_AUTH_ALG       (13)
#define WF_UNKNOWN_AUTH_TRANSACTION     (14)
#define WF_CHALLENGE_FAIL               (15)
#define WF_AUTH_TIMEOUT                 (16)
#define WF_AP_UNABLE_TO_HANDLE_NEW_STA  (17)
#define WF_ASSOC_DENIED_RATES           (18)
#define WF_ASSOC_DENIED_NOSHORTPREAMBLE (19)
#define WF_ASSOC_DENIED_NOPBCC          (20)
#define WF_ASSOC_DENIED_NOAGILITY       (21)
#define WF_ASSOC_DENIED_NOSHORTTIME     (25)
#define WF_ASSOC_DENIED_NODSSSOFDM      (26)
#define WF_S_INVALID_IE                 (40)
#define WF_S_INVALID_GROUPCIPHER        (41)
#define WF_S_INVALID_PAIRWISE_CIPHER    (42)
#define WF_S_INVALID_AKMP               (43)
#define WF_UNSUPPORTED_RSN_VERSION      (44)
#define WF_S_INVALID_RSN_IE_CAP         (45)
#define WF_S_CIPHER_SUITE_REJECTED      (46)
#define WF_TIMEOUT                      (47)



/* WiFi Device Types */
#define MRF24WB0M_DEVICE                (1)


    /* Domain Codes */
#define WF_DOMAIN_FCC                           (0)           /* Available Channels: 1 - 11 */
#define WF_DOMAIN_IC                            (1)           /* Available Channels: 1 - 11 */
#define WF_DOMAIN_ETSI                          (2)           /* Available Channels: 1 - 13 */
#define WF_DOMAIN_SPAIN    (3)           /* Available Channels: 10 - 11 */
#define WF_DOMAIN_FRANCE   (4)           /* Available Channels: 10 - 13 */
#define WF_DOMAIN_JAPAN_A                       (5)           /* Available Channels: 14     */
#define WF_DOMAIN_JAPAN_B                       (6)           /* Available Channels: 1 - 13 */


/* Power save states */
#define WF_PS_HIBERNATE                         (1)
#define WF_PS_PS_POLL_DTIM_ENABLED              (2)
#define WF_PS_PS_POLL_DTIM_DISABLED             (3)
#define WF_PS_OFF                               (4)

/* Hibernate states */
#define WF_HB_NO_SLEEP 		                    (0)
#define WF_HB_ENTER_SLEEP 	                    (1)
#define WF_HB_WAIT_WAKEUP 	                    (2)

/* Pin Level */
#define WF_LOW                                  (0)
#define WF_HIGH                                 (1)

/*-----------------------------------------------------------------------*/
/* defines used for the p_currentCpID value in WF_CMGetConnectionState() */
/*-----------------------------------------------------------------------*/
#define WF_CURRENT_CPID_NONE                    (0)
#define WF_CURRENT_CPID_LIST                    (0xff)

/* Connection States */
#define WF_CSTATE_NOT_CONNECTED                 (1)
#define WF_CSTATE_CONNECTION_IN_PROGRESS        (2)
#define WF_CSTATE_CONNECTED_INFRASTRUCTURE      (3)
#define WF_CSTATE_CONNECTED_ADHOC               (4)
#define WF_CSTATE_RECONNECTION_IN_PROGRESS      (5)
#define WF_CSTATE_CONNECTION_PERMANENTLY_LOST   (6)


/* eventInfo define for WF_ProcessEvent() when no additional info is supplied */
#define WF_NO_ADDITIONAL_INFO       ((uint16_t)0xffff)

#define ENABLE_WPS_PRINTS	(1 << 0)
#define ENABLE_P2P_PRINTS	(1 << 1)

/*-----------------------------*/
/* Connection Profile Elements */
/*-----------------------------*/

// Connection profile elements structure
typedef struct WFCPElementsStruct
{
    /** 
      SSID, which must be less than or equal to 32 characters.  Set to all 0’s 
      if not being used.  If ssidLength is 0 this field is ignored.  If SSID is 
      not defined then the MRF24W, when using this profile to connect, will 
      scan all channels within its regional domain.

      Default: SSID not used. 
      */
    uint8_t  ssid[WF_MAX_SSID_LENGTH];
    /**
      Basic Service Set Identifier, always 6 bytes.  This is the 48-bit MAC of 
      the SSID.  It is an optional field that can be used to specify a specific 
      SSID if more than one AP exists with the same SSID.  This field can also 
      be used in lieu of the SSID.  

      Set each byte to 0xFF if BSSID is not going to be used.
      Default: BSSID not used (all FF’s)
      */
    uint8_t  bssid[WF_BSSID_LENGTH];
    /**
      Number of ASCII bytes in ssid.  Set to 0 is SSID is not going to be used.

      Default: 0
      */
    uint8_t  ssidLength;
    /**
      Designates the desired security level for the connection.  Choices are:
      <table>
        WF_SECURITY_OPEN                        No security encryption used.
        WF_SECURITY_WEP_40                      Use WEP security.  
                                                WEP key, using four 5-byte keys will be provided in securityKey.
                                                 Note that only open authentication is supported for WEP.
        WF_SECURITY_WEP_104                     Use WEP security.  
                                                 WEP key, using four 13-byte keys will be provided in securityKey.
                                                 Note that only open authentication is supported for WEP.
        WF_SECURITY_WPA_WITH_KEY                Use WPA security.  
                                                 Binary PSK (Pre-shared Key) key will be provided in securityKey.
        WF_SECURITY_WPA_WITH_PASS_PHRASE        Use WPA security.  
                                                 ASCII WPA passphrase will be provided in securityKey and, 
                                                 after a call to WF_CMConnect(), the MRF24W will calculate 
                                                 the PSK key (which can take up to 30 seconds).
        WF_SECURITY_WPA2_WITH_KEY               Use WPA-2 security.  
                                                 Binary WPA-2 key will be provided in securityKey.
        WF_SECURITY_WPA2_WITH_PASSPHRASE        Use WPA-2 security.
                                                 ASCII WPA-2 passphrase will be provided in securityKey and, 
                                                 after a call to WF_CMConnect(), the MRF24W will calculate 
                                                 the PSK key (which can take up to 30 seconds).
        WF_SECURITY_WPA_AUTO_WITH_KEY           Same as WF_SECURITY_WPA_WITH_KEY or WF_SECURITY_WPA2_WITH_KEY 
                                                 except connection manager will connect to the AP using highest 
                                                 level security the AP supports (WPA or WPA2).
        WF_SECURITY_WPA_AUTO_WITH_PASSPHRASE    Same as WF_SECURITY_WPA_WITH_PASS_PHRASE or 
                                                 WF_SECURITY_WPA2_WITH_PASS_PHRASE except connection manager 
                                                 will connect to the AP using highest level security the AP 
                                                 supports (WPA or WPA2).
      </table>
      Default: WF_SECURITY_OPEN
      */
    uint8_t  securityType;
    /**
      Set to NULL if securityType is WF_SECURITY_OPEN.  If securityKeyLength is 0 
      this field is ignored.
      <table>
        WEP Keys        If using WEP this field must contain 4 keys.  Each key must be 
                         either 5 bytes in length (if securityType is WF_SECURITY_WEP_40) 
                         or 13 bytes in length  (if securityType is WF_SECURITY_WEP_104).  
                         The keys must be contiguous within this field.  For example, if 
                         using 5 bytes keys the first key starts at index 0, the second 
                         key at index 5, the third key at index 10, and the last key at 
                         index 15.  Unused keys should be set to all 0’s.
        WPA/WPA2 Keys   If using WPA or WPA2 you can provide the actual binary key or 
                         ASCII passphrase used to generate a key.  [64 byte array covers 
                         all cases of keys or passphrases].  If using this field for a 
                         security passphrase the MRF24W will need to calculate the 
                         binary key after the call to WF_CMConnect() – this can add about 
                         30 seconds to the connection time.
      </table>
      Default: No security key defined
      */
    uint8_t  securityKey[WF_MAX_SECURITY_KEY_LENGTH];
    /**
      Number of bytes used in the securityKey.  Set to 0 if securityType is WF_SECURITY_OPEN.
      <table>
        WEP Keys        If securityType is WF_SECURITY_WEP_40 or WF_SECURITY_WEP_104 
                         then this field is the length of the four WEP keys.

                         Range is  
                         20 if securityType is WF_SECURITY_WEP_40 (four 5-byte keys),
                         52 if securityType is WF_SECURITY_WEP_104 (four 13-byte keys)
        WPA/WPA2 Keys   If securityType is one of the WPA or WPA2 choices then this 
                         field is the number of bytes in the binary key or the 
                         passphrase, whichever is being used.

      </table>
      Default: 0
      */
    uint8_t  securityKeyLength;
    /**
      This field is only used if securityType is WF_SECURITY_WEP.  This field 
      designates which of the four WEP keys defined in securityKey to use when 
      connecting to a WiFi network.  The range is 0 thru 3, with the default
      being 0.
      */
    uint8_t  wepDefaultKeyId;
    /**
      WF_INFRASTRUCTURE  or WF_ADHOC

      Default: WF_INFRASTRUCTURE  
      */
    uint8_t  networkType;
    /**
      Only applicable if networkType is WF_ADHOC.  Configures Adhoc behavior.  Choices are:
      <table>
        WF_ADHOC_CONNECT_THEN_START     Attempt to connect to existing network.  
                                         If that fails, then start a network.
        WF_ADHOC_CONNECT_ONLY           Connect only to an existing network.  
                                         Do not start a network.
        WF_ADHOC_START_ONLY             Only start a network.
      </table>
      Default: WF_ADHOC_CONNECT_THEN_START
      */
    uint8_t  adHocBehavior;
	/**
	1 - enable hidden ssid in adhoc mode
	*/
	uint8_t hiddenSSID;
	/**
	0- shared key, 1 - open key
	*/
	uint8_t wepKeyType;
} tWFCPElements;

/*-------------------------------*/
/* Connection Algorithm Elements */
/*-------------------------------*/
typedef struct WFCAElementsStruct
{
    /**
      This parameter is only used when PS Poll mode is enabled.  See 
      WF_PsPollEnable().  Number of 100ms intervals between instances when the 
      MRF24W wakes up to received buffered messages from the network.  Range
      is from 1 (100ms) to 6553.5 sec (~109 min).

      Note that the 802.11 standard defines the listen interval in terms of 
      Beacon Periods, which are typically 100ms.  If the MRF24W is communicating 
      to a network with a network that has Beacon Periods that is not 100ms it 
      will round up (or down) as needed to match the actual Beacon Period as 
      closely as possible.

      Important Note: If the listenInterval is modified while connected to a 
      network the MRF24W will automatically reconnect to the network with the 
      new Beacon Period value.  This may cause a temporary loss of data packets.
      */
    uint16_t  listenInterval;
    /**
      WF_ACTIVE_SCAN (Probe Requests sent out) or WF_PASSIVE_SCAN (listen only)

      Default: WF_ACTIVE_SCAN
      */
    uint8_t   scanType;
    /**
      Specifies RSSI restrictions when connecting.  This field is only used if:
      1.  The Connection Profile has not defined a SSID or BSSID, or  
      2.  An SSID is defined in the Connection Profile and multiple AP’s are discovered with the same SSID.
    
      <table>
        0       Connect to the first network found
        1-254   Only connect to a network if the RSSI is greater than or equal to the specified value.
        255     Connect to the highest RSSI found (default)
      </table>
      */
    uint8_t   rssi;
    /**
      <b>Note: Connection Profile lists are not yet supported.  This array should be set to all FF’s.</b>
      */
    uint8_t   connectionProfileList[WF_CP_LIST_LENGTH];
    /**
      This field is used to specify the number of retries for the single 
      connection profile before taking the connection lost action.

      Range 1 to 254 or WF_RETRY_FOREVER (255)

      Default is 3
      */
    uint8_t   listRetryCount;
    /**
      There are several connection-related events that can occur.  The Host has 
      the option to be notified (or not) when some of these events occur.  This 
      field controls event notification for connection-related events.
      <table>
        Bit 7   Bit 6   Bit 5   Bit 4   Bit 3   Bit 2   Bit 1   Bit 0
        -----   -----   -----   -----   -----   -----   -----   -----
        Not     Not     Not     Event   Event   Event   Event   Event
         used    used    used    E       D       C       B       A
      </table>
      The defines for each bit are shown below.
      <table>
        Event Code  #define
        ----------  -------
        A           WF_NOTIFY_CONNECTION_ATTEMPT_SUCCESSFUL  
        B           WF_NOTIFY_CONNECTION_ATTEMPT_FAILED
        C           WF_NOTIFY_CONNECTION_TEMPORARILY_LOST
        D           WF_NOTIFY_CONNECTION_PERMANENTLY_LOST
        E           WF_NOTIFY_CONNECTION_REESTABLISHED
      </table>
      If a bit is set, then the host will be notified if the associated event 
      occurs.  If the bit is not set then the host will not be notified if the 
      associated event occurs.   A #define, WF_NOTIFY_ALL_EVENTS, exists as a 
      shortcut to allow notification for all events. 

      Note that if an event is not in the above bit mask the application will 
      always be notified of the event via the call to WF_ProcessEvent().

      Default: WF_NOTIFY_ALL_EVENTS
      */
    uint8_t   eventNotificationAction;
    /**
      Specifies the action the Connection Manager should take if a Connection is 
      lost due to a Beacon Timeout.  
      If this field is set to WF_ATTEMPT_TO_RECONNECT then the number of attempts 
      is limited to the value in listRetryCount.

      Choices are:
      WF_ATTEMPT_TO_RECONNECT or WF_DO_NOT_ATTEMPT_TO_RECONNECT

      Default: WF_ATTEMPT_TO_RECONNECT
      */
    uint8_t   beaconTimeoutAction;
    /**
      Designates what action the Connection Manager should take if it receives a 
      Deauthentication message from the AP.  

      If this field is set to WF_ATTEMPT_TO_RECONNECT then the number of attempts 
      is limited to the value in listRetryCount.

      Choices are:
      WF_ATTEMPT_TO_RECONNECT or WF_DO_NOT_ATTEMPT_TO_RECONNECT

      Default: WF_ATTEMPT_TO_RECONNECT
      */
    uint8_t   deauthAction;
    /**
      List of one or more channels that the MRF24W should utilize when 
      connecting or scanning.  If numChannelsInList is set to 0 then this 
      parameter should be set to NULL.

      Default: All valid channels for the regional domain of the MRF24W (set 
      at manufacturing).
    */
    uint8_t   channelList[WF_CHANNEL_LIST_LENGTH];
    /**
      Number of channels in channelList.  If set to 0 then the MRF24W will 
      populate the list with all valid channels for the regional domain.

      Default: The number of valid channels for the regional domain of the 
      MRF24W (set at manufacturing).
      */
    uint8_t   numChannelsInList;
    /**
      Specifies the number of beacons that can be missed before the action 
      described in beaconTimeoutAction is taken.

      <table>
        0       * Not monitor the beacon timeout condition
                 * Will not indicate this condition to Host
        1-255   Beacons missed before disconnect event occurs and beaconTimeoutAction 
                 occurs.  If enabled, host will receive an event message indicating 
                 connection temporarily or permanently lost, and if retrying, a 
                 connection successful event.
      </table>
      Default: 0 (no monitoring or notification of beacon timeout)
      */
    uint8_t   beaconTimeout;
    /**
      The number of times to scan a channel while attempting to find a particular 
      access point.

      Default: 1
      */
    uint8_t   scanCount;
    uint8_t   pad1; 
    /**
      The minimum time (in milliseconds) the connection manager will wait for a 
      probe response after sending a probe request.  If no probe responses are 
      received in minChannelTime then the connection manager will go on to the 
      next channel, if any are left to scan, or quit.

      Default: 200ms
      */
    uint16_t  minChannelTime;
    /**
      If a probe response is received within minChannelTime then the connection 
      manager will continue to collect any additional probe responses up to 
      maxChannelTime before going to the next channel in the channelList.  Units 
      are in milliseconds.

      Default: 400ms
      */
    uint16_t  maxChannelTime;
    /**
      The number of microseconds to delay before transmitting a probe request 
      following the channel change event.

      Default: 20us
    */
    uint16_t  probeDelay;


} tWFCAElements;


/*--------------------------*/
/* used in WF_GetDeviceInfo */
/*--------------------------*/
typedef struct tWFDeviceInfoStruct
{
    uint8_t  deviceType;    /* MRF24W_DEVICE_TYPE  */
    uint8_t  romVersion;    /* const version number     */
    uint8_t  patchVersion;  /* Patch version number   */
} tWFDeviceInfo;


/*--------------*/
/* Scan Results */
/*--------------*/
typedef struct
{
    uint8_t      bssid[WF_BSSID_LENGTH]; // Network BSSID value
    uint8_t      ssid[WF_MAX_SSID_LENGTH]; // Network SSID value

    /**
      Access point configuration
      <table>
        Bit 7       Bit 6       Bit 5       Bit 4       Bit 3       Bit 2       Bit 1       Bit 0
        -----       -----       -----       -----       -----       -----       -----       -----
        WPA2        WPA         Preamble    Privacy     Reserved    Reserved    Reserved    IE
      </table>
      
      <table>
      IE        1 if AP broadcasting one or more Information Elements, else 0
      Privacy   0 : AP is open (no security)
                 1: AP using security,  if neither WPA and WPA2 set then security is WEP.
      Preamble  0: AP transmitting with short preamble
                 1: AP transmitting with long preamble
      WPA       Only valid if Privacy is 1.
                 0: AP does not support WPA
                 1: AP supports WPA
      WPA2      Only valid if Privacy is 1.
                 0: AP does not support WPA2
                 1: AP supports WPA2
      </table>
      */
    uint8_t      apConfig;
    uint8_t      reserved;
    uint16_t     beaconPeriod; // Network beacon interval          
    uint16_t     atimWindow; // Only valid if bssType = WF_INFRASTRUCTURE

    /**
      List of Network basic rates.  Each rate has the following format:
      
	  Bit 7
      * 0 – rate is not part of the basic rates set
      * 1 – rate is part of the basic rates set

	  Bits 6:0 
      Multiple of 500kbps giving the supported rate.  For example, a value of 2 
      (2 * 500kbps) indicates that 1mbps is a supported rate.  A value of 4 in 
      this field indicates a 2mbps rate (4 * 500kbps).
      */
    uint8_t      basicRateSet[WF_MAX_NUM_RATES]; 
    uint8_t      rssi; // Signal strength of received frame beacon or probe response
    uint8_t      numRates; // Number of valid rates in basicRates
    uint8_t      DtimPeriod; // Part of TIM element
    uint8_t      bssType; // WF_INFRASTRUCTURE or WF_ADHOC
    uint8_t      channel; // Channel number
    uint8_t      ssidLen; // Number of valid characters in ssid

} tWFScanResult; 

#if defined(WF_CM_DEBUG)
typedef struct
{
    uint8_t     byte[12*4];  // Currently, CM has 12 states; 4-byte for each state info entry.
} tWFCMInfoFSMStats;
#endif



/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES                               
*********************************************************************************************************
*/

/*--------------------------*/
/* Initialization Functions */
/*--------------------------*/
bool WF_Init(const void* pNetIf);

/*--------------------------*/
/* Connection Functions */
/*--------------------------*/
void WF_Connect(const void* hMac);

/*-------------------*/
/* Version functions */
/*-------------------*/
void WF_GetDeviceInfo(tWFDeviceInfo *p_deviceInfo);

/*----------------------------*/
/* WF Driver process function */
/*----------------------------*/
void WFProcess(void);

/*---------------------------------------*/
/* WF Driver External Interrupt function */
/* Must be called when:                  */
/*  1) External interrupt is enabled AND */
/*  2) EXINT line is asserted (low)      */
/*---------------------------------------*/
void WFEintISR(void);

/*-----------------------*/
/* MAC Address Functions */
/*-----------------------*/
void WF_SetMacAddress(uint8_t *p_mac);       
void WF_GetMacAddress(uint8_t *p_mac);


/*------------------------------*/
/* Connection Profile Functions */
/*------------------------------*/
void WF_CPCreate(uint8_t *p_CpId);
void WF_CPDelete(uint8_t CpId);
void WF_CPGetIds(uint8_t *cpIdList);

void WF_CPSetElements(uint8_t CpId, tWFCPElements *p_elements);
void WF_CPGetElements(uint8_t CpId, tWFCPElements *p_elements);

void WF_CPSetSsid(uint8_t CpId, uint8_t *p_ssid,  uint8_t ssidLength);
void WF_CPGetSsid(uint8_t CpId, uint8_t *p_ssid, uint8_t *p_ssidLength);
void WF_CPSetBssid(uint8_t CpId, uint8_t *p_bssid);
void WF_CPGetBssid(uint8_t CpId, uint8_t *p_bssid);
void WF_CPSetSecurity(uint8_t CpId, 
                      uint8_t securityType,
                      uint8_t wepKeyIndex,
                      uint8_t *p_securityKey,
                      uint8_t securityKeyLength);
void WF_CPGetSecurity(uint8_t CpId, 
                      uint8_t *p_securityType,
                      uint8_t *p_wepKeyIndex,
                      uint8_t *p_securityKey,
                      uint8_t *p_securityKeyLength);
void WF_CPSetDefaultWepKeyIndex(uint8_t CpId, uint8_t defaultWepKeyIndex);
void WF_CPGetDefaultWepKeyIndex(uint8_t CpId, uint8_t *p_defaultWepKeyIndex);
void WF_CPSetNetworkType(uint8_t CpId, uint8_t networkType);
void WF_CPGetNetworkType(uint8_t CpId, uint8_t *p_networkType);
void WF_CPSetWepKeyType(uint8_t CpId, uint8_t wepKeyType);
void WF_CPSetAdHocBehavior(uint8_t CpId, uint8_t adHocBehavior);
void WF_CPGetAdHocBehavior(uint8_t CpId, uint8_t *p_adHocBehavior);
    

/*--------------------------------*/
/* Connection Algorithm Functions */
/*--------------------------------*/
void WF_CASetElements(tWFCAElements *p_elements);
void WF_CAGetElements(tWFCAElements *p_elements);

void WF_CASetScanType(uint8_t scanType);
void WF_CAGetScanType(uint8_t *p_scanType);
void WF_CASetRssi(uint8_t rssi);
void WF_CAGetRssi(uint8_t *p_rssi);
void WF_CASetConnectionProfileList(uint8_t cpList[WF_CP_LIST_LENGTH]);
void WF_CAGetConnectionProfileList(uint8_t cpList[WF_CP_LIST_LENGTH]);
void WF_CASetListRetryCount(uint8_t listRetryCount);
void WF_CAGetListRetryCount(uint8_t *p_listRetryCount);
void WF_CASetEventNotificationAction(uint8_t eventNotificationAction);
void WF_CAGetEventNotificationAction(uint8_t *p_eventNotificationAction);
void WF_CASetBeaconTimeoutAction(uint8_t beaconTimeoutAction);
void WF_CAGetBeaconTimeoutAction(uint8_t *p_beaconTimeoutAction);
void WF_CASetDeauthAction(uint8_t deauthAction);
void WF_CAGetDeauthAction(uint8_t *p_deauthAction);
void WF_CASetChannelList(uint8_t *p_channelList, uint8_t numChannels);
void WF_CAGetChannelList(uint8_t *p_channelList, uint8_t *p_numChannels);
void WF_CASetListenInterval(uint16_t listenInterval);
void WF_CAGetListenInterval(uint16_t *p_listenInterval);
void WF_CASetBeaconTimeout(uint8_t beaconTimeout);
void WF_CAGetBeaconTimeout(uint8_t *p_beaconTimeout);
void WF_CASetScanCount(uint8_t scanCount);
void WF_CAGetScanCount(uint8_t *p_scanCount);
void WF_CASetMinChannelTime(uint16_t minChannelTime);
void WF_CAGetMinChannelTime(uint16_t *p_minChannelTime);
void WF_CASetMaxChannelTime(uint16_t minChannelTime);
void WF_CAGetMaxChannelTime(uint16_t *p_minChannelTime);
void WF_CASetProbeDelay(uint16_t probeDelay);
void WF_CAGetProbeDelay(uint16_t *p_probeDelay);
void WF_CASetBeaconPeriod(uint16_t beaconPeriod);
void WF_CAGetBeaconPeriod(uint16_t *beaconPeriod);

/*--------------------------------*/
/* Connection Manager Functions   */
/*--------------------------------*/
void WF_CMConnect(uint8_t CpId);
void WF_CMDisconnect(void);
void WF_CMGetConnectionState(uint8_t *p_state, uint8_t *p_currentCpId);
void WF_CMCheckConnectionState(uint8_t *p_state, uint8_t *p_currentCpId);


/*----------------------------*/
/* Tx Power Control Functions */
/*----------------------------*/
void WF_TxPowerSetMinMax(int8_t minTxPower, int8_t maxTxPower);
void WF_TxPowerGetMinMax(int8_t *p_minTxPower, int8_t *p_maxTxPower);
void WF_FixTxRateWithMaxPower(bool oneMegaBps);
void WF_TxPowerGetFactoryMax(int8_t *p_factoryMaxTxPower);


/*----------------------------*/
/* Power Management Functions */
/*----------------------------*/
void WF_PsPollDisable(void);
void WF_PsPollEnable(bool rxDtim, bool aggressive);
void WF_GetPowerSaveState(uint8_t *p_powerSaveState);
void WF_HibernateEnable(void);

/*-------------------------*/
/* RTS Threshold Functions */
/*-------------------------*/
void WF_SetRtsThreshold(uint16_t rtsThreshold);
void WF_GetRtsThreshold(uint16_t *p_rtsThreshold);

/*---------------------------*/
/* Regional Domain Functions */
/*---------------------------*/
void WF_SetRegionalDomain(uint8_t regionalDomain);     /* see tWFRegDomain enumerated types */
void WF_GetRegionalDomain(uint8_t *p_regionalDomain);  /* see tWFRegDomain enumerated types */

/*---------------------*/
/* Multicast Functions */
/*---------------------*/
void WF_SetMultiCastFilter(uint8_t multicastFilterId, uint8_t multicastAddress[6]);
void WF_GetMultiCastFilter(uint8_t multicastFilterId, uint8_t multicastAddress[6]);


/* MAC Stats */
void WF_GetMacStats(tWFMacStats *p_macStats);

/*----------------*/
/* Scan Functions */
/*----------------*/
void WF_Scan(uint8_t CpId);
void WF_ScanGetResult(uint8_t         listIndex, 
                      tWFScanResult *p_scanResult);

/*------------------------------*/
/* External Interrupt Functions */
/*------------------------------*/
void WF_EintInit(void);
void WF_EintEnable(void);
void WF_EintDisable(void);
bool WF_EintIsDisabled(void);
void WFEintHandler(void); 
/* WF_EintIsPending - used by the WF Driver to test for whether */
/* external interrupts are pending.  The pending case is not something */
/* that should normally happen.  It says we have the interrupt line */
/* asserted but the WF_EINT_IF bit is not set, thus, no interrupt generated */
bool WF_EintIsPending(void);

/*---------------*/
/* SPI Functions */
/*---------------*/
bool     WF_SpiInit(void);
void     WF_SpiEnableChipSelect(void);
void     WF_SpiDisableChipSelect(void);
void     WFSpiTxRx(uint8_t   *p_txBuf, 
                   uint16_t  txLen, 
                   uint8_t   *p_rxBuf,
                   uint16_t  rxLen);


/*--------------------------*/
/* Event Handling Functions */
/*--------------------------*/
/*******************************************************************************
  Function:	
    void MRF24W_SetUserEvents(tWFEvents event, uint16_t eventInfo, bool isMgmt);

  Summary:
    Sets the current events from the MRF24W

  Description:

  Precondition:
  	TCPIP stack should be initialized.

  Parameters:
    event       -- current MRF24W traffic event.
    eventInfo   -- additional event info
                   This is not applicable to all events.
    isMgmt      -- specifies traffic or management event

  Returns:
  	None.
  	
  Remarks:
  	None.
  *****************************************************************************/
void MRF24W_SetUserEvents(uint8_t event, uint16_t eventInfo, bool isMgmt);

#if defined(WF_CM_DEBUG)
/*--------------------------*/
/* CM Info Functions        */
/*--------------------------*/
void WF_CMInfoGetFSMStats(tWFCMInfoFSMStats *p_info);
#endif


/*******************************************************************************
  Function:	
    uint16_t MRF24W_GetTrafficEvents(uint16_t* pEventInfo);

  Summary:
    Returns the current traffic events from the MRF24W

  Description:

  Precondition:
  	TCPIP stack should be initialized.

  Parameters:
    pEventInfo -- address to store additional information about the traffic event.
                  This is not applicable to all events.

  Returns:
  	The traffic event that occurred.
  	
  Remarks:
  	None.
  *****************************************************************************/
uint16_t MRF24W_GetTrafficEvents(uint16_t* pEventInfo);


/*******************************************************************************
  Function:	
    uint16_t MRF24W_GetMgmtEvents(uint16_t* pEventInfo);

  Summary:
    Returns the current management events from the MRF24W

  Description:

  Precondition:
  	TCPIP stack should be initialized.

  Parameters:
    pEventInfo -- address to store additional information about the management event.
                  This is not applicable to all events.

  Returns:
  	The management event that occurred.
  	
  Remarks:
  	None.
  *****************************************************************************/
uint16_t MRF24W_GetMgmtEvents(uint16_t* pEventInfo);



void MRF24W_ChipReset(void);

#if defined(WF_CM_DEBUG)
/*--------------------------*/
/* CM Info Functions        */
/*--------------------------*/
void WF_CMInfoGetFSMStats(tWFCMInfoFSMStats *p_info);
#endif

#endif /* __WF_API_H_ */



