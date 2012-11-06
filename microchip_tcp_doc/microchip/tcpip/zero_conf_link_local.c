/*******************************************************************************
  Zero Configuration (Zeroconf) IPV4 Link Local Addressing
  Module for Microchip TCP/IP Stack

  Summary:
   
  Description:
*******************************************************************************/

/*******************************************************************************
FileName:   ZeroconfLinkLocal.c
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

#define __Zeroconf__Link_Local_C

#include "tcpip_private.h"


#if defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)
#include "tcpip/zeroconflinklocal.h"

// access to low level ARP definitions
#include "arp_private.h"

extern void DisplayIPValue(IP_ADDR IPVal);

/* Constant to cross-check whether
 * DHCP Clinet is configured to use */
#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
const uint8_t g_zcll_dhcp_client = 1;
#else
const uint8_t g_zcll_dhcp_client = 0;
#endif

/* constants from RFC2937, section 9 */
#define PROBE_WAIT           1 /*second  (initial random delay)              */
#define PROBE_MIN            1 /*second  (minimum delay till repeated probe) */
#define PROBE_MAX            2 /*seconds (maximum delay till repeated probe) */
#define PROBE_NUM            3 /*         (number of probe packets)          */
#define ANNOUNCE_WAIT        2 /*seconds  (delay before announcing)          */
#define ANNOUNCE_NUM         2 /*        (number of announcement packets)    */
#define ANNOUNCE_INTERVAL    2 /*seconds (time between announcement packets) */
#define MAX_CONFLICTS       10 /*        (max conflicts before rate limiting)*/
#define RATE_LIMIT_INTERVAL 60 /*seconds (delay between successive attempts) */
#define DEFEND_INTERVAL     10 /*seconds (min. wait between defensive ARPs)  */

/* compilation constants */
#define IPV4_LLBASE 0xa9fe0100 /* 169.254.1.0 */

/* ARP  States Enum */
typedef enum _ARPState{
	ARPProbe = 0,
	ARPClaim,
	ARPDefend
} ARP_STATE;

/* Link-Local States Enum */
typedef enum _ZeroconfLLState
{
	SM_INIT = 0,
	SM_INTF_NOT_PRESENT,
	SM_INTF_NOT_CONNECTED,
	SM_DHCP_PRESENT,
	SM_ADDR_INIT,
	SM_ADDR_PROBE,
	SM_ADDR_CLAIM,
	SM_ADDR_DEFEND,	
	SM_ADDR_RELEASE,
} ZCLL_STATE;

/* Required for Tracking DHCP events */
typedef enum __zcll_dhcp_substate {
	ZCLL_DHCP_INIT = 0,
	ZCLL_DHCP_PHASE_1,
	ZCLL_DHCP_PHASE_2,
}ZCLL_DHCP_SUBSTATE;

/* ARP-Packet Types defined in RFC 2937 */
typedef enum arp_pkt_type {
	ARP_REQUEST_TYPE = 0,
	ARP_PROBE_TYPE,
	ARP_CLAIM_TYPE,
	ARP_DEFEND_TYPE,
	ARP_RESPONSE_TYPE,
	UNKNOWN_TYPE,
}ARP_PKT_TYPE;

/* Flags for the ZCLL State Machine */
typedef union _ZCLL_FLAGS
{
    struct
    {
        unsigned char bIsIntfPresent : 1;		// Whether or not Interface is present
        unsigned char bIsDHCPPresent : 1;		// Whether or not DHCP Client is enabled & 
												// DHCP server is detected
		unsigned char bIsConnected : 1;			// Indicates if WiFi interface is connected
		unsigned char probe_conflict :1 ;		// Conflict When selecting IP-addr
		unsigned char late_conflict : 1;		// Conflict When using IP-addr
    } bits;
    uint8_t Val;
} ZCLL_FLAGS;

/**************** Global Declarations ***************/
ZCLL_STATE zcll_state = SM_INIT;
ZCLL_DHCP_SUBSTATE zcll_dhcp_substate =ZCLL_DHCP_INIT;
static ZCLL_FLAGS zcll_flags = {{0x00}};

static IP_ADDR temp_IP_addr;	// Temporary IP address before checking uniquness
#ifdef TCPIP_STACK_CLIENT_MODE
static MAC_ADDR temp_MAC_addr;  // Temporary MAC address
#endif
static uint8_t probe_count = 0;
static uint8_t conflict_count = 0; 
static uint8_t announce_count = 0;

static bool bDefaultIPTried = false;

static int8_t arp_reg_id;

/***************** Forward Declarations **************/
void ZeroconfARPPktNotify (uint32_t SenderIPAddr, uint32_t TargetIPAddr, 
                           MAC_ADDR* SenderMACAddr, 
                           MAC_ADDR* TargetMACAddr, uint8_t op_req);
static struct arp_app_callbacks callbacks = 
{
	.ARPPkt_notify  = ZeroconfARPPktNotify,
};

//rex#if 0
//rextZGVoidReturn 
//rexZeroconfIndicate(tZGU8 type, tZGDataPtr fourByteHeader, tZGDataPtr pBuf, tZGU16 len)
//rex{
//rex    tZGU16 status;
//rex    
//rex    switch(type)
//rex    {
//rex    	case kZGMgtIndDisconnect:
//rex        	/* A deauth indication occurs when the AP has announced
//rex             * to our chip that the connection is no longer valid.  It becomes
//rex             * the responsibility of the wifiManager to determine what to do
//rex             * in that situation */
//rex			INFO_ZCLL_PRINT("Zeroconf-Callback: Disconnect Event Notification \r\n");
//rex            zcll_flags.bits.bIsConnected = 0;   
//rex        break;
//rex        
//rex    case kZGMgtIndConnStatus:
//rex    
//rex        status = (fourByteHeader[0] << 8) | fourByteHeader[1];
//rex        if (status == kZGConnStatusFound)
//rex        {
//rex            //APPCXT.bConnLost = 0;
//rex            //APPCXT.bConnFound = 1;
//rex			zcll_flags.bits.bIsConnected = 1;
//rex        }
//rex        break;
//rex        
//rex    default:
//rex        /* error condition un-recognized type */
//rex        break;
//rex    }
//rex    
//rex}
//rex#endif

static SYS_TICK zcll_rand(void)
{
	return SYS_TICK_Get();
}

/***************************************************************
  Function:
	static void ZeroconfStateMachineReset(void)

  Summary:
	Resets Zeroconf's Link-Local state-machine
    
  Description:
	This function resets the state-machine of Link-Local module.
    This is invoked in the initalization and when a address-conflict
    is detected.   

  Parameters:
	None 

  Returns:
  	None
  ***************************************************************/
#if 0
#define ZeroconfStateMachineReset() \
{probe_count = zcll_flags.bits.late_conflict = zcll_flags.bits.probe_conflict =0;}
#endif

static void ZeroconfStateMachineReset(bool bResetProbeCount)
{
	if (bResetProbeCount)
	{
		probe_count = 0;
	}
	zcll_flags.bits.late_conflict = 0; 
	
	//conflict_count = 0;
	//zcll_flags.bits.probe_conflict = 0;
}


/***************************************************************
  Function:
	void ARPAction(IP_ADDR SrcIPAddr,IP_ADDR DestIPAddr ,uint8_t op_req,ARP_STATE ARPAction)

  Summary:
     a).ARPProbe:
	  Sends out the ARP-probe packet.
     b).ARPClaim:
	  Sends out the ARP-Claim packet.
     c).ARPDefend:
         Sends out the ARP-Defend packet, when a address-conflict is detected.
    
  Description:


    a).ARPProbe:
	This function is used to send out the ARP-Probe packet to check
    the uniquness of selected IP-address in private space(169.254.x.x)

    This function makes use of ARPSendPkt User-API exposed by ARP 
    module. 

    ARP-Probe Packet:
	  ARP-Request
	  sender IP address: 0.0.0.0
	  sender HW address: Self MAC address
	  target IP address: <probe addr> (Chosen IP-address 169.254.x.x)
	  target HW address: FF:FF:FF:FF:FF:FF
	  
    b).ARPClaim:
 	This function is used to send out the ARP-Claim packet to finalize
    the uniquness of selected IP-address in private space(169.254.x.x).
    This claim packet is final-step in decision making of selected IP-
    address.

    This function makes use of ARPSendPkt User-API exposed by ARP 
    module. 

    ARP-Probe Packet:
	  ARP-Request
	  sender IP address: <claim addr> (Chosen IP-address 169.254.x.x)
	  sender HW address: Self MAC address
	  target IP address: <claim addr> (Chosen IP-address 169.254.x.x)
	  target HW address: FF:FF:FF:FF:FF:FF
	  
    c).ARPDefend:
    	This function is used to send out the ARP-Defend packet to defend
    the selected IP-address. When a conflicting ARP-packet (Probe or
    Claim) is observed on local network ARP-defend packet will be sent
    out to announe its authority of owning IP-address.

    This function makes use of ARPSendPkt User-API exposed by ARP 
    module. 

    ARP-Probe Packet:
	  ARP-Response
	  sender IP address: <claim addr> (Chosen IP-address 169.254.x.x)
	  sender HW address: Self MAC address
	  target IP address: <claim addr> (Chosen IP-address 169.254.x.x)
	  target HW address: FF:FF:FF:FF:FF:FF

  Parameters:
	None 

  Returns:
  	None
  ***************************************************************/

void ARPAction(uint32_t SrcIPAddr,uint32_t DestIPAddr ,uint8_t op_req,ARP_STATE ARPAction)
{
	bool rc;

	rc = ARPSendPkt( SrcIPAddr, DestIPAddr,op_req);

#if defined ( INFO_ZCLL )
	if(rc == false)
	{
         switch (ARPAction)
         {
		 	case ARPProbe:
				INFO_ZCLL_PRINT("ARPProbe: Error in sending out ARP-Probe pkt \n");
			    break;
		 	case ARPClaim:
				INFO_ZCLL_PRINT("ARPClaim: Error in sending out ARP-Claim pkt \n");
			    break;
		 	case ARPDefend:
				INFO_ZCLL_PRINT("ARPDefend: Error in sending out ARP-Defend pkt \n");
			    break;
         }		
	}
#endif
}

/***************************************************************
  Function:
	ARP_PKT_TYPE FindARPPktType (uint32_t SrcIPAddr, uint32_t DestIPAddr, 
                                 uint8_t op_req)

  Summary:
	Finds the Type of ARP-Packet based on the Source IP-address,
    Destination IP-address and operation-request.
    
  Description:
	This function is used to find out the ARP-packet type. When ARP
    module passes up a ARP-packet to Zeroconf Link-Local module, it 
    parses contents and finds the packet-type (like ARP-Probe, ARP-
    Claim, ARP-Defend, or generic ARP-request/response)    

  Parameters:
	SrcIPAddr    - Source IP-Address
    DestIPAddr   - Destination IP-Address
    op_req       - Operation-Request (ARP-Request/Response)

  Returns:
  	ARP_PKT_TYPE - Type of ARP-Packet (Probe, Claim, Defend or 
                    generic ARP-request/response)
  ***************************************************************/
ARP_PKT_TYPE FindARPPktType (uint32_t SrcIPAddr, uint32_t DestIPAddr,uint8_t op_req)
{
	if(op_req == ARP_OPERATION_REQ)
	{
		if(SrcIPAddr == 0x0)
			return ARP_PROBE_TYPE;
		else if (SrcIPAddr == DestIPAddr)
			return ARP_CLAIM_TYPE;
		else 
			return ARP_REQUEST_TYPE;

	}

	else if(op_req == ARP_OPERATION_RESP)
	{
		if(SrcIPAddr == DestIPAddr)
			return ARP_DEFEND_TYPE;
		else
			return ARP_RESPONSE_TYPE;
	}
	
	else
		return UNKNOWN_TYPE;
}
	
/***************************************************************
  Function:
	void ZeroconfARPPktNotify (uint32_t SenderIPAddr, uint32_t TargetIPAddr, 
                           MAC_ADDR* SenderMACAddr, 
                           MAC_ADDR* TargetMACAddr, uint8_t op_req)

  Summary:
	Callback registered with ARP-Module. This gets invoked from ARP-
    module and runs in the same context.
    
  Description:
	This function is registered as a callback with ARP-module to get 
    notified about incoming Packet-events. Based on the type of packet
    received and Link-Local current state, appropriate action will be 
    taken. To find the type of ARP-Packet this function makes use of 
    FindARPPktType routine.

    Primary purpose of this function is to decipher the ARP-Packet rxed
    and check whether its leading to a conflict with the selected IP-
    address.

    Two types of conflicts are defined: Probe-Conflict and Late-Conflict
    If the current state of Link-Local is Probe/Claim and a conflict is
    detected its called "Probe-Conflict"
    If the current state of Link-Local is Defend-state and a conflict is
    detected its called "Late-Conflict"

  Parameters:
	SenderIPAddr    - Sender IP-Address
    TargetIPAddr    - Target IP-Address
    SenderMACAddr   - Sender MAC-Address
    TargetMACAddr   - Target MAC-Address
    op_req          - Operation-Request (ARP-Request/Response)
	
  Returns:
  	None
  ***************************************************************/	
void ZeroconfARPPktNotify (uint32_t SenderIPAddr, uint32_t TargetIPAddr, 
                           MAC_ADDR* SenderMACAddr, 
                           MAC_ADDR* TargetMACAddr, uint8_t op_req)
{
	ARP_PKT_TYPE pkt_type;

	pkt_type = FindARPPktType (SenderIPAddr, TargetIPAddr, op_req);

	if(pkt_type == UNKNOWN_TYPE)
		return; // Can't hit this

	switch (zcll_state)
	{
	case SM_ADDR_PROBE:
	case SM_ADDR_CLAIM:
		{
			switch(pkt_type)
			{
			case ARP_PROBE_TYPE:
			case ARP_CLAIM_TYPE:
			case ARP_DEFEND_TYPE:
				if(temp_IP_addr.Val == TargetIPAddr ) // Probe-Conflict
				{
					if(memcmp(SenderMACAddr, &NetConfig.MyMACAddr, 6)) 
					{
						DEBUG_ZCLL_PRINT("ARPPktNotify: Somebody has initiated " \
							"the procedure to use same Addr.\r\n");

						zcll_flags.bits.probe_conflict =1;
					}
				}
				break;

			case ARP_RESPONSE_TYPE:
				/* Some-body has been using probed addr
				* We need to choose different Address */
				if(temp_IP_addr.Val == SenderIPAddr) 
				{
					DEBUG_ZCLL_PRINT("ARPPktNotify: Some-body has responded for " \
						"probed IP-addr \r\n");

					zcll_flags.bits.probe_conflict =1;	
				}
				break;
			default:
				break;
			}
		}
		break;

	case SM_ADDR_DEFEND:
		{
			if(NetConfig.MyIPAddr.Val == SenderIPAddr)
			{
				if(memcmp(SenderMACAddr, &NetConfig.MyMACAddr, 6)) 
				{
					DEBUG_ZCLL_PRINT("ARPPktNotify: Some-one has started using" \
						" our IP-address \r\n");

					zcll_flags.bits.late_conflict = 1;
				}
			}
		}
		break;

	default:
		break; // Nothing to do in other states
	}

}

/***************************************************************
  Function:
	static void zcll_seed_random()

  Summary:
	Seeds the Random-Number generator with Last four bytes of MAC-
    address & current-time.
    
  Description:
	This function is used to seed the random-number generator. To get
    better uniquness in random-number, last four bytes of MAC-address 
    is used as seed. Though the MAC-address is unique, the first 3
    bytes represent the manufacturer, which will be same for all the 
    Network-Interface cards from same manufacturer.

    To add more randomness the current-time is also added as seed. The
    current-time alone can't be used as seed. Because there's a fair
    chance that all the devices in local-area network might be powered
    up at the same-time, then which leads to conflict.

    So combination of MAC-Address and current-time is good seed for a
    random-number generator.
        
  Parameters:
	None

  Returns:
  	None
  ***************************************************************/
#if 0
static void zcll_seed_random()
{
	TCPIP_UINT32_VAL temp;
	SYS_TICK tick = SYS_TICK_Get();
	temp.v[3] = NetConfig.MyMACAddr.v[2];
	temp.v[2] = NetConfig.MyMACAddr.v[3];
	temp.v[1] = NetConfig.MyMACAddr.v[4];
	temp.v[0] =	NetConfig.MyMACAddr.v[5];
	srand( temp.Val + tick ); 
}
#endif

/***************************************************************
  Function:
	void ZeroconfLLInitialize(void)

  Summary:
	Initialization routine for Zeroconf Link-Local state-machine.
    
  Description:
    This is initialization function for Zeroconf Link-Local and 
    invoked from initialization portion of Main-function. 

    This function registers with ARP-module to get notifications
    about the incoming packets. Checks whether the WiFi MAC is 
    connected to an Access-Point or not.

  Parameters:
	None

  Returns:
  	None
  ***************************************************************/
void ZeroconfLLInitialize(void)
{
	/* ARP is core module for IPv4 LL protocol.
	 * Register event callbacks with ARP, to get 
	 * notified. */
	arp_reg_id = ARPRegisterCallbacks(&callbacks);
	if(arp_reg_id <0)
	{
		WARN_ZCLL_PRINT("ZeroconfLLInitialize: ARP Callback registration Failed!!! \r\n");
		return;
	}
			
	/* Check for the presence of Interface 
     * Currently there's no way to check the
     * presence of interface. So always true. 
     * Later it needs to replaced by actual
     * condition */
	if(0)
	{
		zcll_state = SM_INTF_NOT_PRESENT;
		return;
	}

	if(!MACIsLinked())
	{
		zcll_state = SM_INTF_NOT_CONNECTED;
		return;
	}
	
	/* check if DHCP-Client is enabled & able to detect DHCP-Server */
	if(g_zcll_dhcp_client)
	{
		zcll_state = SM_DHCP_PRESENT;
		zcll_dhcp_substate = ZCLL_DHCP_INIT;
		return;
		
	}
	
	zcll_state = SM_ADDR_INIT;
	/* setup random number generator
	* we key this off the MAC-48 HW identifier
	* the first 3 octets are the manufacturer
	* the next 3 the serial number
	* we'll use the last four for the largest variety
	*/
	//zcll_seed_random();
			
	return;
}

/***************************************************************
  Function:
	void ZeroconfLLProcess(void)

  Summary:
	Initialization routine for Zeroconf Link-Local state-machine.
    
  Description:
    This is Polled from Main-Application & Designed to support 
    co-operative multi-tasking. This needs to retrun to Main, 
    if we have to wait for Longer durations.

    This is the main function for Zeroconf's Link-Local and takes 
    the actions accoding to current-state and event-notifications
    from ARP-Module.

  Parameters:
	None

  Returns:
  	None
  ***************************************************************/


void ZeroconfLLProcess(void)
{
	static SYS_TICK event_time; // Internal Timer, to keep track of events
	static unsigned char time_recorded; // Flag to indicate event_time is loaded
	static unsigned char defended; // Flag to indicate, whether or not defended earlier
	static SYS_TICK random_delay;

	if(!MACIsLinked())
	{
		zcll_state = SM_INTF_NOT_CONNECTED;
	}

	switch(zcll_state)
	{			
	case SM_INIT:
		WARN_ZCLL_PRINT("SM_INIT: Wrong state \r\n");
		break;

	case SM_INTF_NOT_PRESENT:
		DEBUG_ZCLL_PRINT("SM_INTF_NOT_PRESENT: Intf not detected \r\n");
		break;

	case SM_INTF_NOT_CONNECTED:

		//DEBUG_ZCLL_MESG(zeroconf_dbg_msg,"SM_INTF_NOT_CONNECTED \r\n");
		//DEBUG_ZCLL_PRINT((char*)zeroconf_dbg_msg);

		if(!MACIsLinked())
		{
			//INFO_ZCLL_PRINT((char *)("MAC is not connected yet \r\n"));
			return;
		}
		else
		{
			/* Interface is connected now */
			zcll_state = SM_DHCP_PRESENT;
			zcll_dhcp_substate = ZCLL_DHCP_INIT;
			time_recorded = 0;
			event_time = 0;
			defended = 0;
		}

	case SM_DHCP_PRESENT:
		{
			//DEBUG_ZCLL_PRINT("SM_INTF_DHCP_PRESENT \r\n");

			if(!g_zcll_dhcp_client)
			{
				zcll_state = SM_ADDR_INIT;
			}
#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
			else
			{

				switch(zcll_dhcp_substate)
				{
				case ZCLL_DHCP_INIT:

					DEBUG_ZCLL_PRINT("ZCLL_DHCP_INIT: Entered \r\n");
					DEBUG_ZCLL_MESG((char *) zeroconf_dbg_msg,"SYS_TICK_TicksPerSecondGet() = %ld \r\n", 
						( (SYS_TICK_TicksPerSecondGet())));
					DEBUG_ZCLL_PRINT((char *) zeroconf_dbg_msg);
					if(!NetConfig.Flags.bIsDHCPEnabled)
					{
						DEBUG_ZCLL_PRINT("ZCLL_DHCP_INIT: Enabling DHCP client \r\n");
						DHCPEnable((uint8_t) 0); //DHCPEnable();
						time_recorded = 0; 	
					}	
					/* Start a Fisrt-phase Timer with 1 min Timeout
					* to allow DHCP-client to IP-address DHCP-ser */

					NetConfig.MyIPAddr.Val = 0;

					event_time = SYS_TICK_Get();
					time_recorded = 1;
					random_delay = (ZEROCONF_LINK_LOCAL_DHCP_TIMEOUT * SYS_TICK_TicksPerSecondGet() );
					zcll_dhcp_substate = ZCLL_DHCP_PHASE_1;
					INFO_ZCLL_PRINT("Waiting for DHCP-Client to Get IP-addr from" \
						" DHCP-server..... \r\n");

					return;

				case ZCLL_DHCP_PHASE_1:

					if(time_recorded) 
					{
						if(SYS_TICK_Get() - event_time < random_delay)
						{
							//if(DHCPFlags.bits.bDHCPServerDetected && 
							//   DHCPFlags.bits.bIsBound)
							if (DHCPIsServerDetected((uint8_t) 0) &&
								DHCPIsBound((uint8_t) 0))
							{
								INFO_ZCLL_PRINT("DHCP IP-address received. No Zeroconf " \
									"Link-Local required !! \r\n");
								INFO_ZCLL_MESG(zeroconf_dbg_msg,"DHCP IP-Addr: " \
									"%d.%d.%d.%d \r\n",
									NetConfig.MyIPAddr.v[0], 
									NetConfig.MyIPAddr.v[1],
									NetConfig.MyIPAddr.v[2], 
									NetConfig.MyIPAddr.v[3]);
								INFO_ZCLL_PRINT((char *)zeroconf_dbg_msg);
								DisplayIPValue(NetConfig.MyIPAddr); // LCD Disaply

								time_recorded = 0; // Cancel Timer
								zcll_dhcp_substate = ZCLL_DHCP_PHASE_2;
								return;

							}
							return;
						}
						// First Time out. Disable DHCP & Use Link-Local
						time_recorded = 0; // Reset Timer Flag
						event_time = 0;  // Cancel Timer

						break;
					}
					else
					{
						DEBUG_ZCLL_PRINT("ERROR !! Can't Enter this DHCP " \
							"Substate without timer \r\n");
						break;
					}

				case ZCLL_DHCP_PHASE_2:

					/* Able to get an IP-address from DHCP,
					* constantly moniotor for Validity. If
					* found invalid, move back to phase-1. */

					//if(	DHCPFlags.bits.bIsBound)
					if (DHCPIsBound((uint8_t) 0)) 
					{
						return;
					}
					else
					{
						if(!NetConfig.Flags.bIsDHCPEnabled)
						{
							/* Somebody else had disabled DHCP client.
							* Goto Link-Local addressing */
							DEBUG_ZCLL_PRINT("ZCLL_DHCP_PHASE_2: Externel module " \
								"disabled DHCP-Client \r\n");
							break;
						}

						temp_IP_addr.Val = 0x0;
						time_recorded = 0;
						zcll_dhcp_substate = ZCLL_DHCP_INIT;
						return;							
					}

				default:

					DEBUG_ZCLL_PRINT("ERROR !! Invalid DHCP Substate \r\n");
					break;
				}
			}

			// No break. Fall through

			// The following is commented out, to implement the behavior described in
			// RFC 3927, section 2.11.
			// In particular, IPv4LL should not "stop the DHCP client from attempting
			// to acquire a new IP address".

			#if defined(DISREGARD_RFC3927_SECTION_2_11)
			/* Disable DHCP Client, as it's not able to get an IP */
			INFO_ZCLL_PRINT("Disabling DHCP-Client. Going to use Link-Local \r\n");
			DHCPDisable((uint8_t) 0);
			#endif

#else
			INFO_ZCLL_PRINT("DHCP-Client is not present. Going to use Link-Local \r\n");
#endif
			zcll_state = SM_ADDR_INIT;
			/* Not yet seeded in init routine */
			/* setup random number generator
			* we key this off the MAC-48 HW identifier
			* the first 3 octets are the manufacturer
			* the next 3 the serial number
			* we'll use the last four for the largest variety
			*/
			//zcll_seed_random();
			DEBUG0_ZCLL_MESG(zeroconf_dbg_msg,"SM_DHCP_PRESENT --> SM_ADDR_INIT \r\n");
			DEBUG0_ZCLL_PRINT((char*)zeroconf_dbg_msg);

			// No break. Fall through
		}

	case SM_ADDR_INIT:

		ZeroconfStateMachineReset(false);
		conflict_count = 0;
		NetConfig.MyIPAddr.Val = 0x0;
		DisplayIPValue(NetConfig.MyIPAddr); // LCD Display

#ifdef TCPIP_STACK_CLIENT_MODE
		ARPInit();
#endif
		probe_count = 0;

		zcll_state = SM_ADDR_PROBE;
		INFO_ZCLL_PRINT("ADDR_INIT --> ADDR_PROBE \r\n");

		// No break. Fall through

	case SM_ADDR_PROBE:

		//DEBUG_ZCLL_PRINT("SM_ADDR_PROBE \r\n");

		switch ( zgzc_wait_for(&random_delay, &event_time, &time_recorded) )
		{
		case ZGZC_STARTED_WAITING:

			if (probe_count == 0)
			{
				// First probe. Wait for [0 ~ PROBE_WAIT] seconds before sending the probe.

				//random_delay = (((zcll_rand()) % (PROBE_WAIT) * SYS_TICK_TicksPerSecondGet()));
				//if (random_delay == 0) random_delay = SYS_TICK_TicksPerSecondGet();

				random_delay = (zcll_rand() % (PROBE_WAIT * SYS_TICK_TicksPerSecondGet()));

				DEBUG0_ZCLL_MESG(zeroconf_dbg_msg,"PROBE_WAIT Random Delay [%d]: %ld secs \r\n",
					probe_count,
					random_delay);
			}
			else if (probe_count < PROBE_NUM)
			{
				// Subsequent probes. Wait for [PROBE_MIN ~ PROBE_MAX] seconds before sending the probe.

				// random_delay = ( (((zcll_rand() % (PROBE_MAX-PROBE_MIN+1))+PROBE_MIN) * SYS_TICK_TicksPerSecondGet()) + (SYS_TICK_TicksPerSecondGet()>>3) );
				// Added a little bit more delay to pass conformance test.
				
				random_delay = ( (zcll_rand() % ((PROBE_MAX-PROBE_MIN) * SYS_TICK_TicksPerSecondGet()) ) +
					(PROBE_MIN * SYS_TICK_TicksPerSecondGet()) );

				DEBUG0_ZCLL_MESG(zeroconf_dbg_msg,"PROBE Random Delay [%d]: %ld ticks \r\n",
					probe_count,
					random_delay);
			}
			else
			{
				// Completed PROBE_NUM of probes. Now wait for ANNOUNCE_WAIT seconds to determine if
				// we can claim it.

				random_delay = (ANNOUNCE_WAIT * SYS_TICK_TicksPerSecondGet());
				DEBUG0_ZCLL_MESG(zeroconf_dbg_msg,"ANNOUNCE_WAIT delay [%d]: %ld ticks\r\n",
					probe_count,
					random_delay /*SYS_TICK_TicksPerSecondGet() */);
			}

			DEBUG0_ZCLL_PRINT((char*)zeroconf_dbg_msg);

			// Intentional fall-through

		case ZGZC_KEEP_WAITING:

			// Not Completed the delay proposed
			return;
		}

		// Completed the delay required

		DEBUG0_ZCLL_MESG(zeroconf_dbg_msg,"   delay: %ld ticks " \
			"completed \r\n", random_delay);
		DEBUG0_ZCLL_PRINT((char *)zeroconf_dbg_msg);

		if(zcll_flags.bits.probe_conflict)
		{
			/* Conflict with selected address */
			INFO_ZCLL_PRINT("Probe Conflict-1 Detected. Need to select diff addr \r\n");
			ZeroconfStateMachineReset(false);
			temp_IP_addr.Val = 0x0;

			conflict_count++;
			NetConfig.MyIPAddr.Val = 0x0;
		}
#ifdef TCPIP_STACK_CLIENT_MODE
		else if((conflict_count == 0) && temp_IP_addr.Val && (ARPIsResolved(&temp_IP_addr, &temp_MAC_addr)) ) 
		{
			if(!memcmp (&temp_MAC_addr, &NetConfig.MyMACAddr, 6) )
			{
				DEBUG0_ZCLL_PRINT("SM_ADDR_PROBE: Resolved with our address only. " \
					"Rare Case !!!! \r\n");
			}
			else
			{
				/* Conflict with selected address */
				INFO_ZCLL_PRINT("Probe Conflict-2 Detected. Need to select diff addr \r\n");
				ZeroconfStateMachineReset(false);
				temp_IP_addr.Val = 0x0;

				conflict_count++;
				NetConfig.MyIPAddr.Val = 0x0;
			}
		}
#endif

		if ((zcll_flags.bits.probe_conflict == 1) ||
			(!bDefaultIPTried))
		{
			/*
			* Pick random IP address in IPv4 link-local range
			* 169.254.1.0/16 is the allowed address range however 
			* 169.254.0.0/24 and 169.254.255.0/24 must be excluded, 
			* which removes 512 address from our 65535 candidates. 
			* That leaves us with 65023 (0xfdff) 
			*/

			// Need to start/restart the probe procedure.
			probe_count = 0;

			if ((!bDefaultIPTried) &&
				(NetConfig.DefaultIPAddr.v[0] == 169) && 
				(NetConfig.DefaultIPAddr.v[1] == 254) &&
				(NetConfig.DefaultIPAddr.v[2] != 0) &&
				(NetConfig.DefaultIPAddr.v[2] != 255))
			{
				// First probe, and the default IP is a valid IPv4LL address.
				// Use it.

				temp_IP_addr.Val = swapl(NetConfig.DefaultIPAddr.Val);
				bDefaultIPTried = true;
			}
			else
			{
				temp_IP_addr.Val = (IPV4_LLBASE | ((abs(zcll_rand()) % 0xfdff) ));
			}                

			INFO_ZCLL_MESG(zeroconf_dbg_msg,"Picked IP-Addr [%d]: %d.%d.%d.%d \r\n",
				probe_count,
				temp_IP_addr.v[3],temp_IP_addr.v[2],
				temp_IP_addr.v[1],temp_IP_addr.v[0]);
			INFO_ZCLL_PRINT((char *)zeroconf_dbg_msg);

			temp_IP_addr.Val = swapl((uint32_t) temp_IP_addr.Val);
		}

		// ToDo: check the max probe limit and probing rate
		if((zcll_flags.bits.probe_conflict == 1) || (probe_count < PROBE_NUM))
		{

			zcll_flags.bits.probe_conflict = 0;
			ARPAction(NetConfig.MyIPAddr.Val, temp_IP_addr.Val, ARP_OPERATION_REQ, ARPProbe);
			probe_count++;

			DEBUG0_ZCLL_MESG(zeroconf_dbg_msg, "Sending ARP [%d]\r\n", probe_count);
			DEBUG0_ZCLL_PRINT((char *)zeroconf_dbg_msg);

			break;
		}

		// No conflict detected ...

		if(probe_count >= PROBE_NUM) 
		{		
			zcll_state = SM_ADDR_CLAIM;
			announce_count = 0;

			INFO_ZCLL_PRINT("ADDR_PROBE --> ADDR_CLAIM \r\n");

			return;
		}

		break;

	case SM_ADDR_CLAIM:

		switch ( zgzc_wait_for(&random_delay, &event_time, &time_recorded) )
		{
		case ZGZC_STARTED_WAITING:
			if (announce_count == 0)
			{
				// First announcement is immediate. We have passed the ANNOUNCE_WAIT in
				// PROBE state already.

				random_delay = 0;
			}
			else 
			{
				// Subsequent announcements need to wait ANNOUNCE_INTERVAL seconds
				// before sending the announcement.

				random_delay = (ANNOUNCE_INTERVAL * SYS_TICK_TicksPerSecondGet());
			}
			// Intentional fall-through

		case ZGZC_KEEP_WAITING:

			// Not Completed the delay proposed
			return;
		}

		// Completed the delay required

		DEBUG0_ZCLL_MESG(zeroconf_dbg_msg,"ANNOUNCE delay: %ld ticks completed \r\n", random_delay);
		DEBUG0_ZCLL_PRINT((char *)zeroconf_dbg_msg);

		if ( announce_count < ANNOUNCE_NUM )
		{
			ARPAction(temp_IP_addr.Val,temp_IP_addr.Val, ARP_OPERATION_REQ, ARPClaim);
			announce_count++;

			DEBUG0_ZCLL_MESG(zeroconf_dbg_msg, "Sending ANNOUNCEMENT [%d]\r\n", announce_count);
			DEBUG0_ZCLL_PRINT((char *)zeroconf_dbg_msg);
		}
		else 
		{
			// Claim it. Goto DEFEND state

			NetConfig.MyIPAddr = temp_IP_addr;
			NetConfig.MyMask.Val = 0x0000FFFF;
			zcll_state = SM_ADDR_DEFEND;
			DisplayIPValue(NetConfig.MyIPAddr);
			INFO_ZCLL_MESG(zeroconf_dbg_msg,"\r\n******** Taken IP-Addr: " \
				"%d.%d.%d.%d ******** \r\n",
				NetConfig.MyIPAddr.v[0], NetConfig.MyIPAddr.v[1], 
				NetConfig.MyIPAddr.v[2],NetConfig.MyIPAddr.v[3]);
			INFO_ZCLL_PRINT((char *)zeroconf_dbg_msg);
			INFO_ZCLL_PRINT("ADDR_CLAIM --> ADDR_DEFEND \r\n");
		}

		break;

	case SM_ADDR_DEFEND:

		//DEBUG_ZCLL_PRINT("SM_ADDR_DEFEND \r\n");

		if( zcll_flags.bits.late_conflict)
		{
			if (!defended)
			{
				zcll_flags.bits.late_conflict = 0;
				INFO_ZCLL_PRINT("CONFLICT DETECTED !!! \r\n");

				INFO_ZCLL_PRINT("Defending the Self Address once \r\n");
				ARPAction(NetConfig.MyIPAddr.Val, NetConfig.MyIPAddr.Val, ARP_OPERATION_RESP, ARPDefend); 

				defended = 1;
			}
			else
			{
				// We are not allowed to defend another conflict during an active defended period

				INFO_ZCLL_PRINT("Releasing the IP-Address because of multiple Conflicts \r\n");

				zcll_state = SM_ADDR_RELEASE;

				defended = 0;
				event_time = 0;
				random_delay = 0;

				INFO_ZCLL_PRINT("ADDR_DEFEND --> ADDR_RELEASE \r\n");
				break;
			}
		}

		if (defended)
		{
			switch ( zgzc_wait_for(&random_delay, &event_time, &time_recorded) )
			{
			case ZGZC_STARTED_WAITING:

				random_delay = (DEFEND_INTERVAL * SYS_TICK_TicksPerSecondGet());
				DEBUG0_ZCLL_MESG(zeroconf_dbg_msg,"DEFEND_INTERVAL Delay : %ld ticks\r\n",
					random_delay/*SYS_TICK_TicksPerSecondGet() */);
				DEBUG0_ZCLL_PRINT((char *)zeroconf_dbg_msg);

				// Intentional fall-through

			case ZGZC_KEEP_WAITING:

				// Not Completed the delay proposed
				return;
			}

			// Completed the delay required

			DEBUG0_ZCLL_MESG(zeroconf_dbg_msg,"ANNOUNCE delay: %ld ticks " \
				"completed \r\n", random_delay);
			DEBUG0_ZCLL_PRINT((char *)zeroconf_dbg_msg);

			defended = 0;
		}

		break;

	case SM_ADDR_RELEASE:

		INFO_ZCLL_PRINT("ADDR_RELEASE --> ADDR_INIT\r\n");

		NetConfig.MyIPAddr.Val = 0x00;
		
		// Need New Addr
		temp_IP_addr.Val = (IPV4_LLBASE | ((abs(zcll_rand()) % 0xfdff) ));
		temp_IP_addr.Val = swapl((uint32_t) temp_IP_addr.Val);

		zcll_state = SM_ADDR_INIT;
		time_recorded = 0;
		defended = 0;
		event_time = 0;
		break;

	default:
		break;	
	}
}		
#endif //#if defined(TCPIP_STACK_USE_ZERCONF_LINK_LOCAL)



