/*******************************************************************************
  Main Application Entry Point and TCP/IP Stack Demo

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Demonstrates how to call and use the Microchip TCP/IP stack
    -Reference: Microchip TCP/IP Stack Help (tcpip Help.chm)
 *******************************************************************************/

/*******************************************************************************
FileName:  MainDemo.c 
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

// Include all headers for any enabled tcpip functions
#include "tcpip/tcpip.h"


// Include functions specific to this stack application
#include "main_demo.h"
#if defined(APP_USE_IPERF)
#include "iperf_console.h"
#endif

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

int stackNotifyCnt = 0;
const void* stackNotifyHandle;
TCPIP_EVENT stackTotEvents = 0;

#if defined(TCPIP_IF_MRF24W)
uint16_t mrf_txEvents = 0, mrf_txEventInfo = 0;
uint16_t mrf_mgmtEvents = 0, mrf_mgmtEventInfo = 0;
int mrf_EventCnt = 0;
#endif
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)


// Local functions prototypes    
#if defined(TCPIP_STACK_USE_TELNET_SERVER)
static const char* ProcessIO(void);
#endif  // defined(TCPIP_STACK_USE_TELNET_SERVER)

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
static void StackNotification(const void* hIf, TCPIP_EVENT event, void* nParam);
static void ProcessNotification(const void* hIf);
#if defined(TCPIP_IF_MRF24W)
extern void WF_ProcessEvent(uint16_t event, uint16_t eventInfo);
#endif
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

void ICMPv6Callback(uint8_t type, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, void * header);




//
// Main application entry point.
//

int main(void)
{

#if defined(APP_USE_IPERF)
    static uint8_t iperfOk = 0;
#endif
    static SYS_TICK startTick = 0;
    static IP_ADDR dwLastIP[sizeof (TCPIP_HOSTS_CONFIGURATION) / sizeof (*TCPIP_HOSTS_CONFIGURATION)];
    uint8_t i;


    // perform system initialization
    if(!SYS_Initialize())
    {
        return 0;
    }
    
    SYS_CONSOLE_MESSAGE("\r\n\n\n --- Unified TCPIP Demo Starts! --- \r\n");

    SYS_OUT_MESSAGE("TCPStack " TCPIP_STACK_VERSION "  ""                ");


    
#if defined(TCPIP_STACK_USE_MPFS) || defined(TCPIP_STACK_USE_MPFS2)
    MPFSInit();
#endif

    // Initiates board setup process if button is depressed 
    // on startup
    if (BUTTON0_IO == 0u)
    {

#if defined(TCPIP_STACK_USE_STORAGE)  && (defined(SPIFLASH_CS_TRIS) || defined(EEPROM_CS_TRIS))
        // Invalidate the EEPROM contents if BUTTON0 is held down for more than 4 seconds
        SYS_TICK StartTime = SYS_TICK_Get();
        LED_PUT(0x00);

        while (BUTTON0_IO == 0u)
        {
            if (SYS_TICK_Get() - StartTime > 4 * SYS_TICK_TicksPerSecondGet())
            {
                TCPIP_STORAGE_HANDLE hStorage;

                // just in case we execute this before the stack is initialized
                TCPIP_STORAGE_Init(0);
                hStorage = TCPIP_STORAGE_Open(0, false); // no refresh actually needed
                if (hStorage)
                {
                    TCPIP_STORAGE_Erase(hStorage);
                    SYS_CONSOLE_MESSAGE("\r\n\r\nBUTTON0 held for more than 4 seconds.  Default settings restored.\r\n\r\n");
                    TCPIP_STORAGE_Close(hStorage);
                }
                else
                {
                    SYS_ERROR(SYS_ERROR_WARN, "\r\n\r\nCould not restore the default settings!!!.\r\n\r\n");
                }
                TCPIP_STORAGE_DeInit(0);

                LED_PUT(0x0F);
                // wait 4.5 seconds here then reset
                while ((SYS_TICK_Get() - StartTime) <= (9 * SYS_TICK_TicksPerSecondGet() / 2));
                LED_PUT(0x00);
                while (BUTTON0_IO == 0u);
                SYS_Reboot();
                break;
            }
        }
#endif  // defined(TCPIP_STACK_USE_STORAGE)  && (defined(SPIFLASH_CS_TRIS) || defined(EEPROM_CS_TRIS))
    }

    // Initialize the TCPIP stack
    if (!TCPIP_STACK_Init(TCPIP_HOSTS_CONFIGURATION, sizeof (TCPIP_HOSTS_CONFIGURATION) / sizeof (*TCPIP_HOSTS_CONFIGURATION),
            TCPIP_STACK_MODULE_CONFIG_TBL, sizeof (TCPIP_STACK_MODULE_CONFIG_TBL) / sizeof (*TCPIP_STACK_MODULE_CONFIG_TBL)))
    {
        return 0;
    }
    
#if defined(TCPIP_STACK_USE_TELNET_SERVER)
    TelnetRegisterCallback(ProcessIO);
#endif  // defined(TCPIP_STACK_USE_TELNET_SERVER)

#if defined (TCPIP_STACK_USE_IPV6)
    TCPIP_ICMPV6_RegisterCallback(ICMPv6Callback);
#endif

#if defined(TCPIP_STACK_USE_ICMP_CLIENT) || defined(TCPIP_STACK_USE_ICMP_SERVER)
    ICMPRegisterCallback(PingProcessIPv4);
#endif


#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    TCPIP_NET_HANDLE hWiFi = TCPIP_STACK_NetHandle("MRF24W");
    if (hWiFi)
    {
        TCPIP_STACK_SetNotifyEvents(hWiFi, TCPIP_EV_RX_ALL | TCPIP_EV_TX_ALL | TCPIP_EV_RXTX_ERRORS);
        TCPIP_STACK_SetNotifyHandler(hWiFi, StackNotification, 0);
    }
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)


#if defined(APP_USE_IPERF)
    IperfConsoleInit();
    iperfOk = IperfAppInit(TCPIP_HOSTS_CONFIGURATION[0].interface);
#endif

    // Now that all items are initialized, begin the co-operative
    // multitasking loop.  This infinite loop will continuously
    // execute all stack-related tasks, as well as your own
    // application's functions.  Custom functions should be added
    // at the end of this loop.
    // Note that this is a "co-operative mult-tasking" mechanism
    // where every task performs its tasks (whether all in one shot
    // or part of it) and returns so that other tasks can do their
    // job.
    // If a task needs very long time to do its job, it must be broken
    // down into smaller pieces so that other tasks can have CPU time.
    while (1)
    {
        // Blink LED0 (right most one) every second.
        if (SYS_TICK_Get() - startTick >= SYS_TICK_TicksPerSecondGet() / 2ul)
        {
            startTick = SYS_TICK_Get();
            LED0_IO ^= 1;
        }

        // This task performs normal stack task including checking
        // for incoming packet, type of packet and calling
        // appropriate stack entity to process it.
        TCPIP_STACK_Task();

        // Process application specific tasks here.
        // For this demo app, this will include the Generic TCP
        // client and servers, and the SNMP, Ping, and SNMP Trap
        // demos.  Following that, we will process any IO from
        // the inputs on the board itself.
        // Any custom modules or processing you need to do should
        // go here.
#if defined(TCPIP_STACK_USE_GENERIC_TCP_CLIENT_EXAMPLE)
        GenericTCPClient();
#endif

#if defined(TCPIP_STACK_USE_GENERIC_TCP_SERVER_EXAMPLE)
        GenericTCPServer();
#endif

#if defined(TCPIP_STACK_USE_SMTP_CLIENT)
        SMTPDemo();
#endif

#if defined(TCPIP_STACK_USE_ICMP_CLIENT) || defined (TCPIP_STACK_USE_ICMP_SERVER) || defined (TCPIP_STACK_USE_IPV6)
        // use ping on the default interface
        PingDemoTask();
#endif

#if defined(TCPIP_STACK_USE_SNMP_SERVER) && !defined(SNMP_TRAP_DISABLED)
        //User should use one of the following SNMP demo
        // This routine demonstrates V1 or V2 trap formats with one variable binding.
        SNMPTrapDemo();

#if defined(SNMP_STACK_USE_V2_TRAP) || defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)
        //This routine provides V2 format notifications with multiple (3) variable bindings
        //User should modify this routine to send v2 trap format notifications with the required varbinds.
        //SNMPV2TrapDemo();
#endif 
        if (gSendTrapFlag)
            SNMPSendTrap();
#endif

#if defined(TCPIP_STACK_USE_BERKELEY_API)
        BerkeleyTCPClientDemo();
        BerkeleyTCPServerDemo();
        BerkeleyUDPClientDemo(0);
#endif

#if defined(APP_USE_IPERF)
        IperfConsoleProcess();
        if (iperfOk) IperfAppCall(); // Only running in case of init succeed
        IperfConsoleProcessEpilogue();
#endif

        // If the local IP address has changed (ex: due to DHCP lease change)
        // write the new IP address to the console display, UART, and Announce
        // service
        // We use the default interface
        for (i = 0; i < sizeof (TCPIP_HOSTS_CONFIGURATION) / sizeof (*TCPIP_HOSTS_CONFIGURATION); i++)
        {
            TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandle(TCPIP_HOSTS_CONFIGURATION[i].interface);
            if ((uint32_t) dwLastIP[i].Val != TCPIP_STACK_NetAddress(netH))
            {
                dwLastIP[i].Val = TCPIP_STACK_NetAddress(netH);

                SYS_CONSOLE_MESSAGE(TCPIP_HOSTS_CONFIGURATION[i].interface);
                SYS_CONSOLE_MESSAGE(" new IP Address: ");

                DisplayIPValue(dwLastIP[i]);

                SYS_CONSOLE_MESSAGE("\r\n");

            }
        }

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
        if (stackNotifyCnt)
        {
            stackNotifyCnt = 0;
            ProcessNotification(stackNotifyHandle);
        }
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    }
}

// Writes an IP address to the LCD display and the UART as available

void DisplayIPValue(IP_ADDR IPVal)
{
    char message[16 + 1];

    //	printf("%u.%u.%u.%u", IPVal.v[0], IPVal.v[1], IPVal.v[2], IPVal.v[3]);
#if defined (__dsPIC33E__) || defined (__PIC24E__)
    static uint8_t IPDigit[4]; /* Needs to be declared as static to avoid the array getting optimized by C30 v3.30 compiler for dsPIC33E/PIC24E.
				  Otherwise the LCD displays corrupted IP address on Explorer 16. To be fixed in the future compiler release*/
#else
    uint8_t IPDigit[4];
#endif
    uint8_t i;

    strcpy(message, "");
    for (i = 0; i < sizeof (IP_ADDR); i++)
    {
        uitoa((uint16_t) IPVal.v[i], IPDigit);
        strcat(message, (char *) IPDigit);
        if (i < sizeof (IP_ADDR) - 1)
        {
            strcat(message, ".");
        }
    }
    SYS_OUT_MESSAGE_LINE(message, 1);
    SYS_CONSOLE_MESSAGE((char *) message);
}

// Processes A/D data from the potentiometer

#if defined(TCPIP_STACK_USE_TELNET_SERVER)
static const char* ProcessIO(void)
{
    static uint8_t AN0String[8];
    // Convert potentiometer result into ASCII string
    uitoa((uint16_t) ADC1BUF0, AN0String);
    return (const char*)AN0String;
}
#endif  // defined(TCPIP_STACK_USE_TELNET_SERVER)



#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

static void StackNotification(const void* hIf, TCPIP_EVENT event, void* nParam)
{
    stackNotifyCnt++;
    stackNotifyHandle = hIf;
    stackTotEvents |= event;
}

static void ProcessNotification(const void* hIf)
{
#if defined(TCPIP_IF_MRF24W)

    mrf_txEvents = MRF24W_GetTrafficEvents(&mrf_txEventInfo);
    mrf_mgmtEvents = MRF24W_GetMgmtEvents(&mrf_mgmtEventInfo);
    mrf_EventCnt++;
    if (mrf_mgmtEvents)
    {
        WF_ProcessEvent(mrf_mgmtEvents, mrf_mgmtEventInfo);
    }
#endif
}
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)


#if defined (TCPIP_STACK_USE_IPV6)

void ICMPv6Callback(uint8_t type, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, void * header)
{
    switch (((ICMPV6_HEADER_ERROR *) header)->vType)
    {
        case ICMPV6_ERROR_DEST_UNREACHABLE:

            break;
        case ICMPV6_ERROR_PACKET_TOO_BIG:

            break;
        case ICMPV6_ERROR_TIME_EXCEEDED:

            break;
        case ICMPV6_ERROR_PARAMETER_PROBLEM:

            break;
        case ICMPV6_INFO_ECHO_REQUEST:

            break;
        case ICMPV6_INFO_ECHO_REPLY:
            PingProcessIPv6((ICMPV6_HEADER_ECHO *) header, remoteIP, localIP);
            break;
    }
}
#endif
