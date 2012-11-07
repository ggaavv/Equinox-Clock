
/*
Copyright (c) 2010 Donatien Garnier (donatiengar [at] gmail [dot] com)
 
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
 
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
 
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

/**
@file
ZG2100 Low-level driver definitions
*/
//Donatien Garnier 2010

#ifndef ZG_DEFS_H
#define ZG_DEFS_H

/* Parameters */

#define ZG_HEAD_BUF_SIZE 64//16
#define ZG_FIFO_BUF_SIZE 2048

#define ZG_MAX_TX_PENDING 7

#define ZG_SCAN_PROBE_DELAY     20 //In us
#define ZG_SCAN_MIN_CHAN_TIME   400 //In (1024 us)
#define ZG_SCAN_MAX_CHAN_TIME   800 //In (1024 us)

#define ZG_REGION               ZG_REGION_EUROPE

/* Interface-specific constants */

#define ZG_SSID_LEN             32 //Max SSID length
#define ZG_MAX_SUPPORTED_RATES  8 //Max number of supported rates
#define ZG_SCAN_MAX_CHANNELS    14
#define ZG_MACADDR_LEN          6
#define ZG_SNAP_LEN             6

#define ZG_REGION_EUROPE   2 //Channels 1 - 13
#define ZG_REGION_MEA      2 //Channels 1 - 13
#define ZG_REGION_AMERICAS 0 //Channels 1 - 11
#define ZG_REGION_DEFAULT  1 //Channels 1 - 11
#define ZG_REGION_ASIAPAC  2 //Channels 1 - 13
#define ZG_REGION_JAPAN_A  5 //Channel 14 only

#define ZG_WEP_KEYS_COUNT  4
#define ZG_WEP_KEY_LEN     13
#define ZG_WPA_PASS_LEN    64
#define ZG_PMK_LEN         32

/* Commands */

#define ZG_CMD_REG     0x00
#define ZG_CMD_REG_RD  (ZG_CMD_REG | 0x40)
#define ZG_CMD_REG_WR  (ZG_CMD_REG | 0x00)

#define ZG_CMD_FIFO         0x80

#define ZG_CMD_FIFO_RD_DATA (ZG_CMD_FIFO | 0x00)
#define ZG_CMD_FIFO_RD_MGMT (ZG_CMD_FIFO | 0x00)
#define ZG_CMD_FIFO_RD_DONE (ZG_CMD_FIFO | 0x50)

#define ZG_CMD_FIFO_WR_DATA (ZG_CMD_FIFO | 0x20)
#define ZG_CMD_FIFO_WR_MGMT (ZG_CMD_FIFO | 0x30)
#define ZG_CMD_FIFO_WR_DONE (ZG_CMD_FIFO | 0x40)

/* Registers */

#define ZG_REG_INTF            0x01 //8-bit register, interrupt raised flags
#define ZG_REG_INTE            0x02 //8-bit register, interrupt enable flags

#define ZG_REG_INTF2           0x2d //16-bit register, 2nd level interrupt raised flags
#define ZG_REG_INTE2           0x2e //16-bit register, 2nd level interrupt enable flags

#define ZG_REG_SYSINFO         0x21 //8-bit register, read sys info data window
#define ZG_REG_SYSINFO_INDEX   0x2b //16-bit register, sys info data window offset
#define ZG_REG_SYSINFO_STATUS  0x2c //16-bit register, sys info data window status

#define ZG_REG_F0_CTRL0        0x25 //16-bit register, FIFO0 Ctrl reg
#define ZG_REG_F0_CTRL1        0x26 //16-bit register, FIFO0 Ctrl reg
#define ZG_REG_F0_INDEX        0x27 //16-bit register, FIFO0 Index reg
#define ZG_REG_F0_STATUS       0x28 //16-bit register, FIFO0 Status reg

#define ZG_REG_F1_CTRL0        0x29 //16-bit register, FIFO1 Ctrl reg
#define ZG_REG_F1_CTRL1        0x2a //16-bit register, FIFO1 Ctrl reg
#define ZG_REG_F1_INDEX        0x2b //16-bit register, FIFO1 Index reg
#define ZG_REG_F1_STATUS       0x2c //16-bit register, FIFO1 Status reg

#define ZG_REG_F0_ROOM         0x2f //16-bit register, room in FIFO0 for writing

#define ZG_REG_F0_LEN          0x33 //16-bit register, bytes ready to read on FIFO0
#define ZG_REG_F1_LEN          0x35 //16-bit register, bytes ready to read on FIFO1

#define ZG_REG_LOWPWR          0x3d //16-bit register, low power mode cfg
#define ZG_REG_IREG_ADDR       0x3e //16-bit register, set/read the ptr to indexed regs
#define ZG_REG_IREG_DATA       0x3f //16-bit register, index regs data

#define ZG_REG_LEN( reg )      ((reg)>=0x25?2:1) //Return bytes len of a register

/* Indexed Registers */

#define ZG_IREG_HW_STATUS      0x2a //16-bit register, hardware status
#define ZG_IREG_HW_RST         0x2b //16-bit register, reset reg

#define ZG_HW_STATUS_RESET     0x1000 //ZG_IREG_HW_STATUS Mask to determine wether the chip is in reset

#define ZG_IREG_PWR_STATUS     0x3e //16-bit register, power status (sleep state)

#define ZG_IREG_LEN( reg )     2 //Bytes len of an indexed register

/* FIFOs */

/*
When using FIFO requests, frame is formatted like this:
[ZG_CMD_FIFO_WR_****][Type][Subtype]
*/

//Just here for zg_fifo_* cmds
#define ZG_FIFO_DATA 0
#define ZG_FIFO_MGMT 1
#define ZG_FIFO_ANY  0 //A specific FIFO does not have to be specified for reads

/* Types */

#define ZG_FIFO_WR_TXD_REQ      1 //Req to write TX Data  

#define ZG_FIFO_RD_TXD_ACK      1 //TX Data ack'ed

#define ZG_FIFO_RD_RXD_AVL      3 //RX Data available

#define ZG_FIFO_WR_MGMT_REQ     2 //Req to read/write mgmt Data  

#define ZG_FIFO_RD_MGMT_AVL     2 //Mgmt Data available/written ok
#define ZG_FIFO_RD_MGMT_EVT     4 //Mgmt Data event

/* Subtypes */

//TXD, RXD
#define ZG_FIFO_TXD_STD         1
#define ZG_FIFO_RXD_STD         1

//Management

//Events
#define ZG_FIFO_MGMT_DISASSOC   1 //Disassociated
#define ZG_FIFO_MGMT_DEAUTH     2 //Deauthenticated
#define ZG_FIFO_MGMT_CONN       4 //Connection state

//Commands
#define ZG_FIFO_MGMT_SCAN       1 //Network scan

#define ZG_FIFO_MGMT_PSK_CALC   12 //Compute Pre-Shared Key

#define ZG_FIFO_MGMT_PMK_KEY    8  //Set Pairwise Master Key
#define ZG_FIFO_MGMT_WEP_KEY    10 //Set WEP key

#define ZG_FIFO_MGMT_PARM_SET   15 //Set param
#define ZG_FIFO_MGMT_PARM_GET   16 //Get param

#define ZG_FIFO_MGMT_ADHOC      18 //Start an adhoc connection
#define ZG_FIFO_MGMT_CONNECT    19
#define ZG_FIFO_MGMT_DISC       5

#define ZG_FIFO_MGMT_CONN_MGMT  20 //Manage connection

/* Params IDs for Get/Set Params */

#define ZG_FIFO_MGMT_PARM_MACAD 1  //MAC Addr (6 bytes long)
#define ZG_FIFO_MGMT_PARM_REGION 2 //Region (1 byte long)
#define ZG_FIFO_MGMT_PARM_SYSV  26 //System version (2 bytes long)

/* Masks used by ZG_REG_INT* */

#define ZG_INT_MASK_F0 0x40
#define ZG_INT_MASK_F1 0x80

/* Internal Error Codes */

typedef enum __ZG_INT_ERR 
{
  ZG_INT_OK = 1,
  ZG_INT_RESOURCES = 12, //Not enough resources
  ZG_INT_TIMEOUT,
  ZG_INT_FRAME_ERROR,
  ZG_INT_AUTH_REFUSED,
  ZG_INT_ASSOC_REFUSED,
  ZG_INT_IN_PROGRESS,
  ZG_INT_SUPPLICANT_FAILED = 21

} ZG_INT_ERR;

/* F0 / F1 helpers */
#define ZG_REG_F_CTRL0(n)   ((n==0)?ZG_REG_F0_CTRL0:ZG_REG_F1_CTRL0)
#define ZG_REG_F_CTRL1(n)   ((n==0)?ZG_REG_F0_CTRL1:ZG_REG_F1_CTRL1)
#define ZG_REG_F_INDEX(n)   ((n==0)?ZG_REG_F0_INDEX:ZG_REG_F1_INDEX)
#define ZG_REG_F_STATUS(n)  ((n==0)?ZG_REG_F0_STATUS:ZG_REG_F1_STATUS)

#define ZG_REG_F_LEN(n)     ((n==0)?ZG_REG_F0_LEN:ZG_REG_F1_LEN)

#define ZG_INT_MASK_F(n)    ((n==0)?ZG_INT_MASK_F0:ZG_INT_MASK_F1)

/* Macro helpers (LE Platform, SPI is BE) */

#define HTODS( x ) ( (((x)<<8) | ((x)>>8)) & 0xFFFF ) //Host to device, short
#define DTOHS( x ) ( HTODS(x) ) //Device to host, short

#define HTODL( x ) ( ( ((x) & 0x000000FF) << 24 ) \
                   | ( ((x) & 0x0000FF00) << 8  ) \
                   | ( ((x) & 0x00FF0000) >> 8  ) \
                   | ( ((x) & 0xFF000000) >> 24 ) ) //Host to device, long
#define DTOHL( x ) ( HTODL(x) )

/* Platform Specific defs */

#define ZG_MEM __attribute((section("AHBSRAM1")))

/* Typedefs */

typedef unsigned char  byte;
typedef unsigned short word;
typedef unsigned int   dword;

typedef signed   char  int8_t;
typedef unsigned char  uint8_t;

typedef signed   short int16_t;
typedef unsigned short uint16_t;

typedef signed   int  int32_t;
typedef unsigned int  uint32_t;

/* Data structures */

typedef __packed struct __ZG_SYSV
{
  byte rom;
  byte revision;
} ZG_SYSV;

///BSS types
typedef enum __ZG_BSS_TYPE
{
  ZG_BSS_INFRA = 1, ///< Infrastructure
  ZG_BSS_ADHOC = 2, ///< Ad-Hoc
  ZG_BSS_ANY = 3 ///< Either
} ZG_BSS_TYPE;

typedef enum __ZG_PROBE_TYPE
{
  ZG_PROBE_ACTIVE = 1,
  ZG_PROBE_PASSIVE = 2
} ZG_PROBE_TYPE;

//Scan request
typedef __packed struct __ZG_SCAN_REQ
{
  word probe_delay; //In us
  word min_chan_time; //Min scan time on each channel in (1024 us)
  word max_chan_time; //Max scan time on each channel in (1024 us)
  byte bssid_mask[ZG_MACADDR_LEN];
  ZG_BSS_TYPE bss_type; //Infra, adhoc or both
  ZG_PROBE_TYPE probe_req; //Send probe request frames
  byte ssid_len;
  byte channels_count; //Number
  char ssid[ZG_SSID_LEN]; //Scan for a specific network, 0 otherwise
  byte channels[ZG_SCAN_MAX_CHANNELS]; //Channels to scan
} ZG_SCAN_REQ;

//Scan results header
typedef __packed struct __ZG_SCAN_RES
{
  byte result;
  byte state;
  byte last_channel; //Last channel scanned
  byte results_count; //Number of ZG_SCAN_ITEM following this header
} ZG_SCAN_RES;

//Scan result
//See: 
// - http://en.wikipedia.org/wiki/Beacon_frame
// - IEEE 802.11-2007 7.2.3.1 Beacon frame format (p. 80 ~ 81)
// - IEEE 802.11-2007 7.3 Management frame body components  (p. 87 ~ 139)
typedef __packed struct __ZG_SCAN_ITEM
{
  byte bssid[ZG_MACADDR_LEN]; //This is actually a MAC address
  char ssid[ZG_SSID_LEN]; //SSID of the network
  word capability;
  word beacon_period;
  word atim_wdw; //http://en.wikipedia.org/wiki/Announcement_Traffic_Indication_Message
  byte supported_rates[ZG_MAX_SUPPORTED_RATES];//(0x80 | enc_rate) where enc_rate = (rate / 500Kbps) 
  byte rssi;
  byte supported_rates_count;
  byte dtim_period; //http://en.wikipedia.org/wiki/Delivery_Traffic_Indication_Message
  ZG_BSS_TYPE bss_type; //Infra or adhoc
  byte channel;
  byte ssid_len;
} ZG_SCAN_ITEM;

//WEP Key setup
typedef __packed struct __ZG_WEP_REQ
{
  byte slot;
  byte key_size;
  byte default_key;
  byte ssid_len;
  char ssid[ZG_SSID_LEN]; //SSID of the network
  byte keys[ZG_WEP_KEYS_COUNT][ZG_WEP_KEY_LEN];
} ZG_WEP_REQ;

//PSK Key computation
typedef __packed struct __ZG_PSK_CALC_REQ
{
  byte config;
  byte phrase_len;
  byte ssid_len;
  byte _pad; //Padding
  char ssid[ZG_SSID_LEN]; //SSID of the network
  char pass_phrase[ZG_WPA_PASS_LEN];
} ZG_PSK_CALC_REQ;

typedef __packed struct __ZG_PSK_CALC_RES
{
  byte result;
  byte state;
  byte key_returned;
  byte _pad; //Padding
  byte key[ZG_PMK_LEN]; //PSK key returned
} ZG_PSK_CALC_RES;

typedef __packed struct __ZG_PMK_REQ
{
  byte slot;
  byte ssid_len;
  char ssid[ZG_SSID_LEN]; //SSID of the network
  byte key[ZG_PMK_LEN]; //PSK key returned
} ZG_PMK_REQ;

///Security type
typedef enum __ZG_SECURITY
{
  ZG_SECURITY_NONE = 0x00, ///< None
  ZG_SECURITY_WEP  = 0x01, ///< WEP
  ZG_SECURITY_WPA  = 0x02, ///< WPA
  ZG_SECURITY_WPA2 = 0x03, ///< WPA2
  ZG_SECURITY_TRY  = 0xFF ///< Try all (not recommanded)
} ZG_SECURITY;

typedef __packed struct __ZG_CONNECT_REQ
{
  ZG_SECURITY security;
  byte ssid_len;
  char ssid[ZG_SSID_LEN]; //SSID of the network
  word sleep_duration; //Power save sleep duration (/100 ms)
  ZG_BSS_TYPE bss_type;
  byte _pad; //Padding
} ZG_CONNECT_REQ;

typedef __packed struct __ZG_DISCONNECT_REQ
{
  word reason_code;
  bool disconnect;
  bool deauth_frame;
} ZG_DISCONNECT_REQ;

//Ethernet / ZG Packet headers
typedef __packed struct __ZG_ETH_HDR //Ethernet packet header
{
  byte dest[ZG_MACADDR_LEN];
  byte src[ZG_MACADDR_LEN];
  word type;
} ZG_ETH_HDR;

typedef __packed struct __ZG_RX_HDR //ZG packet header on rx
{
  word rssi;
  byte dest[ZG_MACADDR_LEN];
  byte src[ZG_MACADDR_LEN];
  dword arrival_time;
  word data_len;
  byte snap[ZG_SNAP_LEN]; //SNAP word, see zg_net
  word type; //Ethernet type
} ZG_RX_HDR;

typedef __packed struct __ZG_TX_HDR //ZG packet header on tx
{
  word zero; //Must be set to zero
  byte dest[ZG_MACADDR_LEN];
  byte snap[ZG_SNAP_LEN]; //SNAP word, see zg_net
  word type; //Ethernet type
} ZG_TX_HDR;

#endif

