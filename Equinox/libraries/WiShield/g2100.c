
/*****************************************************************************

Filename:		g2100.c
Description:	Driver for the ZeroG Wireless G2100 series devices

 *****************************************************************************

 Driver for the WiShield 1.0 wireless devices

 Copyright(c) 2009 Async Labs Inc. All rights reserved.

 This program is free software; you can redistribute it and/or modify it
 under the terms of version 2 of the GNU General Public License as
 published by the Free Software Foundation.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 more details.

 You should have received a copy of the GNU General Public License along with
 this program; if not, write to the Free Software Foundation, Inc., 59
 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

 Contact Information:
 <asynclabs@asynclabs.com>

 Author               Date        Comment
 ----------------------------------------------------------------------------
 AsyncLabs			02/25/2009	Initial port
 AsyncLabs			05/29/2009	Adding support for new library

 *****************************************************************************/

//#include <string.h>
#include "config.h"
#include "g2100.h"
#include "global-conf.h"
#include "debug_frmwrk.h"

#include "wifi_config.h"
#include "pinout.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_exti.h"

//#include "libmaple.h"
//#include "spi.h"
//#include "gpio.h"
//#include "nvic.h"

//#define ZG2100_SPI_DBG 0
//#define ZG2100_DBG 0

//#if ZG2100_SPI_DBG
//#define zg2100_spi_dbg(...) iprintf(__VA_ARGS__)
//#else
//#define zg2100_spi_dbg(...)
//#endif

static U8 mac[6];

static U8 hdr[5];
static volatile U8 zg_conn_status;
static volatile U8 intr_occured;
static volatile U8 intr_valid;
static volatile U8 zg_drv_state;
static volatile U8 tx_ready;
static volatile U8 rx_ready;
static volatile U8 cnf_pending;
static U8* zg_buf;
static volatile U16 zg_buf_len;

static U8 wpa_psk_key[32];

/* D10  */
//#define CS_PORT                         GPIOA_BASE
//#define CS_PIN                          4

/* D9  */
//#define LED_PORT                        GPIOB_BASE
//#define LED_PIN                         7
//#define LEDConn_on()                    gpio_write_bit(LED_PORT, LED_PIN, 1);
//#define LEDConn_off()                   gpio_write_bit(LED_PORT, LED_PIN, 0);

/* D4  */
//#define DEBUG_PORT                      GPIOB_BASE
//#define DEBUG_PIN                       5


void zg_init()
{
   intr_occured = 0;
   intr_valid = 0;
   zg_drv_state = DRV_STATE_INIT;
   zg_conn_status = 0;
   tx_ready = 0;
   rx_ready = 0;
   cnf_pending = 0;
   zg_buf = uip_buf;
   zg_buf_len = UIP_BUFSIZE;

   zg_chip_reset();
   _DBG("[OK]-zg_chip_reset()");_DBD(__LINE__);_DBG_(__FILE__);
   zg_interrupt2_reg();
   _DBG("[OK]-zg_interrupt2_reg()");_DBD(__LINE__);_DBG_(__FILE__);
   zg_interrupt_reg(0xff, 0);
   _DBG("[OK]-zg_interrupt_reg(0xff, 0)");_DBD(__LINE__);_DBG_(__FILE__);
   zg_interrupt_reg(0x80|0x40, 1);
   _DBG("zg_interrupt_reg(0x80|0x40, 1)");_DBD(__LINE__);_DBG_(__FILE__);

   ssid_len = (U8)strlen(ssid);
   security_passphrase_len = (U8)strlen(security_passphrase);
}

void spi_transfer(volatile U8* buf, U16 len, U8 toggle_cs)
{

   U8 i;

   FIO_ClearValue(WF_CS_PORT, WF_CS_BIT);

   SSP_DATA_SETUP_Type xferConfig;
   xferConfig.tx_data = buf;
   xferConfig.rx_data = buf;
   xferConfig.length = len;


//   uart_writestr("\nTx");
//   for (i = 0; i < len; i++) {
//	   uart_send_32_Hex(buf[i]);
//	   uart_writestr(".");
//   }
//   uart_writestr("\n");

   SSP_ReadWrite (LPC_SSP0, &xferConfig, SSP_TRANSFER_POLLING);

//   uart_writestr("Rx");
//   for (i = 0; i < len; i++) {
//	   uart_send_32_Hex(buf[i]);
//	   uart_writestr(".");
//   }
//   uart_writestr("\n");

   if (toggle_cs) {
	   FIO_SetValue(WF_CS_PORT, WF_CS_BIT);
   }

   return;
}

void zg_chip_reset()
{
   U8 loop_cnt = 0;

   do {
      // write reset register addr
      hdr[0] = ZG_INDEX_ADDR_REG;
      hdr[1] = 0x00;
      hdr[2] = ZG_RESET_REG;

      spi_transfer(hdr, 3, 1);
      _DBG("[OK]-zg_chip_reset() -- 1");_DBD(__LINE__);_DBG_(__FILE__);

      hdr[0] = ZG_INDEX_DATA_REG;
      hdr[1] = (loop_cnt == 0)?(0x80):(0x0f);
      hdr[2] = 0xff;
      spi_transfer(hdr, 3, 1);
   } while(loop_cnt++ < 1);

   // write reset register data
   hdr[0] = ZG_INDEX_ADDR_REG;
   hdr[1] = 0x00;
   hdr[2] = ZG_RESET_STATUS_REG;
   spi_transfer(hdr, 3, 1);
   _DBG("[OK]-zg_chip_reset() -- 2");_DBD(__LINE__);_DBG_(__FILE__);

   do {
      hdr[0] = 0x40 | ZG_INDEX_DATA_REG;
      hdr[1] = 0x00;
      hdr[2] = 0x00;
      spi_transfer(hdr, 3, 1);
   } while((hdr[1] & ZG_RESET_MASK) == 0);
   _DBG("[OK]-zg_chip_reset() -- 3");_DBD(__LINE__);_DBG_(__FILE__);

   do {
      hdr[0] = 0x40 | ZG_BYTE_COUNT_REG;
      hdr[1] = 0x00;
      hdr[2] = 0x00;
      spi_transfer(hdr, 3, 1);
   } while((hdr[1] == 0) && (hdr[2] == 0));
}

void zg_interrupt2_reg()
{
   // read the interrupt2 mask register
   hdr[0] = 0x40 | ZG_INTR2_MASK_REG;
   hdr[1] = 0x00;
   hdr[2] = 0x00;
   spi_transfer(hdr, 3, 1);

   // modify the interrupt mask value and re-write the value to the interrupt
   // mask register clearing the interrupt register first
   hdr[0] = ZG_INTR2_REG;
   hdr[1] = 0xff;
   hdr[2] = 0xff;
   hdr[3] = 0;
   hdr[4] = 0;
   spi_transfer(hdr, 5, 1);

   return;
}

void zg_interrupt_reg(U8 mask, U8 state)
{
   // read the interrupt register
   hdr[0] = 0x40 | ZG_INTR_MASK_REG;
   hdr[1] = 0x00;
   spi_transfer(hdr, 2, 1);

   // now regBuf[0] contains the current setting for the
   // interrupt mask register
   // this is to clear any currently set interrupts of interest
   hdr[0] = ZG_INTR_REG;
   hdr[2] = (hdr[1] & ~mask) | ( (state == 0)? 0 : mask );
   hdr[1] = mask;
   spi_transfer(hdr, 3, 1);

   return;
}

//void zg_isr()
// void EINT0_IRQHandler (void) // TODO: set to eint0 when finnished debugging
void EINT2_IRQHandler (void)
{
	EXTI_ClearEXTIFlag(EXTI_EINT2);
	intr_occured = 1;
	_DBG("E2 ");//_DBD(__LINE__);_DBD_(__FILE__);
}

void EINT3_IRQHandler (void)
{
	EXTI_ClearEXTIFlag(EXTI_EINT3);
	intr_occured = 1;
	_DBG("E3 Interup xxxxxxxxxxxxx");//_DBD(__LINE__);_DBG_(__FILE__);
}

void zg_process_isr()
{
//	_DBG("E3 Interup servie routine zzzzzzzzzzzzzzzzzzzzzzz");//_DBD(__LINE__);_DBG_(__FILE__);

   U8 intr_state = 0;
   U8 next_cmd = 0;

   hdr[0] = 0x40 | ZG_INTR_REG;
   hdr[1] = 0x00;
   hdr[2] = 0x00;
   spi_transfer(hdr, 3, 1);

   intr_state = ZG_INTR_ST_RD_INTR_REG;

   do {
      switch(intr_state) {
      case ZG_INTR_ST_RD_INTR_REG:
      {
         U8 intr_val = hdr[1] & hdr[2];

         if ( (intr_val & ZG_INTR_MASK_FIFO1) == ZG_INTR_MASK_FIFO1) {
            hdr[0] = ZG_INTR_REG;
            hdr[1] = ZG_INTR_MASK_FIFO1;
            spi_transfer(hdr, 2, 1);

            intr_state = ZG_INTR_ST_WT_INTR_REG;
            next_cmd = ZG_BYTE_COUNT_FIFO1_REG;
         }
         else if ( (intr_val & ZG_INTR_MASK_FIFO0) == ZG_INTR_MASK_FIFO0) {
            hdr[0] = ZG_INTR_REG;
            hdr[1] = ZG_INTR_MASK_FIFO0;
            spi_transfer(hdr, 2, 1);

            intr_state = ZG_INTR_ST_WT_INTR_REG;
            next_cmd = ZG_BYTE_COUNT_FIFO0_REG;
         }
         else if (intr_val) {
            intr_state = 0;
         }
         else {
            intr_state = 0;
         }

         break;
      }
      case ZG_INTR_ST_WT_INTR_REG:
         hdr[0] = 0x40 | next_cmd;
         hdr[1] = 0x00;
         hdr[2] = 0x00;
         spi_transfer(hdr, 3, 1);

         intr_state = ZG_INTR_ST_RD_CTRL_REG;
         break;
      case ZG_INTR_ST_RD_CTRL_REG:
      {
            // Get the size of the incoming packet
         U16 rx_byte_cnt = (0x0000 | (hdr[1] << 8) | hdr[2]) & 0x0fff;

         // Check if our buffer is large enough for packet
         if(rx_byte_cnt + 1 < (U16)UIP_BUFSIZE ) {
            zg_buf[0] = ZG_CMD_RD_FIFO;
            // Copy ZG2100 buffer contents into zg_buf (uip_buf)
            spi_transfer(zg_buf, rx_byte_cnt + 1, 1);
            // interrupt from zg2100 was meaningful and requires further processing
            intr_valid = 1;
         }
         else {
            // Too Big, ignore it and continue
            intr_valid = 0;
         }

         // Tell ZG2100 we're done reading from its buffer
         hdr[0] = ZG_CMD_RD_FIFO_DONE;
         spi_transfer(hdr, 1, 1);

         // Done reading interrupt from ZG2100
         intr_state = 0;
         break;
      }
      }
   } while (intr_state);

   intr_occured = 0;
}

void zg_send(U8* buf, U16 len)
{
   hdr[0] = ZG_CMD_WT_FIFO_DATA;
   hdr[1] = ZG_MAC_TYPE_TXDATA_REQ;
   hdr[2] = ZG_MAC_SUBTYPE_TXDATA_REQ_STD;
   hdr[3] = 0x00;
   hdr[4] = 0x00;
   spi_transfer(hdr, 5, 0);

   buf[6] = 0xaa;
   buf[7] = 0xaa;
   buf[8] = 0x03;
   buf[9] = buf[10] = buf[11] = 0x00;
   spi_transfer(buf, len, 1);

   hdr[0] = ZG_CMD_WT_FIFO_DONE;
   spi_transfer(hdr, 1, 1);
}

void zg_recv(U8* buf, U16* len)
{
   zg_rx_data_ind_t* ptr = (zg_rx_data_ind_t*)&(zg_buf[3]);
   *len = ZGSTOHS( ptr->dataLen );

   memcpy(&zg_buf[0], &zg_buf[5], 6);
   memcpy(&zg_buf[6], &zg_buf[11], 6);
   memcpy(&zg_buf[12], &zg_buf[29], *len);

   *len += 12;
}

U16 zg_get_rx_status()
{
   if (rx_ready) {
      rx_ready = 0;
      return zg_buf_len;
   }
   else {
      return 0;
   }
}

void zg_clear_rx_status()
{
   rx_ready = 0;
}

void zg_set_tx_status(U8 status)
{
   tx_ready = status;
}

U8 zg_get_conn_state()
{
   return zg_conn_status;
}

void zg_set_buf(U8* buf, U16 buf_len)
{
   zg_buf = buf;
   zg_buf_len = buf_len;
}

U8* zg_get_mac()
{
   return mac;
}

void zg_write_wep_key(U8* cmd_buf)
{
   zg_wep_key_req_t* cmd = (zg_wep_key_req_t*)cmd_buf;

   cmd->slot = 3;                    // WEP key slot
   cmd->keyLen = 13;    // Key length: 5 bytes (64-bit WEP); 13 bytes (128-bit WEP)
   cmd->defID = 0; // Default key ID: Key 0, 1, 2, 3
   cmd->ssidLen = ssid_len;
   memset(cmd->ssid, 0x00, 32);
   memcpy(cmd->ssid, ssid, ssid_len);
   memcpy(cmd->key, wep_keys, ZG_MAX_ENCRYPTION_KEYS * ZG_MAX_ENCRYPTION_KEY_SIZE);

   return;
}

static void zg_calc_psk_key(U8* cmd_buf)
{
   zg_psk_calc_req_t* cmd = (zg_psk_calc_req_t*)cmd_buf;

   cmd->configBits = 0;
   cmd->phraseLen = security_passphrase_len;
   cmd->ssidLen = ssid_len;
   cmd->reserved = 0;
   memset(cmd->ssid, 0x00, 32);
   memcpy(cmd->ssid, ssid, ssid_len);
   memset(cmd->passPhrase, 0x00, 64);
   memcpy(cmd->passPhrase, security_passphrase, security_passphrase_len);

   return;
}

static void zg_write_psk_key(U8* cmd_buf)
{
   zg_pmk_key_req_t* cmd = (zg_pmk_key_req_t*)cmd_buf;

   cmd->slot = 0;       // WPA/WPA2 PSK slot
   cmd->ssidLen = ssid_len;
   memset(cmd->ssid, 0x00, 32);
   memcpy(cmd->ssid, ssid, cmd->ssidLen);
   memcpy(cmd->keyData, wpa_psk_key, ZG_MAX_PMK_LEN);

   return;
}

void zg_drv_process()
{
   // TX frame
   if (tx_ready && !cnf_pending) {
      zg_send(zg_buf, zg_buf_len);
      tx_ready = 0;
      cnf_pending = 1;
   }

//   _DBG("zg_drv_process() after // TX frame");_DBD(__LINE__);_DBG_(__FILE__);

   // process interrupt
   if (intr_occured) {
      zg_process_isr();
   }
   _DBG("zg_drv_process() after // process interrupt");_DBD_(__LINE__);_DBG_(__FILE__);

//   uart_writestr("\n");
//   uart_send_32_Hex(intr_valid);
//   uart_writestr("\n");
//   uart_send_32_Hex(zg_buf[1]);
//   uart_writestr("\n");
//   uart_send_32_Hex(zg_buf[2]);
//   uart_writestr("\n");

   if (intr_valid) {
      switch (zg_buf[1]) {
      case ZG_MAC_TYPE_TXDATA_CONFIRM:
         cnf_pending = 0;
         break;
      case ZG_MAC_TYPE_MGMT_CONFIRM:
         if (zg_buf[3] == ZG_RESULT_SUCCESS) {
            switch (zg_buf[2]) {
            case ZG_MAC_SUBTYPE_MGMT_REQ_GET_PARAM:
          	   uart_writestr("\ncase ZG_MAC_SUBTYPE_MGMT_REQ_GET_PARAM");
               mac[0] = zg_buf[7];
               mac[1] = zg_buf[8];
               mac[2] = zg_buf[9];
               mac[3] = zg_buf[10];
               mac[4] = zg_buf[11];
               mac[5] = zg_buf[12];
               zg_drv_state = DRV_STATE_SETUP_SECURITY;
               break;
            case ZG_MAC_SUBTYPE_MGMT_REQ_WEP_KEY:
         	   uart_writestr("\ncase ZG_MAC_SUBTYPE_MGMT_REQ_WEP_KEY");
               zg_drv_state = DRV_STATE_ENABLE_CONN_MANAGE;
               break;
            case ZG_MAC_SUBTYPE_MGMT_REQ_CALC_PSK:
               memcpy(wpa_psk_key, ((zg_psk_calc_cnf_t*)&zg_buf[3])->psk, 32);
               zg_drv_state = DRV_STATE_INSTALL_PSK;
               break;
            case ZG_MAC_SUBTYPE_MGMT_REQ_PMK_KEY:
               zg_drv_state = DRV_STATE_ENABLE_CONN_MANAGE;
               break;
            case ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT_MANAGE:
               zg_drv_state = DRV_STATE_START_CONN;
               break;
            case ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT:
 //              LEDConn_on();
               zg_conn_status = 1;      // connected
               break;
            default:
               break;
            }
         }
         break;
      case ZG_MAC_TYPE_RXDATA_INDICATE:
         zg_drv_state = DRV_STATE_PROCESS_RX;
         break;
      case ZG_MAC_TYPE_MGMT_INDICATE:
         switch (zg_buf[2]) {
         case ZG_MAC_SUBTYPE_MGMT_IND_DISASSOC:
         case ZG_MAC_SUBTYPE_MGMT_IND_DEAUTH:
//            LEDConn_off();
            zg_conn_status = 0; // lost connection

            //try to reconnect
            zg_drv_state = DRV_STATE_START_CONN;
            break;
         case ZG_MAC_SUBTYPE_MGMT_IND_CONN_STATUS:
            {
               U16 status = (((U16)(zg_buf[3]))<<8)|zg_buf[4];

               if (status == 1 || status == 5) {
 //                 LEDConn_off();
                  zg_init();
                  /* Block execution until reconnected  */
                  while(zg_get_conn_state() != 1) {
                      zg_drv_process();
                  }
               }
               else if (status == 2 || status == 6) {
 //                 LEDConn_on();
                  zg_conn_status = 1;   // connected
               }
            }
            break;
         }
         break;
      }

      intr_valid = 0;
   }


//   uart_writestr("111111\n");
 //  uart_send_32_Hex(zg_drv_state);
//   uart_writestr("\n222222\n");

   switch (zg_drv_state) {
   case DRV_STATE_INIT:
      zg_drv_state = DRV_STATE_GET_MAC;
      break;
   case DRV_STATE_GET_MAC:
      // get MAC address
	   uart_writestr("\n// get MAC address");
      zg_buf[0] = ZG_CMD_WT_FIFO_MGMT;
      zg_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
      zg_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_GET_PARAM;
      zg_buf[3] = 0;
      zg_buf[4] = ZG_PARAM_MAC_ADDRESS;
      spi_transfer(zg_buf, 5, 1);
      zg_buf[0] = ZG_CMD_WT_FIFO_DONE;
      spi_transfer(zg_buf, 1, 1);

      zg_drv_state = DRV_STATE_IDLE;
      break;
   case DRV_STATE_SETUP_SECURITY:
      switch (security_type) {
      case ZG_SECURITY_TYPE_NONE:
         zg_drv_state = DRV_STATE_ENABLE_CONN_MANAGE;
         break;
      case ZG_SECURITY_TYPE_WEP:
         // Install all four WEP keys on G2100
   	   uart_writestr("\n// Install all four WEP keys on G2100");
         zg_buf[0] = ZG_CMD_WT_FIFO_MGMT;
         zg_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
         zg_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_WEP_KEY;
         zg_write_wep_key(&zg_buf[3]);
         spi_transfer(zg_buf, ZG_WEP_KEY_REQ_SIZE+3, 1);

         zg_buf[0] = ZG_CMD_WT_FIFO_DONE;
         spi_transfer(zg_buf, 1, 1);

         zg_drv_state = DRV_STATE_IDLE;
         break;
      case ZG_SECURITY_TYPE_WPA:
      case ZG_SECURITY_TYPE_WPA2:
         // Initiate PSK calculation on G2100
      	   uart_writestr("\n// Initiate PSK calculation on G2100");
         zg_buf[0] = ZG_CMD_WT_FIFO_MGMT;
         zg_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
         zg_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_CALC_PSK;
         zg_calc_psk_key(&zg_buf[3]);
         spi_transfer(zg_buf, ZG_PSK_CALC_REQ_SIZE+3, 1);

         zg_buf[0] = ZG_CMD_WT_FIFO_DONE;
         spi_transfer(zg_buf, 1, 1);

         zg_drv_state = DRV_STATE_IDLE;
         break;
      default:
         break;
      }
      break;
   case DRV_STATE_INSTALL_PSK:
      // Install the PSK key on G2100
  	   uart_writestr("\n// Install the PSK key on G2100");
      zg_buf[0] = ZG_CMD_WT_FIFO_MGMT;
      zg_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
      zg_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_PMK_KEY;
      zg_write_psk_key(&zg_buf[3]);
      spi_transfer(zg_buf, ZG_PMK_KEY_REQ_SIZE+3, 1);

      zg_buf[0] = ZG_CMD_WT_FIFO_DONE;
      spi_transfer(zg_buf, 1, 1);

      zg_drv_state = DRV_STATE_IDLE;
      break;
   case DRV_STATE_ENABLE_CONN_MANAGE:
      // enable connection manager
  	   uart_writestr("\n// enable connection manager");
      zg_buf[0] = ZG_CMD_WT_FIFO_MGMT;
      zg_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
      zg_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT_MANAGE;
      zg_buf[3] = 0x01; // 0x01 - enable; 0x00 - disable
      zg_buf[4] = 10;           // num retries to reconnect
      zg_buf[5] = 0x10 | 0x02 | 0x01;   // 0x10 -       enable start and stop indication messages
                              //                        from G2100 during reconnection
                              // 0x02 - start reconnection on receiving a deauthentication
                              //                        message from the AP
                              // 0x01 - start reconnection when the missed beacon count
                              //                        exceeds the threshold. uses default value of
                              //                        100 missed beacons if not set during initialization
      zg_buf[6] = 0;
      spi_transfer(zg_buf, 7, 1);

      zg_buf[0] = ZG_CMD_WT_FIFO_DONE;
      spi_transfer(zg_buf, 1, 1);

      zg_drv_state = DRV_STATE_IDLE;
      break;
   case DRV_STATE_START_CONN:
   {
      zg_connect_req_t* cmd = (zg_connect_req_t*)&zg_buf[3];

      // start connection to AP
 	   uart_writestr("\n// start connection to AP");
      zg_buf[0] = ZG_CMD_WT_FIFO_MGMT;
      zg_buf[1] = ZG_MAC_TYPE_MGMT_REQ;
      zg_buf[2] = ZG_MAC_SUBTYPE_MGMT_REQ_CONNECT;

      cmd->secType = security_type;

      cmd->ssidLen = ssid_len;
      memset(cmd->ssid, 0, 32);
      memcpy(cmd->ssid, ssid, ssid_len);

      // units of 100 milliseconds
      cmd->sleepDuration = 0;

      if (wireless_mode == WIRELESS_MODE_INFRA)
         cmd->modeBss = 1;
      else if (wireless_mode == WIRELESS_MODE_ADHOC)
         cmd->modeBss = 2;

      spi_transfer(zg_buf, ZG_CONNECT_REQ_SIZE+3, 1);

      zg_buf[0] = ZG_CMD_WT_FIFO_DONE;
      spi_transfer(zg_buf, 1, 1);

      zg_drv_state = DRV_STATE_IDLE;
      break;
   }
   case DRV_STATE_PROCESS_RX:
      zg_recv(zg_buf, (U16*)&zg_buf_len);
      rx_ready = 1;

      zg_drv_state = DRV_STATE_IDLE;
      break;
   case DRV_STATE_IDLE:
      break;
   }
//   _DBG("[END]-zg_drv_process()");_DBD_(__LINE__);_DBG_(__FILE__);
}

// END
