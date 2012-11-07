
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

/** \file
ZG2100 Wi-Fi Module network interface header file
*/

#ifndef ZG2100NETIF_H
#define ZG2100NETIF_H

struct netif;

#include "mbed.h"

#include "if/lwip/LwipNetIf.h"
#include "core/net.h"

#include "drv/zg2100/zg_defs.h"
#include "drv/zg2100/zg_err.h"

///ZG2100 Wi-Fi Module network interface error codes
typedef zg_err ZG2100Err;

///ZG2100 Wi-Fi Module network interface
/**
This class provides Wi-Fi connectivity to the stack
*/
class ZG2100NetIf : public LwipNetIf
{
public:
  ///Instantiates the Interface and register it against the stack, DHCP will be used
  ZG2100NetIf( PinName mosi, PinName miso, PinName sclk, PinName cs, PinName intp, PinName nrst ); //W/ DHCP
  ///Instantiates the Interface and register it against the stack, DHCP will not be used
  /**
  IpAddr is a container class that can be constructed with either 4 bytes or no parameters for a null IP address.
  */
  ZG2100NetIf( PinName mosi, PinName miso, PinName sclk, PinName cs, PinName intp, PinName nrst,
               IpAddr ip, IpAddr netmask, IpAddr gateway, IpAddr dns ); //W/o DHCP
  virtual ~ZG2100NetIf();
  
  ///Initializes module
  void init();
  
  ///Sets network's SSID
  void setSsid(const char* ssid);

  ///Sets network's WEP key
  void setWepKey(const byte* key, int len);

  ///Sets network's WPA passphrase (can last up to 30s)
  void setWpaPass(const char* pass);

  ///Sets network's PSK key (preferred when using WPA to avoid computing it each time)
  void setPskKey(const byte* key, int len);

  ///Connects to network
  ZG2100Err connect(ZG_BSS_TYPE type, ZG_SECURITY security);

  ///Disconnects from network
  void disconnect();

  ///Brings the interface up (must be connected to a network first)
  /**
  Uses DHCP if necessary
  @param timeout_ms : You can set the timeout parameter in milliseconds, if not it defaults to 15s
  @return : ZG_OK on success or ZG_TIMEOUT on timeout
  */  
  ZG2100Err setup(int timeout_ms = 15000);

  virtual void poll();

private:
  void waitReady();

  SPI m_spi;
  DigitalOut m_cs;
  /*InterruptIn*/ DigitalIn m_intp;
  DigitalOut m_nrst;

  Timer m_ethArpTimer;
  Timer m_dhcpCoarseTimer;
  Timer m_dhcpFineTimer;
  Timer m_igmpTimer;
    
  bool m_useDhcp;

  netif* m_pNetIf;
  
  IpAddr m_netmask;
  IpAddr m_gateway;
  
  const char* m_hostname;
  
};

#endif

