#ifndef __IPERFAPP_H__
#define __IPERFAPP_H__

#include <assert.h>

/* these are the ports for this application */

#define IPERF_APP_SERVER_PORT (5001)
#define kIperfAppID (1100)

#define IPERF_GET_MSEC_TICK_COUNT() SYS_TICK_Get()

#define IPERF_ASSERT(expr) assert(expr)

extern bool IperfAppInit(const char* interface);
extern void IperfAppCall(void);

#endif /* __IPERFAPP_H__ */
