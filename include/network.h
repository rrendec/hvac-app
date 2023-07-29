#ifndef __HVAC_NETWORK_H__
#define __HVAC_NETWORK_H__

#define NET_CONNECT_TIMEOUT_S	10
#define NET_CONNECT_RETRY_S	20
#define NET_SOCKET_TIMEOUT_S	20

int net_connect_loop(const char *host, const char *port);

#endif
