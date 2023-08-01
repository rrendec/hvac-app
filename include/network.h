#ifndef __HVAC_NETWORK_H__
#define __HVAC_NETWORK_H__

#define NET_CONNECT_TIMEOUT_S	10
#define NET_CONNECT_RETRY_S	20
#define NET_SOCKET_TIMEOUT_S	20

int net_connect_loop(const char *host, const char *port);
int net_write(int sock, const void *buf, size_t len);
ssize_t __net_read(int sock, void *buf, size_t len, int nl);

static inline ssize_t net_read_line(int sock, void *buf, size_t len)
{
	return __net_read(sock, buf, len, 1);
}

static inline int net_read(int sock, void *buf, size_t len)
{
	return __net_read(sock, buf, len, 0);
}

#endif
