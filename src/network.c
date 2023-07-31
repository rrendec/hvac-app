#include <poll.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

#include "common.h"
#include "network.h"

static int net_connect_one(const struct addrinfo *ap)
{
	int rc, sock, opt;
	char host[NI_MAXHOST];
	char serv[NI_MAXSERV];
	struct timeval timeout = {NET_CONNECT_TIMEOUT_S};

	rc = getnameinfo(ap->ai_addr, ap->ai_addrlen,
			 host, sizeof(host), serv, sizeof(serv),
			 NI_NUMERICHOST | NI_NUMERICSERV);
	if (rc) {
		snprintf(host, sizeof(host), "?");
		snprintf(serv, sizeof(serv), "<%d>", rc);
	}

	sock = socket(ap->ai_family, ap->ai_socktype, ap->ai_protocol);
	xassert(sock != -1, return -1, "%d", errno);

	/* Linux specific; see `man 7 socket` */
	rc = setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
	xassert(!rc, goto out_close, "%d", errno);

	rc = RETRY_NC(connect(sock, ap->ai_addr, ap->ai_addrlen), goto out_close);
	if (rc) {
		xprerrf(SD_DEBUG "Connect to [%s]:%s failed: %d\n", host, serv, errno);
		goto out_close;
	}

	timeout.tv_sec = NET_SOCKET_TIMEOUT_S;
	rc = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
	xassert(!rc, goto out_close, "%d", errno);
	/*
	 * FIXME: do we need to set SO_SNDTIMEO to NET_SOCKET_TIMEOUT_S as well?
	 * Without the TCP_USER_TIMEOUT setting below, a socket write() is still
	 * successful even when the socket is stuck, because the send buffer is
	 * large enough.
	 */
	timeout.tv_sec = 0;
	rc = setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
	xassert(!rc, goto out_close, "%d", errno);
	opt = NET_SOCKET_TIMEOUT_S * 1000;
	rc = setsockopt(sock, IPPROTO_TCP, TCP_USER_TIMEOUT, &opt, sizeof(opt));
	xassert(!rc, goto out_close, "%d", errno);

	xprintf(SD_DEBUG "Connected to [%s]:%s\n", host, serv);

	return sock;

out_close:
	rc = errno;
	close(sock);
	errno = rc;

	return -1;
}

int net_connect_loop(const char *host, const char *port)
{
	static const struct addrinfo hints = {
		.ai_family = AF_UNSPEC,
		.ai_socktype = SOCK_STREAM,
	};

	struct addrinfo *ares, *ap;
	int rc, sock;

	for (;;) {
		rc = getaddrinfo(host, port, &hints, &ares);
		xassert(!rc, ares = NULL, "%d %d", rc, errno);

		for (ap = ares; ap; ap = ap->ai_next) {
			sock = net_connect_one(ap);
			if (sock < 0) {
				if (errno == EINTR)
					goto out_ret;
				continue;
			}

			xprintf(SD_INFO "Connected to %s:%s\n", host, port);
			goto out_ret;
		}

		freeaddrinfo(ares);
		xprerrf(SD_NOTICE "Connect to %s:%s failed: %d\n", host, port, errno);

		sleep(NET_CONNECT_RETRY_S);
		CHECK_CANCELED(goto out_intr);
	}

out_ret:
	freeaddrinfo(ares);
	return sock;
out_intr:
	errno = EINTR;
	return -1;
}

int net_write(int sock, const void *buf, size_t len)
{
	const void *ptr;
	int rc;

	for (ptr = buf; ptr < buf + len; ptr += rc) {
		rc = RETRY_NC(write(sock, ptr, buf + len - ptr));
		if (!rc) {
			errno = EIO;
			return -1;
		}
		if (rc < 0)
			return -1;
	}

	return 0;
}
