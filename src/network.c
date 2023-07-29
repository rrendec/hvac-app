#include <poll.h>
#include <netdb.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

#include "common.h"
#include "network.h"

static int net_connect_one(const struct addrinfo *ap)
{
	int rc, sock, fl, opt;
	struct pollfd fds = {.events = POLLOUT};
	socklen_t len = sizeof(opt);
	char host[NI_MAXHOST];
	char serv[NI_MAXSERV];

	rc = getnameinfo(ap->ai_addr, ap->ai_addrlen,
			 host, sizeof(host), serv, sizeof(serv),
			 NI_NUMERICHOST | NI_NUMERICSERV);
	if (rc) {
		snprintf(host, sizeof(host), "?");
		snprintf(serv, sizeof(serv), "<%d>", rc);
	}

	sock = socket(ap->ai_family, ap->ai_socktype, ap->ai_protocol);
	xassert(sock != -1, return -1, "%d", errno);

	fl = fcntl(sock, F_GETFL, NULL);
	xassert(fl >= 0, goto out_close, "%d", errno);

	rc = fcntl(sock, F_SETFL, fl | O_NONBLOCK);
	xassert(!rc, goto out_close, "%d", errno);

	rc = RETRY_NC(connect(sock, ap->ai_addr, ap->ai_addrlen), goto out_close);
	if (!rc)
		goto connect_ok;

	if (errno != EINPROGRESS) {
		xprerrf(SD_DEBUG "Connect to [%s]:%s failed: %d\n", host, serv, errno);
		goto out_close;
	}

	fds.fd = sock;
	rc = RETRY_NC(poll(&fds, 1, NET_CONNECT_TIMEOUT_S * 1000), goto out_close);
	xassert(rc >= 0, goto out_close, "%d", errno);

	if (!rc) {
		xprerrf(SD_DEBUG "Connect to [%s]:%s timed out\n", host, serv);
		errno = ETIMEDOUT;
		goto out_close;
	}

	xassert(fds.revents & POLLOUT, goto out_close, "%d", fds.revents);

	rc = getsockopt(sock, SOL_SOCKET, SO_ERROR, &opt, &len);
	xassert(!rc, goto out_close, "%d", errno);

	if (opt) {
		xprerrf(SD_DEBUG "Connect to [%s]:%s failed: %d\n", host, serv, opt);
		errno = opt;
		goto out_close;
	}

connect_ok:
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
