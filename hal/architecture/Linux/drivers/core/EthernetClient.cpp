/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2020 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * Based on Arduino ethernet library, Copyright (c) 2010 Arduino LLC. All right reserved.
 */

#include "EthernetClient.h"
#include <cstdio>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/sockios.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>
#include <sys/time.h>
#include <netinet/tcp.h>
#include <errno.h>
#include "log.h"

EthernetClient::EthernetClient() : _sock(-1), socketTimeout(1000)
{
}

EthernetClient::EthernetClient(int sock) : _sock(sock), socketTimeout(1000)
{
}

int EthernetClient::connect(const char* host, uint16_t port)
{
	struct addrinfo hints, *servinfo, *localinfo, *p;
	int rv;
	char s[INET6_ADDRSTRLEN];
	char port_str[6];
	bool use_bind = (_srcip != 0);

	close();

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	sprintf(port_str, "%hu", port);
	if ((rv = getaddrinfo(host, port_str, &hints, &servinfo)) != 0) {
		logError("getaddrinfo: %s\n", gai_strerror(rv));
		return -1;
	}
	if (use_bind) {
		if ((rv = getaddrinfo(_srcip.toString().c_str(), port_str, &hints, &localinfo)) != 0) {
			logError("getaddrinfo: %s\n", gai_strerror(rv));
			return -1;
		}
	}

	// loop through all the results and connect to the first we can
	for (p = servinfo; p != NULL; p = p->ai_next) {
		if ((_sock = socket(p->ai_family, p->ai_socktype,
		                    p->ai_protocol)) == -1) {
			logError("socket: %s\n", strerror(errno));
			continue;
		}

		if (use_bind) {
			if (::bind(_sock, localinfo->ai_addr, localinfo->ai_addrlen) == -1) {
				close();
				logError("bind: %s\n", strerror(errno));
				return -1;
			}
		}

		if (::connect(_sock, p->ai_addr, p->ai_addrlen) == -1) {
			close();
			logError("connect: %s\n", strerror(errno));
			continue;
		}

		// Sets the socket timeout
		struct timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = socketTimeout * 1000000;
		setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));
		setsockopt(_sock, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout));

		break;
	}

	if (p == NULL) {
		logError("failed to connect\n");
		return -1;
	}

	void *addr = &(((struct sockaddr_in*)p->ai_addr)->sin_addr);
	inet_ntop(p->ai_family, addr, s, sizeof s);
	logDebug("connected to %s\n", s);

	freeaddrinfo(servinfo); // all done with this structure
	if (use_bind) {
		freeaddrinfo(localinfo); // all done with this structure
	}

	return 1;
}

int EthernetClient::connect(IPAddress ip, uint16_t port)
{
	return connect(ip.toString().c_str(), port);
}

size_t EthernetClient::write(uint8_t b)
{
	return write(&b, 1);
}

size_t EthernetClient::write(const uint8_t *buf, size_t size)
{
	int bytes = 0;

	if (_sock == -1) {
		return 0;
	}

	while (size > 0) {
		int rc = send(_sock, buf + bytes, size, MSG_NOSIGNAL | MSG_DONTWAIT);
		if (rc == -1) {
			logError("send: %s\n", strerror(errno));
			close();
			break;
		}
		bytes += rc;
		size -= rc;
	}

	return bytes;
}

size_t EthernetClient::write(const char *str)
{
	if (str == NULL) {
		return 0;
	}
	return write((const uint8_t *)str, strlen(str));
}
size_t EthernetClient::write(const char *buffer, size_t size)
{
	return write((const uint8_t *)buffer, size);
}

int EthernetClient::available()
{
	int count = 0;

	if (_sock != -1) {
		ioctl(_sock, SIOCINQ, &count);
	}

	return count;
}

int EthernetClient::read()
{
	uint8_t b;
	if ( recv(_sock, &b, 1, MSG_DONTWAIT) > 0 ) {
		// recv worked
		return b;
	} else {
		// No data available
		return -1;
	}
}

int EthernetClient::read(uint8_t *buf, size_t bytes)
{
	return recv(_sock, buf, bytes, MSG_DONTWAIT);
}

int EthernetClient::peek()
{
	uint8_t b;

	if (recv(_sock, &b, 1, MSG_PEEK | MSG_DONTWAIT) > 0) {
		return b;
	} else {
		return -1;
	}
}

void EthernetClient::flush()
{
	int count = 0;

	if (_sock != -1) {
		while (true) {
			ioctl(_sock, SIOCOUTQ, &count);
			if (count == 0) {
				return;
			}
			usleep(1000);
		}
	}
}

void EthernetClient::stop()
{
	if (_sock == -1) {
		return;
	}

	// attempt to close the connection gracefully (send a FIN to other side)
	shutdown(_sock, SHUT_RDWR);

	timeval startTime, curTime;
	gettimeofday(&startTime, NULL);

	// wait up to a second for the connection to close
	do {
		uint8_t s = status();
		if (s == ETHERNETCLIENT_W5100_CLOSED) {
			break; // exit the loop
		}
		usleep(1000);
		gettimeofday(&curTime, NULL);
	} while (((curTime.tv_sec - startTime.tv_sec) * 1000000) + (curTime.tv_usec - startTime.tv_usec) <
	         1000000);

	// free up the socket descriptor
	::close(_sock);
	_sock = -1;
}

uint8_t EthernetClient::status()
{
	if (_sock == -1) {
		return ETHERNETCLIENT_W5100_CLOSED;
	}

	struct tcp_info tcp_info;
	int tcp_info_length = sizeof(tcp_info);

	if ( getsockopt( _sock, SOL_TCP, TCP_INFO, (void *)&tcp_info,
	                 (socklen_t *)&tcp_info_length ) == 0 ) {
		switch (tcp_info.tcpi_state) {
		case TCP_ESTABLISHED:
			return ETHERNETCLIENT_W5100_ESTABLISHED;
		case TCP_SYN_SENT:
			return ETHERNETCLIENT_W5100_SYNSENT;
		case TCP_SYN_RECV:
			return ETHERNETCLIENT_W5100_SYNRECV;
		case TCP_FIN_WAIT1:
		case TCP_FIN_WAIT2:
			return ETHERNETCLIENT_W5100_FIN_WAIT;
		case TCP_TIME_WAIT:
			return TCP_TIME_WAIT;
		case TCP_CLOSE:
			return ETHERNETCLIENT_W5100_CLOSED;
		case TCP_CLOSE_WAIT:
			return ETHERNETCLIENT_W5100_CLOSING;
		case TCP_LAST_ACK:
			return ETHERNETCLIENT_W5100_LAST_ACK;
		case TCP_LISTEN:
			return ETHERNETCLIENT_W5100_LISTEN;
		case TCP_CLOSING:
			return ETHERNETCLIENT_W5100_CLOSING;
		}
	}

	return ETHERNETCLIENT_W5100_CLOSED;
}

uint8_t EthernetClient::connected()
{
	return status() == ETHERNETCLIENT_W5100_ESTABLISHED || available();
}

void EthernetClient::close()
{
	if (_sock != -1) {
		::close(_sock);
		_sock = -1;
	}
}

void EthernetClient::bind(IPAddress ip)
{
	_srcip = ip;
}

int EthernetClient::getSocketNumber()
{
	return _sock;
}

// the next function allows us to use the client returned by
// EthernetServer::available() as the condition in an if-statement.

EthernetClient::operator bool()
{
	return _sock != -1;
}

bool EthernetClient::operator==(const EthernetClient& rhs)
{
	return _sock == rhs._sock && _sock != -1 && rhs._sock != -1;
}
