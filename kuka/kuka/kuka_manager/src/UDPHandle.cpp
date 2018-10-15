#include "kuka_manager/UDPHandle.h"

#ifdef _WIN32
#include <WinSock2.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

//under VxWorks try adding these includes
//types.h, mbuf.h, socket.h, socketvar.h
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#endif

#include <iostream>
#include <cstring>
#include <cmath>

UDPHandle::UDPHandle() :
		_sockSend(-1), _sockRecieve(-1) {
	memset(&_remoteaddrSend, 0, sizeof(_remoteaddrSend));
	memset(&_remoteaddrRecieve, 0, sizeof(_remoteaddrRecieve));
	memset(&_srcAddr, 0, sizeof(_srcAddr));
}

UDPHandle::~UDPHandle() {
}

bool UDPHandle::init(const char* host, int portSend, int portRecieve) {
#ifdef _WIN32
	WSADATA wsaData = {0};
	int error = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (error != 0)
	return false;
#endif

	//std::cout << "host: " << host << ", portS: "<<portSend << ", portR: " << portRecieve<<std::endl;
	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock == -1)
		return false;

	struct hostent* he = gethostbyname(host);
	if (he == NULL) {
		std::cerr << ": could not lookup host IP" << std::endl;
		return false;
	}

	int sock2 = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock2 == -1)
		return false;

	_sockSend = sock;
	_remoteaddrSend.sin_family = AF_INET;
	_remoteaddrSend.sin_port = htons(portSend);
	_remoteaddrSend.sin_addr = *((struct in_addr *) he->h_addr);

	// Init receiver

	// Create the socket and bind it to a local interface and port
	if ((_sockRecieve = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		std::cout << "Can't create socket" << std::endl;
		return 0;
	}
	_remoteaddrRecieve.sin_family = AF_INET;
	_remoteaddrRecieve.sin_port = htons(portRecieve);
	_remoteaddrRecieve.sin_addr.s_addr = htonl(INADDR_ANY);
	//

	if (bind(_sockRecieve, (sockaddr *) &_remoteaddrRecieve, sizeof(_remoteaddrRecieve)) < 0) {
		std::cout << "Bind failed" << std::endl;
		return 0;
	}

	return true;
}

bool UDPHandle::initRecieve(int port) {
#ifdef _WIN32
	if (_sockRecieve != INVALID_SOCKET) {
		std::cerr << "Already connected" << std::endl;
		return false;
	}

	WSADATA wsaData = {0};
	int error = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (error != 0)
	return false;

	int sock = socket(AF_INET, SOCK_DGRAM, 0);

	if (sock == INVALID_SOCKET) {
		return false;
	}
#else
	if (_sockRecieve >= 0) {
		std::cerr << ": Already connected" << std::endl;
		return false;
	}

	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock == -1)
		return false;
#endif
	sockaddr_in localaddr;
	localaddr.sin_family = AF_INET;
	localaddr.sin_port = htons(port);
	localaddr.sin_addr.s_addr = INADDR_ANY;
	memset(localaddr.sin_zero, '\0', sizeof(localaddr.sin_zero));

	if (bind(sock, (struct sockaddr *) &localaddr, sizeof(struct sockaddr)) == -1) {
		std::cerr << ": Could not bind to local port" << std::endl;
		return false;
	}

	_sockRecieve = sock;
	return true;
}

bool UDPHandle::close() {
#ifdef _WIN32
	if (_sock)
	closesocket(_sock);

	WSACleanup();
	return true;
#else
	if (_sockSend < 0)
		return false;

	bool retVal = ::close(_sockSend) == 0;
	_sockSend = -1;

	if (_sockRecieve < 0)
		return false;

	retVal = ::close(_sockRecieve) == 0;
	_sockRecieve = -1;
	return retVal;
#endif
}

bool UDPHandle::send(void* packet, int length, double timeout) {
	fd_set writesock, errsock;
	int n, result;

	FD_ZERO(&writesock);
	FD_ZERO(&errsock);
	FD_SET(_sockSend, &writesock);
	FD_SET(_sockSend, &errsock);
	if (timeout < 0.) {
		if (select(_sockSend + 1, NULL, &writesock, &errsock, NULL) < 0)
			return false;
	} else {
		struct timeval t;
		t.tv_sec = (int) floor(timeout);
		t.tv_usec = (int) floor((timeout - t.tv_sec) * 1e6);
		result = select(_sockSend + 1, NULL, &writesock, &errsock, &t);
		if (result <= 0)
			return false;
	}
	if (FD_ISSET(_sockSend, &errsock))
		return false;
	n = sendto(_sockSend, packet, length, 0, (struct sockaddr *) &_remoteaddrSend, sizeof(struct sockaddr));

	return (n == length);
}

int UDPHandle::receive(void* packet, long length, double timeout) {
	int n = recv(_sockRecieve, packet, length, 0);

	return n;
}

int UDPHandle::receiveFrom(void* packet, int bufferSize, double timeout) {
	fd_set readsock, errsock;
	FD_ZERO(&readsock);
	FD_ZERO(&errsock);
	FD_SET(_sockRecieve, &readsock);
	FD_SET(_sockRecieve, &errsock);
	if (timeout < 0.) {
		if (select(_sockRecieve + 1, &readsock, NULL, &errsock, NULL) < 0)
			return -1;
	} else {
		struct timeval t;
		t.tv_sec = (int) floor(timeout);
		t.tv_usec = (int) floor((timeout - t.tv_sec) * 1e6);
		int result = select(_sockRecieve + 1, &readsock, NULL, &errsock, &t);
		if (result < 0)
			return -1;
		else if (result == 0)
			return 0;
	}
	if (FD_ISSET(_sockRecieve, &errsock))
		return -1;

#ifdef _WIN32
	int addr_len = sizeof(struct sockaddr);
	int ret = recvfrom(_sock, (char*)packet, bufferSize, 0, (struct sockaddr *)&_srcAddr, &addr_len);
#else
	socklen_t addr_len = sizeof(struct sockaddr);
	int ret = recvfrom(_sockRecieve, packet, bufferSize, 0, (struct sockaddr *) &_srcAddr, &addr_len);
#endif
	return ret;
}
