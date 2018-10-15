#include "udpSender.h"

#ifdef _WIN32
#include <WinSock2.h>
#else
#include <sys/types.h>
#include <sys/socket.h>

//under VxWorks try adding these includes
//types.h, mbuf.h, socket.h, socketvar.h
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#endif

#include <iostream>
#include <cstring>
#include <cmath>
#include <unistd.h>

UDPSender::UDPSender() :
  _sock(-1)
{
  memset(&_remoteaddr, 0, sizeof(_remoteaddr));
}

UDPSender::~UDPSender()
{
}

bool UDPSender::init(const char* host, int port)
{
#ifdef _WIN32
	WSADATA wsaData = {0};
	int error = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (error != 0)
		return false;
#endif
	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if(sock == -1) 
    return false;

  struct hostent* he = gethostbyname(host);
  if(he == NULL) {
    std::cerr  << ": could not lookup host IP" << std::endl;
    return false;
  }

  _sock = sock;
  _remoteaddr.sin_family = AF_INET;	 
  _remoteaddr.sin_port = htons(port); 
  _remoteaddr.sin_addr = *((struct in_addr *)he->h_addr);
  return true;
}

bool UDPSender::close()
{
#ifdef _WIN32
	if (_sock)
		closesocket(_sock);

	WSACleanup();
	return true;
#else
  if (_sock < 0)
    return false;

  bool retVal = ::close(_sock) == 0;
  _sock = -1;
  return retVal; 
#endif
}

bool UDPSender::send(void* packet, int length, double timeout)
{
   fd_set writesock, errsock;
  int n, result;

  FD_ZERO(&writesock);
  FD_ZERO(&errsock);
  FD_SET(_sock, &writesock);
  FD_SET(_sock, &errsock);
  if (timeout < 0.) {
    if (select(_sock + 1, NULL, &writesock, &errsock, NULL) < 0)
      return false;
  }
  else {
    struct timeval t;
    t.tv_sec = (int)floor(timeout);
    t.tv_usec = (int)floor((timeout - t.tv_sec) * 1e6);
    result = select(_sock + 1, NULL, &writesock, &errsock, &t);
    if (result <= 0)
      return false;
  }
  if (FD_ISSET(_sock, &errsock)) 
    return false;
#ifdef _WIN32
	n = sendto(_sock, (const char*)packet, length, 0, (struct sockaddr *)&_remoteaddr, sizeof(struct sockaddr));
#else
  n = sendto(_sock, packet, length, 0, (struct sockaddr *)&_remoteaddr, sizeof(struct sockaddr));
#endif
  return (n == length);
}
