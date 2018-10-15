#include "udpListener.h"

#include <iostream>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <sys/ioctl.h>
#include <unistd.h>


UDPListener::UDPListener() :
  _sock(-1)
{
  memset(&_srcAddr, 0, sizeof(_srcAddr));
}

UDPListener::~UDPListener()
{
  close();
}

bool UDPListener::init(int port)
{
#ifdef _WIN32
	if (_sock != INVALID_SOCKET) {
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
  if (_sock >= 0) {
    std::cerr  << ": Already connected" << std::endl;
    return false;
  }

  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if(sock == -1)
    return false;
#endif
  sockaddr_in localaddr;
  localaddr.sin_family = AF_INET;
  localaddr.sin_port = htons(port);
  localaddr.sin_addr.s_addr = INADDR_ANY;  
  memset(localaddr.sin_zero, '\0', sizeof(localaddr.sin_zero));
  
  if (bind(sock, (struct sockaddr *)&localaddr, sizeof(struct sockaddr)) == -1) {
    std::cerr << ": Could not bind to local port" << std::endl;
    return false;
  }

  _sock = sock;
  return true;
}

bool UDPListener::close()
{
#ifdef _WIN32
	if (_sock)
		closesocket(_sock);

	WSACleanup();
	return true;
#else
  if (_sock >= 0) {
    bool retVal = ::close(_sock) == 0;
    _sock = -1;
    return retVal;
  }
  return false;
#endif
}

int UDPListener::receive(void* packet, int bufferSize, double timeout)
{
  if(timeout < 0.) {
    int bytes_available = -1;
    ioctl(_sock, FIONREAD, &bytes_available);
    if (bytes_available < bufferSize)
      return -1;
  }
  else {
    fd_set readsock, errsock;
    FD_ZERO(&readsock);
    FD_ZERO(&errsock);
    FD_SET(_sock, &readsock);
    FD_SET(_sock, &errsock);
    struct timeval t;
    t.tv_sec = (int)floor(timeout);
    t.tv_usec = (int)floor((timeout - t.tv_sec) * 1e6);
    int result = select(_sock + 1, &readsock, NULL, &errsock, &t);
    if(result < 0)
      return -1;
    else if (result == 0)
      return 0;
    if (FD_ISSET(_sock, &errsock)) 
      return -1;
  }

#ifdef _WIN32
	int addr_len = sizeof(struct sockaddr);
	int ret = recvfrom(_sock, (char*)packet, bufferSize, 0, (struct sockaddr *)&_srcAddr, &addr_len);
#else
  socklen_t addr_len = sizeof(struct sockaddr);
  int ret = recvfrom(_sock, packet, bufferSize, 0, (struct sockaddr *)&_srcAddr, &addr_len);
#endif
 return ret;
}

int UDPListener::getPort()
{
  if (_sock == -1) {
    std::cerr << __PRETTY_FUNCTION__ << " ERROR no socket" << std::endl;
    return -1;
  }
  struct sockaddr_storage ss;
  struct sockaddr_in *sa = (struct sockaddr_in *)&ss;
  socklen_t bufferSize = sizeof(ss);
  if (getsockname(_sock, (sockaddr*)sa, &bufferSize) != 0) {
    std::cerr << __PRETTY_FUNCTION__ << " ERROR unable to get socket name" << std::endl;
    return -1;
  } else {
    int portNumber = ntohs(sa->sin_port);
    //std::cerr << __PRETTY_FUNCTION__ << " port is " << portNumber << std::endl;
    return portNumber;
  }
}
