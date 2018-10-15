#ifndef UDP_SENDER_H
#define UDP_SENDER_H

#ifdef _WIN32
#include <WinSock2.h>
#else
#include <netinet/in.h>
#endif

//under VxWorks try adding these includes
//types.h, mbuf.h, socket.h, socketvar.h


class UDPSender
{
  public:
    UDPSender();
    ~UDPSender();

    bool init(const char* host, int port);
    bool close();
    bool send(void* packet, int length, double timeout);

    const sockaddr_in& remoteAddress() const { return _remoteaddr;}
    sockaddr_in& remoteAddress() { return _remoteaddr;}

  protected:
#ifdef _WIN32
	SOCKET _sock;
#else
    int _sock;
#endif
	struct sockaddr_in _remoteaddr;
};

#endif
