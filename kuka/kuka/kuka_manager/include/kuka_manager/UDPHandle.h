#ifndef UDP_SENDER_H
#define UDP_SENDER_H

#ifdef _WIN32
#include <WinSock2.h>
#else
#include <netinet/in.h>
#endif

//under VxWorks try adding these includes
//types.h, mbuf.h, socket.h, socketvar.h


class UDPHandle
{
  public:
	UDPHandle();
    ~UDPHandle();

    bool init(const char* host, int portSend, int portRecieve);
    bool initRecieve(int port);
    bool close();
    bool send(void* packet, int length, double timeout);
    int receive(void* packet, long length, double timeout);
    int receiveFrom(void* packet, int bufferSize, double timeout);

    const sockaddr_in& remoteAddress() const { return _remoteaddrSend;}
    sockaddr_in& remoteAddress() { return _remoteaddrSend;}

  protected:
#ifdef _WIN32
	SOCKET _sock;
#else
    int _sockSend;
    int _sockRecieve;
#endif
	struct sockaddr_in _remoteaddrSend;
	struct sockaddr_in _remoteaddrRecieve;
	struct sockaddr_in _srcAddr;
};

#endif
