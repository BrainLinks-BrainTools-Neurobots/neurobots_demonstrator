#ifndef UDP_LISTENER_H
#define UDP_LISTENER_H

#ifdef _WIN32
#include <WinSock2.h>
#else
#include <netinet/in.h>
#endif

//under VxWorks try adding these includes
//types.h, mbuf.h, socket.h, socketvar.h

/**
 * brief listen on a UDP port
 */
class UDPListener
{
  public:
    UDPListener();
    ~UDPListener();

    /**
     * if port is zero, it is a random free port
     * use getPort() to get the port number in this case
     */
    bool init(int port);
    bool close();
    /**
     * receive data.
     * @return the number of bytes read, -1 on error.
     */
    int receive(void* packet, int bufferSize, double timeout = 0.1);

    //! return the source address, valid after receive
    const sockaddr_in& srcAddress() const { return _srcAddr;}

    /**
     * return the port number to which we listen
     */
    int getPort();

  protected:
#ifdef _WIN32
	SOCKET _sock;
#else
    int _sock;
#endif
    struct sockaddr_in _srcAddr;
};

#endif
