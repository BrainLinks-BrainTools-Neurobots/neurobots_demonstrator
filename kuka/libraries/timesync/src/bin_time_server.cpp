#include "definitions.h"
#include "udpListener.h"
#include "udpSender.h"

#include <iostream>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace {
  inline double get_my_local_time()
  {
    struct timeval ts;
    gettimeofday(&ts, 0);
    return ts.tv_sec + ts.tv_usec * 1e-6;
  }

  inline void fill_time(TimePacket& packet)
  {
    struct timeval ts; 
    gettimeofday(&ts,0);
    packet.timestamp_secs = ts.tv_sec;
    packet.timestamp_nsecs = (int64_t)1000 * (int64_t)ts.tv_usec;
  }
}


int main()
{
  UDPListener udpListener;
  bool status = udpListener.init(TIMEREQUEST_PORT_NUMBER);
  if (!status) {
    std::cerr << "Unable to init listener" << std::endl;
  }

  UDPSender udpSender;
  // we init the sender with localhost first, will overwrite the recipient later
  // also the port will be changed later. Just some default number here
  status = udpSender.init("localhost", 4242);
  if (!status) {
    std::cerr << "Unable to init sender" << std::endl;
  }

  double lastStatusTime = get_my_local_time();
  TimeRequestPacket request;
  TimeReplyPacket reply;
  unsigned int num_packages=0;
  while (1) {
    int received = udpListener.receive(&request, sizeof(TimeRequestPacket), TIMEOUT_RECEIVE);
    if (received > 0) {
      //std::cerr << ".";
      // fill our local time
      fill_time(reply.timestampServerReply);
      //std::cout << "receiving packet from " << inet_ntoa(udpListener.srcAddress().sin_addr) << "        \r" << std::flush;

      //fill reply with values
      reply.timestampClientRequest = request.timestampClientRequest;
      udpSender.remoteAddress().sin_addr = udpListener.srcAddress().sin_addr;
      //std::cerr << "sending back to port " << request.port << std::endl;
      udpSender.remoteAddress().sin_port = ntohs(request.replyPort);

      bool sendStatus = udpSender.send(&reply, sizeof(TimeReplyPacket), TIMEOUT_SEND);
      if (!sendStatus) {
        std::cerr << "\nFailure while sending reply" << std::endl;
      } else {
        num_packages++;
      }

    }
    double now = get_my_local_time();
    if (now - lastStatusTime > 3.0){
      std::cerr << num_packages << " packages returned    \r" << std::flush;
      lastStatusTime = now;
    }
  }
  return 0;
}
