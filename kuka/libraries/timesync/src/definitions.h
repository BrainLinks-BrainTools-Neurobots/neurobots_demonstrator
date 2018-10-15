#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <stdint.h>

#define TIMEREQUEST_PORT_NUMBER 15001
#define TIMEREPLY_PORT_NUMBER 15002

#define TIMEOUT_RECEIVE 0.0025
//we should set this way lower! this will affect precision?!
#define TIMEOUT_SEND 0.00001

#ifdef _WIN32
#pragma pack(push,1)
struct TimePacket {
   uint64_t timestamp_secs;
   uint64_t timestamp_nsecs;
};

struct TimeRequestPacket {
   TimePacket timestampClientRequest;
   int32_t replyPort;
};

struct TimeReplyPacket {
   unsigned long long receiveCount;
   TimePacket timestampClientRequest;
   TimePacket timestampServerReply;
};
#pragma pack(pop)
#else
struct __attribute__ ((__packed__)) TimePacket {
   uint64_t timestamp_secs;
   uint64_t timestamp_nsecs;
};

struct __attribute__ ((__packed__)) TimeRequestPacket {
   TimePacket timestampClientRequest;
   int32_t replyPort;
};

struct __attribute__ ((__packed__)) TimeReplyPacket {
   unsigned long long receiveCount;
   TimePacket timestampClientRequest;
   TimePacket timestampServerReply;
};
#endif


#endif
