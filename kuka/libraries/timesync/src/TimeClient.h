/*
 * TimeClient.h
 *
 *  Created on: Jul 22, 2011
 *      Author: sprunkc
 */

#ifndef TIMECLIENT_H_
#define TIMECLIENT_H_

#include <stdint.h>
#include <boost/thread.hpp>
#ifdef _WIN32
#include "linux_time.h"
#define SLEEP(time) Sleep(time / 1000.0)	//milli
#else
#include <sys/time.h>
#define SLEEP(time) usleep(time)			//micro
#endif

#include "udpListener.h"
#include "udpSender.h"
#include "definitions.h"

#include <vector>
#include <queue>
#include <TICSync/LinearTwoWayClockSync.h>

class TimeClient {
  /*
	typedef struct {
	  int64_t time_send_usecs;
	  int64_t time_receive_usecs;
	  int64_t time_server_usecs;
	} time_info;

	typedef struct {
	  int64_t time_send_usecs;
	  int32_t id;
	} time_request;

	typedef struct {
	  int64_t time_receive_usecs;
	  int64_t time_server_usecs;
	  int32_t id;
	} time_reply;
  */
public:
	TimeClient(char* server_hostname, double send_frequency_hz, bool writeLog=false);
	virtual ~TimeClient();

	void startSyncing();
	void stopSyncing();
	int64_t getServerTime(int64_t clientTimeNsecs);
	int64_t getClientTime(int64_t serverTimeNsecs);
	bool isFilterStable();
private:
  UDPListener udpListener;
  UDPSender udpSender;

  boost::thread* sender;
  boost::thread* receiver;

  bool run_sender;
  bool run_receiver;

int _portNumber;

  boost::mutex mutex_clockMapper;

  double send_frequency;
  bool write_logfile;
  FILE* logfile;

  TICSync::LinearTwoWayClockSync<int64_t> clockMapper;

  bool sendRequest();
  bool receive();

  void sender_worker_function();
  void receiver_worker_function();

  inline void get_nano_time(int64_t &nsecs) {
    struct timeval ts;
    gettimeofday(&ts,0);
    nsecs = (int64_t) ts.tv_sec* ((int64_t) 1000000000) + (int64_t) ts.tv_usec*1000;
  }

  inline int64_t getNSecsFromTimePacket(const TimePacket& packet){
    return ((int64_t) packet.timestamp_secs*(int64_t)1000000000) + ((int64_t) packet.timestamp_nsecs);
  }

};

#endif /* TIMECLIENT_H_ */
