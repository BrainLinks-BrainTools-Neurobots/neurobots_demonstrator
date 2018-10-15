/*
 * TimeClient.cpp
 *
 *  Created on: Jul 22, 2011
 *      Author: sprunkc
 */

#include "TimeClient.h"
#include "definitions.h"
#include <iostream>
#include <stdio.h>

TimeClient::TimeClient(char* server_hostname, double send_frequency_hz, bool writeLog) {
	bool status = udpListener.init(0);
  if (!status) {
    std::cerr << "Unable to init listener" << std::endl;
  }
  _portNumber = udpListener.getPort();

  //we init the sender with localhost first, will overwrite the receipient later
  status = udpSender.init(server_hostname, TIMEREQUEST_PORT_NUMBER);
  if (!status) {
    std::cerr << "Unable to init sender" << std::endl;
  } else {
    std::cerr<< "TimeSync: Client init successful on reply port "<<_portNumber<<std::endl;
  }

  run_sender = false;
  run_receiver = false;

  send_frequency = send_frequency_hz;
  write_logfile = writeLog;
  if(write_logfile){
    logfile = fopen("timesync.log", "w");
    if(logfile){
    fprintf(logfile, "#time request sent (client), time (server), time reply received (client)\n");
    fprintf(logfile, "# all times are in nanoseconds\n");
    } else {
      std::cerr<<"COULD NOT OPEN LOGFILE!"<<std::endl;
      write_logfile = false;
    }
  }
}


TimeClient::~TimeClient() {
	stopSyncing();
	if(write_logfile){
	  fclose(logfile);
	}

	//TODO close logfile
}


void TimeClient::startSyncing(){
    assert(!run_sender && !run_receiver);

    //reset the filter?
    mutex_clockMapper.lock();
    clockMapper.reset();
    mutex_clockMapper.unlock();

    //start the threads
	run_sender = true;
	sender = new boost::thread(boost::bind(&TimeClient::sender_worker_function, this));
	run_receiver = true;
	receiver = new boost::thread(boost::bind(&TimeClient::receiver_worker_function, this));
}

void TimeClient::stopSyncing(){
	if(run_sender){
		run_sender = false;
		sender->join();
	}

	if(run_receiver){
		run_receiver = false;
		receiver->join();
	}
}

int64_t TimeClient::getServerTime(int64_t clientTimeNsecs){
  mutex_clockMapper.lock();
  int64_t result = clockMapper.clientTimeToServerTime(clientTimeNsecs);
  mutex_clockMapper.unlock();

  return result;
}

int64_t TimeClient::getClientTime(int64_t serverTimeNsecs){
  mutex_clockMapper.lock();
  int64_t result = clockMapper.serverTimeToClientTime(serverTimeNsecs);
  mutex_clockMapper.unlock();

  return result;
}

bool TimeClient::isFilterStable(){
  mutex_clockMapper.lock();
  bool result = clockMapper.isStable();
  mutex_clockMapper.unlock();

  return result;
}

bool TimeClient::sendRequest(){
  TimeRequestPacket request_packet;
  int64_t time_nsecs;
  get_nano_time(time_nsecs);
  request_packet.timestampClientRequest.timestamp_secs = time_nsecs/1000000000;
  request_packet.timestampClientRequest.timestamp_nsecs = time_nsecs%1000000000;
  request_packet.replyPort = _portNumber;
  bool sendStatus = udpSender.send(&request_packet, sizeof(TimeRequestPacket), TIMEOUT_SEND);
  if (!sendStatus)
    std::cerr << "\nFailure while sending reply" << std::endl;
  return sendStatus;
}

bool TimeClient::receive(){
  TimeReplyPacket reply_packet;
  int64_t time_receive_nsecs;
  int received = udpListener.receive(&reply_packet, sizeof(TimeReplyPacket), TIMEOUT_RECEIVE);
  get_nano_time(time_receive_nsecs);
  if (received > 0) {
    /*
    //this is a hack to cope with bad server clock resolution
    //we artificially increase the roundtrip time by 2 ms, symmetrically.
    //this should not affect the estimate but separate things better
    reply_packet.timestampClientRequest_usecs -= 1000;
    time_receive_usecs += 1000;
    */
    if(write_logfile){
		  fprintf(logfile, "%ld %ld %ld\n", getNSecsFromTimePacket(reply_packet.timestampClientRequest), getNSecsFromTimePacket(reply_packet.timestampServerReply), time_receive_nsecs); 
    }
		
		mutex_clockMapper.lock();
                try {
  		  clockMapper.update(getNSecsFromTimePacket(reply_packet.timestampClientRequest), getNSecsFromTimePacket(reply_packet.timestampServerReply), time_receive_nsecs);
                }
                catch (TICSync::TimeJumpException ex){
		  if(write_logfile){
		    fprintf(logfile, "%% EXCEPTION \"%s\" with data from line above!\n", (char*) ex.what());
		  }
		  fprintf(stderr, "EXCEPTION! %s\n", (char*) ex.what());		  
		}
		mutex_clockMapper.unlock();
    return true;
  }
  return false;
}



void TimeClient::sender_worker_function(){
	std::cerr<<"TimeSync: Starting sender thread "<<std::endl;
	while(run_sender){
		sendRequest();
		SLEEP(1000000.0/send_frequency);
	}
	std::cerr<<"TimeSync: Stopping sender thread "<<std::endl;
}

void TimeClient::receiver_worker_function(){
	std::cerr<<"TimeSync: Starting receiver thread "<<std::endl;
	while(run_receiver){
		receive();
	}
	std::cerr<<"TimeSync: Stopping receiver thread "<<std::endl;
}

