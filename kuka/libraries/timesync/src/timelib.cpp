#include <timelib/timelib.h>

#ifdef WIN32
#include "linux_time.h"
#endif
#include "TimeClient.h"

namespace timelib {
	TimeClient* timeClient = NULL;

	bool init(std::string host) {
		timeClient = new TimeClient((char*)host.c_str(), 30, true);
		timeClient->startSyncing();
		return true;
	}

	bool cleanUp() {
		if (timeClient != NULL) {
			delete timeClient;
			timeClient = NULL;
		}
		return true;
	}

  bool filterStable() {
    return timeClient->isFilterStable();
  }

	int64_t getServerTime(int64_t clientTimeUsecs) {
		if (timeClient == NULL) {
			std::cerr << "You cannot use this lib without calling init()!" << std::endl;
			return 0;
		}

		if(timeClient->isFilterStable()){
			return timeClient->getServerTime(clientTimeUsecs);
		} else {
			std::cerr << "The time client is unstable." << std::endl;
		}

		return 0;
	}

	int64_t getClientTime(int64_t serverTimeUsecs) {
		if (timeClient == NULL) {
			std::cerr << "You cannot use this lib without calling init()!" << std::endl;
			return 0;
		}

		if(timeClient->isFilterStable()){
			return timeClient->getClientTime(serverTimeUsecs);
		} else {
			std::cerr << "The time client is unstable." << std::endl;
		}

		return 0;
	}

	int64_t getMyTime() {
		struct timeval ts;
		gettimeofday(&ts, 0);
		int64_t nsecs = (int64_t) ts.tv_sec* ((int64_t) 1000000000) + (int64_t) ts.tv_usec*1000;
		return nsecs;
	}
}
