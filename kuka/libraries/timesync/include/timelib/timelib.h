#ifndef TIMELIB_H
#define TIMELIB_H


#include <string>
#include <stdint.h>

class TimeClient;

namespace timelib {

	bool init(std::string host);
	bool cleanUp();
	int64_t getServerTime(int64_t clientTimeUsecs);
	int64_t getClientTime(int64_t serverTimeUsecs);
	int64_t getMyTime();
        bool filterStable();
}


#endif
