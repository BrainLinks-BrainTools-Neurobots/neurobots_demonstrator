//from http://suacommunity.com/dictionary/gettimeofday-entry.php
//There is no direct analog to gettimeofday(...) in Windows!

#ifndef LINUX_TIME_HPP
#define LINUX_TIME_HPP

#include <time.h>
#include <WinSock2.h>
#include <iostream>

#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
#define DELTA_EPOCH_IN_MICROSECS  11644473600000000Ui64
#else
#define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL
#endif

struct timezone_
{
	int  tz_minuteswest; /* minutes W of Greenwich */
	int  tz_dsttime;     /* type of dst correction */
};

// Definition of a gettimeofday function
int gettimeofday(timeval *tv, timezone_ *tz);

#endif LINUX_TIME_HPP