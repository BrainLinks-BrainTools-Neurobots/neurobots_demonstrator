#include <string>
#include <iostream>
#include <iomanip>
#include <csignal>
#include <cstdlib>

#include "timelib/timelib.h"
#include <unistd.h>

static bool g_quit = false;

void signalHandler(int /*sig*/)
{
  std::cerr << "received SIGINT" << std::endl;
  static int sigIntCnt = 0;
  g_quit = true;
  if (++sigIntCnt > 2) {
    std::cerr << "Forcing quit" << std::endl;
    exit(1);
  }
}

int main(int argc, const char** argv)
{
  if (argc!=2) {
    std::cout<<"usage: "<<argv[0]<<" <hostname/ip>"<<std::endl;
    return 0;
  }

  signal(SIGINT, &signalHandler);
  timelib::init(argv[1]);

  while(!g_quit) {
    int64_t now = timelib::getMyTime();
    if(timelib::filterStable()){
      std::cout << std::fixed << "stable. time here " << now << " at server " << timelib::getServerTime(now)
        << " diff " << (timelib::getServerTime(now) - now) << std::endl;
    } else
      std::cout << "unstable." << std::endl;
    usleep(1*1000000);
  }

}

