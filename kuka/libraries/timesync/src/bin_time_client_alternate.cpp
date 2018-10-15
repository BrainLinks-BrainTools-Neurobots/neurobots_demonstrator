#include <timelib/timelib.h>

#include <string>
#include <iostream>
#include <unistd.h>

#ifdef _WIN32
#include <windows.h>
#endif

int main(int argc, char** argv)
{
  if(argc!=2){
    std::cout<<"usage: "<<argv[0]<<" <hostname/ip>"<<std::endl;
    return 0;
  }


  timelib::init(argv[1]);

  while(true){
    int64_t usecs = timelib::getMyTime();
    int64_t usecstwo = timelib::getClientTime(usecs);
    std::cout<<usecs<<" "<<usecstwo<<" "<<usecstwo-usecs<<"\n";
#ifdef _WIN32
    Sleep(1000);
#else
    usleep(1000000);
#endif
  }

  timelib::cleanUp();

}

