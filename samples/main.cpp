
#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <signal.h>
#include <memory>
//#include <unistd.h>
using namespace std;
using namespace ydlidar;
CYdLidar laser;
static bool running = false;

static void Stop(int signo)   
{  
    
    printf("Received exit signal\n");
    running = true;
     
}  

int main(int argc, char * argv[])
{
	printf(" YDLIDAR C++ TEST\n");
    std::string port;
    std::string baudrate;
    printf("Please enter the lidar port:");
    std::cin>>port;
    printf("Please enter the lidar baud rate:");
    std::cin>>baudrate;
    const int baud = atoi(baudrate.c_str());
    const int intensities =  0;

    signal(SIGINT, Stop);
    signal(SIGTERM, Stop);

    laser.setSerialPort(port);
    laser.setSerialBaudrate(baud);
    laser.setFixedResolution(false);
    laser.setHeartBeat(false);
    laser.setReversion(false);
    laser.setIntensities(intensities);
    laser.setAutoReconnect(true);//异常是否重新连接
    laser.initialize();
    while(!running){
		bool hardError;
		LaserScan scan;

		if(laser.doProcessSimple(scan, hardError )){
            fprintf(stdout,"Scan received: %u ranges\n",(unsigned int)scan.ranges.size());
            fflush(stdout);
		}
        //usleep(50*1000);
	}


    laser.turnOff();
    laser.disconnecting();

    return 0;


}
