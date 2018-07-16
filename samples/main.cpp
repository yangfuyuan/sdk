
#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <signal.h>
#include <memory>
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
    const int intensities = 0;

    signal(SIGINT, Stop);
    signal(SIGTERM, Stop);
    laser.setSerialPort(port);
    laser.setSerialBaudrate(baud);//s4b 波特率一定要设置对为115200
    laser.setIntensities(intensities);//s4b 一定要设置为true
    laser.setMaxRange(16.0);
    laser.setMinRange(0.1);
    laser.setMaxAngle(180);
    laser.setMinAngle(-180);
    laser.setHeartBeat(false);
    laser.setReversion(false);
    laser.setFixedResolution(false);
    laser.setAutoReconnect(true);

    laser.initialize();


    while(!running){
		bool hardError;
		LaserScan scan;
        std::vector<gline> lines;

        if(laser.doProcessSimple(scan,lines, hardError )){
            for(int i =0; i < scan.ranges.size(); i++ ){
                float angle = scan.config.min_angle + i*scan.config.ang_increment;
                float dis = scan.ranges[i];

            }
            fprintf(stderr,"min_angle: %f \n",scan.config.min_angle);
            fprintf(stderr,"max_angle: %f \n",scan.config.max_angle);
			fprintf(stderr,"Scan received: %u ranges\n",(unsigned int)scan.ranges.size());

            fprintf(stdout, "fit line size: %d\n",lines.size());
            for(std::vector<gline>::const_iterator it = lines.begin(); it != lines.end(); it++) {
                fprintf(stdout, "line length: %f,   line angle: %f\n", (*it).distance, (*it).angle);
            }

		}

	}
  laser.turnOff();
  laser.disconnecting();

  return 0;


}
