
#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <signal.h>
#include <memory>
#include <regex>
using namespace std;
using namespace ydlidar;
CYdLidar laser;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif

int main(int argc, char * argv[])
{
	printf(" YDLIDAR C++ TEST\n");
    std::string port;
    int baudrate;
    std::string intensity;
    ydlidar::init(argc, argv);

    std::map<std::string, std::string> ports =  ydlidar::YDlidarDriver::lidarPortList();
    std::map<std::string,std::string>::iterator it;
    if(ports.size()==1) {
        it = ports.begin();
        printf("Lidar[%s] detected, whether to select current radar(yes/no)?:", it->first.c_str());
        std::string ok;
        std::cin>>ok;
        for (size_t i=0; i <ok.size(); i++) {
            ok[i] = tolower(ok[i]);
        }
        if(ok.find("yes") != std::string::npos ||atoi(ok.c_str()) == 1 ) {
            port = it->second;
        }else{
            printf("Please enter the lidar serial port:");
            std::cin>>port;
        }
    }else {
        int id = 0;
        for(it = ports.begin(); it != ports.end(); it++) {
            printf("%d. %s\n", id, it->first.c_str());
            id++;
        }

        if(ports.empty()) {
            printf("Not Lidar was detected. Please enter the lidar serial port:");
            std::cin>>port;
        } else {
            while(ydlidar::ok()) {
                printf("Please select the lidar port:");
                std::string number;
                std::cin>>number;
                if((size_t)atoi(number.c_str()) >= ports.size()) {
                    continue;
                }
                it = ports.begin();
                id = atoi(number.c_str());
                while (id) {
                    id--;
                    it++;
                }
                port = it->second;
                break;
            }
        }
    }
    std::vector<unsigned int> baudrateList;
    baudrateList.push_back(115200);
    baudrateList.push_back(128000);
    baudrateList.push_back(153600);
    baudrateList.push_back(230400);
    baudrateList.push_back(500000);
    for( unsigned int i = 0; i < baudrateList.size(); i ++) {
        printf("%u. %u\n", i, baudrateList[i]);
    }
    while (ydlidar::ok()) {
        printf("Please enter the lidar serial baud rate:");
        std::string index;
        std::cin>>index;
        if(atoi(index.c_str()) >= baudrateList.size()) {
            printf("Invalid serial number, Please re-select\n");
            continue;
        }
        baudrate = baudrateList[atoi(index.c_str())];
        break;

    }


    int intensities  = 0;
    printf("0. false\n");
    printf("1. true\n");
    while (ydlidar::ok()) {
        printf("Please enter the lidar intensity:");
        std::cin>>intensity;
        if(atoi(intensity.c_str()) >= 2) {
            printf("Invalid serial number, Please re-select\n");
            continue;
        }

        intensities = atoi(intensity.c_str());
        break;
    }

    laser.setSerialPort(port);
    laser.setSerialBaudrate(baudrate);
    laser.setIntensities(intensities);//intensity
    laser.setFixedResolution(false);
    laser.setHeartBeat(false);//
    laser.setReversion(false); //
    laser.setAutoReconnect(true);//hot plug
    laser.setExposure(false);

    //unit: °C
    laser.setMaxAngle(180);
    laser.setMinAngle(-180);

    //unit: m
    laser.setMinRange(0.1);
    laser.setMaxRange(16.0);

    //unit: K
    laser.setSampleRate(5);

    //unit: Hz
    laser.setScanFrequency(8);

    std::vector<float> ignore_array;
    ignore_array.clear();
    laser.setIgnoreArray(ignore_array);

    laser.initialize();
    while(ydlidar::ok()){
		bool hardError;
		LaserScan scan;
		if(laser.doProcessSimple(scan, hardError )){
            fprintf(stdout,"Scan received: %u ranges\n",(unsigned int)scan.ranges.size());
            fflush(stdout);
        }else {
            fprintf(stderr,"get Scan Data failed\n");
            fflush(stderr);
        }
	}


    laser.turnOff();
    laser.disconnecting();

    return 0;


}
