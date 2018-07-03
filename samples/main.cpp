
#include "CYdLidar.h"
#include "timer.h"
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
    const int intensities = 0;

    signal(SIGINT, Stop);
    signal(SIGTERM, Stop);
    laser.setSerialPort(port);
    laser.setSerialBaudrate(baud);
    laser.setIntensities(intensities);
    laser.setMaxRange(16.0);
    laser.setMinRange(0.26);
    laser.setMaxAngle(180);
    laser.setMinAngle(-180);
    laser.setHeartBeat(false);
    laser.setReversion(false);
    laser.setFixedResolution(false);
    laser.setAutoReconnect(true);

    //雷达相对机器人安装位置
    pose_info laser_pose;
    laser_pose.x = 0;
    laser_pose.y = 0;
    laser_pose.phi = 0;
    laser.setSensorPose(laser_pose);

    laser.initialize();


    while(!running){
		bool hardError;
        LaserScan scan;//原始激光数据
        LaserScan syncscan;//同步后激光数据
        PointCloud pc;//同步后激光点云数据

        if(laser.doProcessSimple(scan, syncscan, pc, hardError )){
            for(int i =0; i < scan.ranges.size(); i++ ){
                float angle = scan.config.min_angle + i*scan.config.ang_increment;
                float dis = scan.ranges[i];

            }
            fprintf(stderr,"min_angle: %f \n",scan.config.min_angle);
            fprintf(stderr,"max_angle: %f \n",scan.config.max_angle);

			fprintf(stderr,"Scan received: %u ranges\n",(unsigned int)scan.ranges.size());

		}

        {//做imu和odometry数据输入
            odom_info odom;
            odom.x = 0;
            odom.y = 0;
            odom.phi = 0;
            odom.stamp = getTime();
            laser.setSyncOdometry(odom);


        }

	}
  laser.turnOff();
  laser.disconnecting();

  return 0;


}
