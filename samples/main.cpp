
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

int main(int argc, char *argv[]) {
  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);
  std::string port;
  std::string baudrate;
  printf("Please enter the lidar serial port or IP:");
  std::cin >> port;
  printf("Please enter the lidar serial baud rate or network port:");
  std::cin >> baudrate;
  const int baud = atoi(baudrate.c_str());
  const int intensities =  0;

  regex reg("(\\d{1,3}).(\\d{1,3}).(\\d{1,3}).(\\d{1,3})");
  smatch m;

  uint8_t driver_type;

  if (regex_match(port, m, reg)) {
    driver_type = 1;
  } else {
    driver_type = 0;
  }


  ydlidar::init(argc, argv);
  laser.setSerialPort(port);
  laser.setSerialBaudrate(baud);
  laser.setDeviceType(driver_type);
  laser.setFixedResolution(false);
  laser.setHeartBeat(true);
  laser.setReversion(false);
  laser.setIntensities(intensities);
  laser.setScanFrequency(12);
  laser.setAutoReconnect(true);//异常是否重新连接
  laser.initialize();

  while (ydlidar::ok()) {
    bool hardError;
    LaserScan scan;

    if (laser.doProcessSimple(scan, hardError)) {
      fprintf(stdout, "Scan received: %u ranges\n", (unsigned int)scan.ranges.size());
      fflush(stdout);
    }
  }

  laser.turnOff();
  laser.disconnecting();

  return 0;


}
