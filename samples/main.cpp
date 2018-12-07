
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
  printf(" YDLIDAR C++ TEST\n");
  printf(" ydlidar_test calibration_filename\n");
  std::string port;
  std::string baudrate;
  std::string intensity;
  std::string calibration_filename = "LidarAngleCalibration.ini";

  if (argc > 1) {
    calibration_filename = (std::string)argv[1];
  }

  printf("lidar angle calibration file: %s\n", calibration_filename.c_str());
  printf("Please enter the lidar serial port:");
  std::cin >> port;
  printf("Please enter the lidar serial baud rate:");
  std::cin >> baudrate;
  printf("Please enter the lidar intensity:");
  std::cin >> intensity;

  int baud = atoi(baudrate.c_str());
  int intensities =  atoi(intensity.c_str());

  ydlidar::init(argc, argv);
  laser.setSerialPort(port);
  laser.setSerialBaudrate(baud);
  laser.setIntensities(intensities);//intensity
  laser.setFixedResolution(false);
  laser.setReversion(false); //
  laser.setAutoReconnect(true);//hot plug

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

  laser.setCalibrationFileName(calibration_filename);//Zero angle offset filename

  std::vector<float> ignore_array;
  ignore_array.clear();
  laser.setIgnoreArray(ignore_array);

  laser.initialize();

  while (ydlidar::ok()) {
    bool hardError;
    LaserScan scan;

    if (laser.doProcessSimple(scan, hardError)) {
      fprintf(stdout, "Scan received: %u ranges\n", (unsigned int)scan.ranges.size());
      fflush(stdout);
    } else {
      fprintf(stderr, "get Scan Data failed\n");
      fflush(stderr);
    }
  }


  laser.turnOff();
  laser.disconnecting();

  return 0;


}
