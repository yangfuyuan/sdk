
#include "CYdLidar.h"
#include <iostream>
#include <string>
using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif

int main(int argc, char *argv[]) {
  printf("__   ______  _     ___ ____    _    ____      ____ ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\    / ___/ ___| \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) |___\\___ \\___ \\ \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <_____|__) |__) | \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\   |____/____/  \n");
  printf("\n");
  fflush(stdout);
  std::string port;
  std::string calibration_filename = "LidarAngleCalibration.ini";

  if (argc > 1) {
    calibration_filename = (std::string)argv[1];
  }
  ydlidar::init(argc, argv);

  printf("lidar angle calibration file: %s\n", calibration_filename.c_str());
  std::map<std::string, std::string> ports =  ydlidar::YDlidarDriver::lidarPortList();
  std::map<std::string, std::string>::iterator it;

  if (ports.size() == 1) {
    it = ports.begin();
    printf("Lidar[%s] detected, whether to select current radar(yes/no)?:",
                          it->first.c_str());
    std::string ok;
    std::cin >> ok;

    for (size_t i = 0; i < ok.size(); i++) {
      ok[i] = tolower(ok[i]);
    }

    if (ok.find("yes") != std::string::npos || atoi(ok.c_str()) == 1) {
      port = it->second;
    } else {
      printf("Please enter the lidar serial port:");
      std::cin >> port;
    }
  } else {
    int id = 0;

    for (it = ports.begin(); it != ports.end(); it++) {
      printf("%d. %s\n", id, it->first.c_str());
      id++;
    }

    if (ports.empty()) {
      printf("Not Lidar was detected. Please enter the lidar serial port:");
      std::cin >> port;
    } else {
      while (ydlidar::ok()) {
        printf("Please select the lidar port:");
        std::string number;
        std::cin >> number;

        if ((size_t)atoi(number.c_str()) >= ports.size()) {
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

  std::string input_frequency;
  float frequency = 8.0;
  while (ydlidar::ok()) {
    printf("Please enter the lidar scan frequency[5-12]:");
    std::cin >> input_frequency;
    frequency = atof(input_frequency.c_str());
    if(frequency <=12.0 && frequency >= 5.0 ) {
       break;
    }
    fprintf(stderr, "Invalid scan frequency,The scanning frequency range is 5 to 12 HZ, Please re-enter.\n");
  }

  if(!ydlidar::ok()) {
    return 0;
  }
  CYdLidar laser;
  laser.setSerialPort(port);
  laser.setSerialBaudrate(230400);
  laser.setIntensities(false);//intensity
  laser.setFixedResolution(false);
  laser.setReversion(false); //
  laser.setAutoReconnect(true);//hot plug

  //unit: °
  laser.setMaxAngle(180);
  laser.setMinAngle(-180);

  //unit: m
  laser.setMinRange(0.1);
  laser.setMaxRange(16.0);

  //unit: K
  laser.setSampleRate(5);

  //unit: Hz
  laser.setScanFrequency(frequency);

  laser.setCalibrationFileName(calibration_filename);//Zero angle offset filename

  std::vector<float> ignore_array;
  ignore_array.clear();
  laser.setIgnoreArray(ignore_array);

  bool ret = laser.initialize();

  while (ret && ydlidar::ok()) {
    bool hardError;
    LaserScan scan;

    if (laser.doProcessSimple(scan, hardError)) {
      fprintf(stdout, "Scan received[%llu]: %u ranges is [%f]Hz\n", scan.self_time_stamp,
              (unsigned int)scan.ranges.size(), 1.0 / scan.config.scan_time);
      fflush(stdout);
    } else {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }
  }

  laser.turnOff();
  laser.disconnecting();

  return 0;
}
