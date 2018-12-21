#include <serial/common.h>
#include <map>
#include "CYdLidar.h"
#include <timer.h>


using namespace std;
using namespace ydlidar;
using namespace impl;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar(): lidarPtr(0) {
  m_SerialPort = "";
  m_SerialBaudrate = 115200;
  m_Intensities = false;
  m_Exposure = false;
  m_HeartBeat = true;
  m_AutoReconnect = true;
  m_DeviceType    = DEVICE_DRIVER_TYPE_SERIALPORT;
  m_Max_x = 1920;
  m_Min_x = 0;
  m_Max_y = 1080;
  m_Min_y = 0;
  m_pose.x = 960;
  m_pose.y = -260;
  m_pose.theta = -90;
  m_SampleRate = 9;
  m_ScanFrequency = 7;
  isScanning = false;
  show_error = 0;
}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar() {
  disconnecting();
}

void CYdLidar::disconnecting() {
  if (lidarPtr) {
    lidarPtr->disconnect();
    delete lidarPtr;
    lidarPtr = 0;
  }
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(std::vector<touch_info> &outPoints, bool &hardwareError) {
  hardwareError			= false;

  // Bound?
  if (!checkHardware()) {
    hardwareError = true;
    return false;
  }

  touch_info nodes[2048];
  size_t   count = _countof(nodes);
  //  wait Scan data:
  result_t op_result = lidarPtr->grabScanData(nodes, count);
  outPoints.clear();

  // Fill in scan data:
  if (op_result == RESULT_OK) {
    unsigned int i = 0;

    for (; i < count; i++) {
      outPoints.push_back(nodes[i]);
    }

    return true;

  } else {
    if (op_result == RESULT_FAIL) {
      // Error? Retry connection
      //this->disconnect();
    }
  }

  return false;

}


/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn() {
  bool ret = false;

  if (isScanning) {
    lidarPtr->startMotor();
    ret = true;
  }

  return ret;
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff() {
  if (lidarPtr) {
    lidarPtr->stop();
    lidarPtr->stopMotor();
    isScanning = false;
  }

  return true;
}

/** Returns true if the device is connected & operative */
bool CYdLidar::getDeviceHealth() const {
  if (!lidarPtr) {
    return false;
  }

  result_t op_result;
  device_health healthinfo;

  op_result = lidarPtr->getHealth(healthinfo);

  if (op_result == RESULT_OK) {
    printf("Yd Lidar running correctly ! The health status: %s\n",
           (int)healthinfo.status == 0 ? "good" : "bad");

    if (healthinfo.status == 2) {
      if (show_error == 3) {
        fprintf(stderr, "Error, Yd Lidar internal error detected. Please reboot the device to retry.\n");
      }

      return false;
    } else {
      return true;
    }

  } else {
    if (show_error == 3) {
      fprintf(stderr, "Error, cannot retrieve Yd Lidar health code: %x\n", op_result);
    }

    return false;
  }

}

bool CYdLidar::getDeviceInfo(int &type) {

  if (!lidarPtr) {
    return false;
  }

  device_info devinfo;

  if (lidarPtr->getDeviceInfo(devinfo) != RESULT_OK) {
    if (show_error == 3) {
      fprintf(stderr, "get DeviceInfo Error\n");
    }

    return false;
  }

  std::string model;
  sampling_rate _rate;
  int _samp_rate = 4;
  result_t ans;
  int bad = 0;

  type = devinfo.model;

  switch (devinfo.model) {
  case 1:
    model = "F4";
    break;

  case 2:
    model = "T1";
    break;

  case 3:
    model = "F2";
    break;

  case 4:
    model = "S4";
    break;

  case 5: {
    model = "G4";
    ans = lidarPtr->getSamplingRate(_rate);

    if (ans == RESULT_OK) {
      switch (m_SampleRate) {
      case 4:
        _samp_rate = 0;
        break;

      case 8:
        _samp_rate = 1;
        break;

      case 9:
        _samp_rate = 2;
        break;

      default:
        _samp_rate = _rate.rate;
        break;
      }

      while (_samp_rate != _rate.rate) {
        ans = lidarPtr->setSamplingRate(_rate);

        if (ans != RESULT_OK) {
          bad++;

          if (bad > 5) {
            break;
          }
        }
      }

      switch (_rate.rate) {
      case 0:
        _samp_rate = 4;
        break;

      case 1:
        _samp_rate = 8;
        break;

      case 2:
        _samp_rate = 9;
        break;
      }


    }

  }
  break;

  case 6:
    model = "X4";

  case 8: {
    model = "F4Pro";
    ans = lidarPtr->getSamplingRate(_rate);

    if (ans == RESULT_OK) {
      switch (m_SampleRate) {
      case 4:
        _samp_rate = 0;
        break;

      case 6:
        _samp_rate = 1;
        break;

      default:
        _samp_rate = _rate.rate;
        break;
      }

      while (_samp_rate != _rate.rate) {
        ans = lidarPtr->setSamplingRate(_rate);

        if (ans != RESULT_OK) {
          bad++;

          if (bad > 5) {
            break;
          }
        }
      }

      switch (_rate.rate) {
      case 0:
        _samp_rate = 4;
        break;

      case 1:
        _samp_rate = 6;
        break;
      }

    }

  }
  break;

  case 9:
    model = "G4C";
    break;

  default:
    model = "Unknown";
    break;
  }

  m_SampleRate = _samp_rate;



  unsigned int maxv = (unsigned int)(devinfo.firmware_version >> 8);
  unsigned int midv = (unsigned int)(devinfo.firmware_version & 0xff) / 10;
  unsigned int minv = (unsigned int)(devinfo.firmware_version & 0xff) % 10;

  printf("[YDLIDAR] Connection established in [%s]:\n"
         "Firmware version: %u.%u.%u\n"
         "Hardware version: %u\n"
         "Model: %s\n"
         "Serial: ",
         m_SerialPort.c_str(),
         maxv,
         midv,
         minv,
         (unsigned int)devinfo.hardware_version,
         model.c_str());

  for (int i = 0; i < 16; i++) {
    printf("%01X", devinfo.serialnum[i] & 0xff);
  }

  printf("\n");

  printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n", _samp_rate);


  float freq = 7.0f;

  if (devinfo.model == 5 || devinfo.model == 8 || devinfo.model == 9) {
    checkScanFrequency();
    checkHeartBeat();
  } else {
    printf("[YDLIDAR INFO] Current Scan Frequency : %fHz\n", freq);
  }

  return true;


}

/*-------------------------------------------------------------
                        checkScanFrequency
-------------------------------------------------------------*/
bool CYdLidar::checkScanFrequency() {
  float freq = 7.0f;
  scan_frequency _scan_frequency;
  int hz = 0;

  if (5 <= m_ScanFrequency && m_ScanFrequency <= 12) {
    result_t ans = lidarPtr->getScanFrequency(_scan_frequency) ;

    if (ans == RESULT_OK) {
      freq = _scan_frequency.frequency / 100.f;
      hz = m_ScanFrequency - freq;

      if (hz > 0) {
        while (hz != 0) {
          lidarPtr->setScanFrequencyAdd(_scan_frequency);
          hz--;
        }

        freq = _scan_frequency.frequency / 100.0f;
      } else {
        while (hz != 0) {
          lidarPtr->setScanFrequencyDis(_scan_frequency);
          hz++;
        }

        freq = _scan_frequency.frequency / 100.0f;
      }
    }

    if (fabs(m_ScanFrequency - freq) < 1.0) {
      hz = (m_ScanFrequency - freq) * 10;

      if (hz > 0) {
        while (hz != 0) {
          lidarPtr->setScanFrequencyAddMic(_scan_frequency);
          hz--;
        }

        freq = _scan_frequency.frequency / 100.0f;
      } else {
        while (hz != 0) {
          lidarPtr->setScanFrequencyDisMic(_scan_frequency);
          hz++;
        }

        freq = _scan_frequency.frequency / 100.0f;
      }
    }
  }

  printf("[YDLIDAR INFO] Current Scan Frequency : %fHz\n", freq);

  return true;

}

/*-------------------------------------------------------------
                        checkHeartBeat
-------------------------------------------------------------*/

bool CYdLidar::checkHeartBeat() const {
  bool ret = false;
  scan_heart_beat beat;

  if (m_HeartBeat) {
    while (m_HeartBeat) {
      result_t ans = lidarPtr->setScanHeartbeat(beat);

      if (ans == RESULT_OK) {
        if (beat.enable) {
          ans = lidarPtr->setScanHeartbeat(beat);

          if (ans == RESULT_OK) {
            if (!beat.enable) {
              lidarPtr->setHeartBeat(true);
              ret = true;
              return ret;
            }
          }
        } else  {
          lidarPtr->setHeartBeat(true);
          ret = true;
          return ret;
        }
      }
    }
  } else {
    lidarPtr->setHeartBeat(false);
    ret = true;
  }

  return ret;

}
/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs() {
  if (!lidarPtr) {
    // create the driver instance
    lidarPtr = new YDlidarDriver(uint8_t(m_DeviceType));

    if (!lidarPtr) {
      fprintf(stderr, "Create Driver fail\n");
      return false;

    }

  }

  if (lidarPtr->isconnected()) {
    return true;
  }

  // Is it COMX, X>4? ->  "\\.\COMX"
  if (m_DeviceType == DEVICE_DRIVER_TYPE_SERIALPORT && m_SerialPort.size() >= 3) {
    if (tolower(m_SerialPort[0]) == 'c' && tolower(m_SerialPort[1]) == 'o' &&
        tolower(m_SerialPort[2]) == 'm') {
      // Need to add "\\.\"?
      if (m_SerialPort.size() > 4 || m_SerialPort[3] > '4') {
        m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
      }
    }
  }

  // make connection...
  result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);

  if (op_result != RESULT_OK) {
    if (m_DeviceType == DEVICE_DRIVER_TYPE_SERIALPORT) {
      fprintf(stderr, "[CYdLidar] Error, cannot bind to the specified serial port[%s] and baudrate[%d]\n",
              m_SerialPort.c_str(), m_SerialBaudrate);
    } else {
      fprintf(stderr, "[CYdLidar] Error, cannot bind to the specified ip address[%s] and port[%d]\n",
              m_SerialPort.c_str(), m_SerialBaudrate);
    }

    return false;
  }

  return true;
}

/*-------------------------------------------------------------
                        checkStatus
-------------------------------------------------------------*/
bool CYdLidar::checkStatus() {

  if (!lidarPtr) {
    return false;
  }

  if (lidarPtr->isscanning()) {
    return true;
  }

  std::map<int, bool> checkmodel;
  checkmodel.insert(std::map<int, bool>::value_type(115200, false));
  checkmodel.insert(std::map<int, bool>::value_type(128000, false));
  checkmodel.insert(std::map<int, bool>::value_type(153600, false));
  checkmodel.insert(std::map<int, bool>::value_type(230400, false));

again:
  // check health:
  bool ret = getDeviceHealth();

  int m_type;

  if ((!ret || !getDeviceInfo(m_type)) && m_DeviceType == DEVICE_DRIVER_TYPE_SERIALPORT) {
    checkmodel[m_SerialBaudrate] = true;
    map<int, bool>::iterator it;

    for (it = checkmodel.begin(); it != checkmodel.end(); ++it) {
      if (it->second) {
        continue;
      }

      show_error++;
      lidarPtr->disconnect();
      delete lidarPtr;
      lidarPtr = 0;
      lidarPtr = new YDlidarDriver;

      if (!lidarPtr) {
        printf("YDLIDAR Create Driver fail, exit\n");
        return false;
      }

      m_SerialBaudrate = it->first;

      bool ret = checkCOMMs();

      if (!ret) {
        return false;
      }

      goto again;
    }

    return false;
  }

  show_error = 0;
  m_Intensities = false;

  if (m_type == 4) {
    if (m_SerialBaudrate == 153600 && m_DeviceType == DEVICE_DRIVER_TYPE_SERIALPORT) {
      m_Intensities = true;
    }

    if (m_Intensities) {
      scan_exposure exposure;
      int cnt = 0;

      while ((lidarPtr->setLowExposure(exposure) == RESULT_OK) && (cnt < 3)) {
        if (exposure.exposure != m_Exposure) {
          printf("set EXPOSURE MODEL SUCCESS!!!\n");
          break;
        }

        cnt++;
      }

      if (cnt >= 3) {
        fprintf(stderr, "set LOW EXPOSURE MODEL FALIED!!!\n");
      }
    }
  }

  lidarPtr->setIntensities(m_Intensities);

  lidarPtr->setScreenBox(m_Max_x, m_Max_y, m_Min_x, m_Min_y);
  lidarPtr->setLaserPose(m_pose);

  // start scan...
  result_t s_result = lidarPtr->startScan();

  if (s_result != RESULT_OK) {
    fprintf(stderr, "[CYdLidar] Error starting scanning mode: %x\n", s_result);
    isScanning = false;
    return false;
  }

  lidarPtr->setAutoReconnect(m_AutoReconnect);
  printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
  fflush(stdout);
  fflush(stderr);
  isScanning = true;
  return true;

}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware() {
  bool ret = true;

  if (!isScanning) {
    ret = false;

    if (checkCOMMs()) {
      if (checkStatus()) {
        if (turnOn()) {
          ret = true;
        }
      }
    }
  }

  return ret;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize() {
  bool ret = true;

  if (!checkCOMMs()) {
    fprintf(stderr, "[CYdLidar::initialize] Error initializing YDLIDAR scanner.\n");
    return false;
  }

  if (!checkStatus()) {
    fprintf(stderr,
            "[CYdLidar::initialize] Error initializing YDLIDAR scanner.because of failure in scan mode.\n");
  }

  if (!turnOn()) {
    fprintf(stderr,
            "[CYdLidar::initialize] Error initializing YDLIDAR scanner. Because the motor falied to start.\n");

  }

  return ret;

}
