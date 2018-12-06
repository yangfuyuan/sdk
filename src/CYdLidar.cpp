#include "CYdLidar.h"
#include "common.h"
#include <map>


using namespace std;
using namespace ydlidar;
using namespace impl;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar(): lidarPtr(0) {
  m_SerialPort        = "";
  m_SerialBaudrate    = 115200;
  m_FixedResolution   = false;
  m_Reversion         = false;
  m_AutoReconnect     = false;
  m_MaxAngle          = 180.f;
  m_MinAngle          = -180.f;
  m_MaxRange          = 16.0;
  m_MinRange          = 0.08;
  isScanning          = false;
  isConnected         = false;
  node_counts         = 720;
  each_angle          = 0.5;
  m_IgnoreArray.clear();
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
    lidarPtr    = 0;
    isConnected = false;
  }
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan, bool &hardwareError) {
  hardwareError			= false;

  // Bound?
  if (!checkHardware()) {
    hardwareError = true;
    return false;
  }

  node_info nodes[2048];
  size_t   count = _countof(nodes);
  size_t all_nodes_counts = node_counts;

  //  wait Scan data:
  uint64_t tim_scan_start = getTime();
  result_t op_result =  lidarPtr->grabScanData(nodes, count);

  // Fill in scan data:
  if (IS_OK(op_result)) {
    op_result = lidarPtr->ascendScanData(nodes, count);
    uint64_t max_time = nodes[0].stamp ;
    uint64_t min_time = nodes[0].stamp;

    if (IS_OK(op_result)) {
      if (!m_FixedResolution) {
        all_nodes_counts = count;
      } else {
        all_nodes_counts = node_counts;
      }

      each_angle = 360.0 / all_nodes_counts;

      node_info *angle_compensate_nodes = new node_info[all_nodes_counts];
      memset(angle_compensate_nodes, 0, all_nodes_counts * sizeof(node_info));
      unsigned int i = 0;

      for (; i < count; i++) {
        if (nodes[i].distance_q2 != 0) {
          float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);

          if (m_Reversion) {
            angle = angle + 180;

            if (angle >= 360) {
              angle = angle - 360;
            }

            nodes[i].angle_q6_checkbit = ((uint16_t)(angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT;
          }

          int inter = (int)(angle / each_angle);
          float angle_pre = angle - inter * each_angle;
          float angle_next = (inter + 1) * each_angle - angle;

          if (angle_pre < angle_next) {
            if (inter < all_nodes_counts) {
              angle_compensate_nodes[inter] = nodes[i];
            }
          } else {
            if (inter < all_nodes_counts - 1) {
              angle_compensate_nodes[inter + 1] = nodes[i];
            }
          }
        }

        if (nodes[i].stamp > max_time) {
          max_time = nodes[i].stamp;
        }

        if (nodes[i].stamp < min_time) {
          min_time = nodes[i].stamp;
        }
      }

      LaserScan scan_msg;

      if (m_MaxAngle < m_MinAngle) {
        float temp = m_MinAngle;
        m_MinAngle = m_MaxAngle;
        m_MaxAngle = temp;
      }


      tim_scan_start = min_time;
      double scan_time = max_time - min_time;
      int counts = all_nodes_counts * ((m_MaxAngle - m_MinAngle) / 360.0f);
      int angle_start = 180 + m_MinAngle;
      int node_start = all_nodes_counts * (angle_start / 360.0f);

      scan_msg.ranges.resize(counts);
      scan_msg.intensities.resize(counts);
      float range = 0.0;
      float intensity = 0.0;
      int index = 0;


      for (size_t i = 0; i < all_nodes_counts; i++) {
        range = (float)angle_compensate_nodes[i].distance_q2 / 4000.f;
        intensity = (float)(angle_compensate_nodes[i].sync_quality);

        if (i < all_nodes_counts / 2) {
          index = all_nodes_counts / 2 - 1 - i;
        } else {
          index = all_nodes_counts - 1 - (i - all_nodes_counts / 2);
        }

        if (m_IgnoreArray.size() != 0) {
          float angle = (float)((angle_compensate_nodes[i].angle_q6_checkbit >>
                                 LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);

          if (angle > 180) {
            angle = 360 - angle;
          } else {
            angle = -angle;
          }

          for (uint16_t j = 0; j < m_IgnoreArray.size(); j = j + 2) {
            if ((m_IgnoreArray[j] < angle) && (angle <= m_IgnoreArray[j + 1])) {
              range = 0.0;
              break;
            }
          }
        }

        if (range > m_MaxRange || range < m_MinRange) {
          range = 0.0;
        }

        int pos = index - node_start ;

        if (0 <= pos && pos < counts) {
          scan_msg.ranges[pos] =  range;
          scan_msg.intensities[pos] = intensity;
        }
      }

      scan_msg.system_time_stamp = tim_scan_start;
      scan_msg.self_time_stamp = tim_scan_start;
      scan_msg.config.min_angle = DEG2RAD(m_MinAngle);
      scan_msg.config.max_angle = DEG2RAD(m_MaxAngle);
      scan_msg.config.ang_increment = (scan_msg.config.max_angle - scan_msg.config.min_angle) /
                                      (double)counts;
      scan_msg.config.time_increment = scan_time / (double)counts;
      scan_msg.config.scan_time = scan_time;
      scan_msg.config.min_range = m_MinRange;
      scan_msg.config.max_range = m_MaxRange;
      outscan = scan_msg;
      delete[] angle_compensate_nodes;
      return true;


    }

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
  bool ret = isScanning;

  if (!isScanning) {
    result_t ans = lidarPtr->startScan();
    ret = true;

    if (!IS_OK(ans)) {
      ret = false;
    } else {
      isScanning = true;
      printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
      fflush(stdout);
    }
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

/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs() {
  if (!lidarPtr) {
    // create the driver instance
    lidarPtr = new YDlidarDriver();

    if (!lidarPtr) {
      fprintf(stderr, "Create Driver fail\n");
      return false;

    }

    printf("[YDLIDAR INFO] Now YDLIDAR SDK VERSION: %s\n", YDlidarDriver::getSDKVersion().c_str());
    fflush(stdout);

  }

  if (isConnected) {
    return true;
  }

  // Is it COMX, X>4? ->  "\\.\COMX"
  if (m_SerialPort.size() >= 3) {
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

  if (!IS_OK(op_result)) {
    fprintf(stderr, "[CYdLidar] Error, cannot bind to the specified serial port[%s] and baudrate[%d]\n",
            m_SerialPort.c_str(), m_SerialBaudrate);
    return false;
  }

  printf("[YDLIDAR INFO] Connection established in %s[%d]:\n", m_SerialPort.c_str(),
         m_SerialBaudrate);
  fflush(stdout);

  isConnected = true;
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
      if (turnOn()) {
        ret = true;
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
    fflush(stderr);
    return false;
  }

  if (!turnOn()) {
    fprintf(stderr, "[CYdLidar::initialize] Error initializing YDLIDAR scanner.\n");
    fflush(stderr);
    ret = false;

  }

  return ret;

}
