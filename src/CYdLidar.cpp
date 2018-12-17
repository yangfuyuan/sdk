#include "CYdLidar.h"
#include "common.h"
#include <map>



using namespace std;
using namespace ydlidar;
using namespace impl;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar() : lidarPtr(nullptr) {
  m_SerialPort        = "";
  m_SerialBaudrate    = 115200;
  m_FixedResolution   = false;
  m_Reversion         = false;
  m_AutoReconnect     = true;
  m_MaxAngle          = 180.f;
  m_MinAngle          = -180.f;
  m_MaxRange          = 64.0;
  m_MinRange          = 0.08;
  isScanning          = false;
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
    lidarPtr = nullptr;
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

  node_info nodes[3600];
  size_t   count = _countof(nodes);

  size_t all_nodes_counts = node_counts;

  //  wait Scan data:
  uint64_t tim_scan_start = getTime();
  result_t op_result =  lidarPtr->grabScanData(nodes, count);
  uint64_t tim_scan_end = getTime();

  // Fill in scan data:
  if (IS_OK(op_result)) {
    op_result = lidarPtr->ascendScanData(nodes, count);
    //同步后的时间
    tim_scan_start = nodes[0].stamp;
    tim_scan_end   = nodes[0].stamp;

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
        if (nodes[i].distance_q != 0) {
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

        if (tim_scan_start > nodes[i].stamp) {
          tim_scan_start = nodes[i].stamp;
        }

        if (tim_scan_end < nodes[i].stamp) {
          tim_scan_end = nodes[i].stamp;
        }

      }

      LaserScan scan_msg;

      if (m_MaxAngle < m_MinAngle) {
        float temp = m_MinAngle;
        m_MinAngle = m_MaxAngle;
        m_MaxAngle = temp;
      }


      double scan_time = tim_scan_end - tim_scan_start;
      int counts = all_nodes_counts * ((m_MaxAngle - m_MinAngle) / 360.0f);
      int angle_start = 180 + m_MinAngle;
      int node_start = all_nodes_counts * (angle_start / 360.0f);

      scan_msg.ranges.resize(counts);
      scan_msg.intensities.resize(counts);
      float range = 0.0;
      float intensity = 0.0;
      int index = 0;


      for (size_t i = 0; i < all_nodes_counts; i++) {
        range = (float)angle_compensate_nodes[i].distance_q;
        intensity = (float)(angle_compensate_nodes[i].sync_quality >> LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

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

      if ((scan_msg.config.max_angle - scan_msg.config.min_angle) == 2 * M_PI) {
        scan_msg.config.ang_increment = (scan_msg.config.max_angle - scan_msg.config.min_angle) /
                                        (double)counts;
        scan_msg.config.time_increment = scan_time / (double)counts;
      } else {
        scan_msg.config.ang_increment = (scan_msg.config.max_angle - scan_msg.config.min_angle) /
                                        (double)(counts - 1);
        scan_msg.config.time_increment = scan_time / (double)(counts - 1);
      }

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

/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs() {
  if (!lidarPtr) {
    // create the driver instance
    lidarPtr = new YDlidarDriver();

    if (!lidarPtr) {
      ydlidar::console.error("Create Driver fail");
      return false;
    }
  }

  if (lidarPtr->isconnected()) {
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
    ydlidar::console.error("[CYdLidar] Error, cannot bind to the specified serial port %s",
                           m_SerialPort.c_str());
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

  // start scan...
  result_t s_result = lidarPtr->startScan();

  if (!IS_OK(s_result)) {
    s_result = lidarPtr->startScan();

    if (!IS_OK(s_result)) {
      ydlidar::console.error("[CYdLidar] Error starting scanning mode: %x", s_result);
      isScanning = false;
      return false;
    }
  }

  lidarPtr->setAutoReconnect(m_AutoReconnect);
  ydlidar::console.message("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
  isScanning = true;
  return true;

}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware() {
  bool ret = true;

  if (!isScanning) {
    ret = checkCOMMs();

    if (ret && (ret = checkStatus())) {
      if (ret) {
        ret = turnOn();
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
    ydlidar::console.error("[CYdLidar::initialize] Error initializing YDLIDAR scanner.");
    return false;
  }

  if (!checkStatus()) {
    ydlidar::console.warning("[CYdLidar::initialize] Error initializing YDLIDAR scanner.because of failure in scan mode.");
  }

  if (!turnOn()) {
    ydlidar::console.warning("[CYdLidar::initialize] Error initializing YDLIDAR scanner. Because the motor falied to start.");

  }

  return ret;

}
