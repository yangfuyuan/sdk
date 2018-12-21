/*
*  YDLIDAR SYSTEM
*  YDLIDAR DRIVER
*
*  Copyright 2015 - 2018 EAI TEAM
*  http://www.eaibot.com
*
*/
#include <serial/common.h>
#include "ydlidar_driver.h"
#include <math.h>
#include <timer.h>
using namespace impl;

namespace ydlidar {

YDlidarDriver::YDlidarDriver(uint8_t drivertype):
  _serial(0), m_driver_type(drivertype) {
  isConnected = false;
  isScanning = false;
  //串口配置参数
  m_intensities = false;
  isHeartbeat = true;
  isAutoReconnect = true;
  isAutoconnting = false;

  _baudrate = 115200;
  isSupportMotorCtrl = true;
  _sampling_rate = -1;
  model = -1;
  touch_point_count = 0;

  //解析参数
  PackageSampleBytes = 2;
  package_Sample_Index = 0;
  IntervalSampleAngle = 0.0;
  IntervalSampleAngle_LastPackage = 0.0;
  FirstSampleAngle = 0;
  LastSampleAngle = 0;
  CheckSun = 0;
  CheckSunCal = 0;
  SampleNumlAndCTCal = 0;
  LastSampleAngleCal = 0;
  CheckSunResult = true;
  Valu8Tou16 = 0;
}

YDlidarDriver::~YDlidarDriver() {
  {
    isScanning = false;
  }

  isAutoReconnect = false;
  _thread.join();

  ScopedLocker lk(_serial_lock);

  if (_serial) {
    if (_serial->isOpen()) {
      _serial->closePort();
    }
  }

  if (_serial) {
    delete _serial;
    _serial = NULL;
  }
}

result_t YDlidarDriver::connect(const char *port_path, uint32_t baudrate) {
  _baudrate = baudrate;
  serial_port = string(port_path);

  if (m_driver_type != DEVICE_DRIVER_TYPE_SERIALPORT && m_driver_type != DEVICE_DRIVER_TYPE_TCP) {
    m_driver_type = DEVICE_DRIVER_TYPE_SERIALPORT;
  }

  {
    ScopedLocker lk(_serial_lock);

    if (!_serial) {
      switch (m_driver_type) {
      case DEVICE_DRIVER_TYPE_SERIALPORT:
        _serial = new serial::Serial(port_path, _baudrate, serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));
        break;

      case DEVICE_DRIVER_TYPE_TCP:
        _serial = new CActiveSocket();

      default:
        break;
      }

      _serial->bindport(port_path, baudrate);
    }

  }

  {
    ScopedLocker l(_lock);

    if (!_serial->open()) {
      return RESULT_FAIL;
    }
  }



  isConnected = true;

  {
    ScopedLocker l(_lock);
    sendCommand(LIDAR_CMD_FORCE_STOP);
    sendCommand(LIDAR_CMD_STOP);
  }
  _serial->flush();
  clearDTR();

  return RESULT_OK;
}


void YDlidarDriver::setDTR() {
  if (!isConnected) {
    return ;
  }

  ScopedLocker l(_serial_lock);

  if (_serial) {
    _serial->setDTR(1);
  }

}

void YDlidarDriver::clearDTR() {
  if (!isConnected) {
    return ;
  }

  ScopedLocker l(_serial_lock);

  if (_serial) {
    _serial->setDTR(0);
  }
}

result_t YDlidarDriver::startMotor() {
  ScopedLocker l(_lock);

  if (isSupportMotorCtrl) {
    setDTR();
    delay(500);
  } else {
    clearDTR();
    delay(500);
  }

  return RESULT_OK;
}

result_t YDlidarDriver::stopMotor() {
  ScopedLocker l(_lock);

  if (isSupportMotorCtrl) {
    clearDTR();
    delay(500);
  } else {
    setDTR();
    delay(500);
  }

  return RESULT_OK;
}

void YDlidarDriver::disconnect() {
  isAutoReconnect = false;

  if (!isConnected) {
    return ;
  }

  stop();

  ScopedLocker l(_serial_lock);

  if (_serial) {
    if (_serial->isOpen()) {
      _serial->closePort();
    }
  }

  isConnected = false;
}


void YDlidarDriver::disableDataGrabbing() {
  {
    isScanning = false;
  }
  _thread.join();
}

const bool YDlidarDriver::isscanning() const {
  return isScanning;
}
const bool YDlidarDriver::isconnected() const {
  return isConnected;
}

void YDlidarDriver::setScreenBox(const float &max_x, const float &max_y, const float &min_x,
                                 const float &min_y) {
  ScopedLocker l(_plock);

  if (max_x > min_x) {
    screen_max_x = max_x;
    screen_min_x = min_x;
  } else {
    screen_max_x = min_x;
    screen_min_x = max_x;
  }

  if (max_y > min_y) {
    screen_max_y = max_y;
    screen_min_y = min_y;
  } else {
    screen_max_y = min_y;
    screen_min_y = max_y;
  }
}

void YDlidarDriver::setLaserPose(const LaserPose &pose) {
  ScopedLocker l(_plock);
  laser_pose = pose;
}

bool YDlidarDriver::inBox(const float &x, const float &y) {
  return (x < screen_max_x && x > screen_min_x && y < screen_max_y && y > screen_min_y);
}



result_t YDlidarDriver::sendCommand(uint8_t cmd, const void *payload, size_t payloadsize) {
  uint8_t pkt_header[10];
  cmd_packet *header = reinterpret_cast<cmd_packet * >(pkt_header);
  uint8_t checksum = 0;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  if (payloadsize && payload) {
    cmd |= LIDAR_CMDFLAG_HAS_PAYLOAD;
  }

  header->syncByte = LIDAR_CMD_SYNC_BYTE;
  header->cmd_flag = cmd;
  sendData(pkt_header, 2) ;

  if ((cmd & LIDAR_CMDFLAG_HAS_PAYLOAD) && payloadsize && payload) {
    checksum ^= LIDAR_CMD_SYNC_BYTE;
    checksum ^= cmd;
    checksum ^= (payloadsize & 0xFF);

    for (size_t pos = 0; pos < payloadsize; ++pos) {
      checksum ^= ((uint8_t *)payload)[pos];
    }

    uint8_t sizebyte = (uint8_t)(payloadsize);
    sendData(&sizebyte, 1);

    sendData((const uint8_t *)payload, sizebyte);

    sendData(&checksum, 1);
  }

  return RESULT_OK;
}

result_t YDlidarDriver::sendData(const uint8_t *data, size_t size) {
  if (!isConnected) {
    return RESULT_FAIL;
  }

  if (data == NULL || size == 0) {
    return RESULT_FAIL;
  }

  size_t r;

  while (size) {
    r = _serial->writeData(data, size);

    if (r < 1) {
      return RESULT_FAIL;
    }

    size -= r;
    data += r;
  }

  return RESULT_OK;
}

result_t YDlidarDriver::getData(uint8_t *data, size_t size) {
  if (!isConnected) {
    return RESULT_FAIL;
  }

  size_t r;

  while (size) {
    r = _serial->readData(data, size);

    if (r < 1) {
      return RESULT_FAIL;
    }

    size -= r;
    data += r;

  }

  return RESULT_OK;

}

result_t YDlidarDriver::waitResponseHeader(lidar_ans_header *header, uint32_t timeout) {
  int  recvPos = 0;
  uint32_t startTs = getms();
  uint8_t  recvBuffer[sizeof(lidar_ans_header)];
  uint8_t  *headerBuffer = reinterpret_cast<uint8_t *>(header);
  uint32_t waitTime;

  while ((waitTime = getms() - startTs) <= timeout) {
    size_t remainSize = sizeof(lidar_ans_header) - recvPos;
    size_t recvSize;

    result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);

    if (ans != RESULT_OK) {
      return ans;
    }

    if (recvSize > remainSize) {
      recvSize = remainSize;
    }

    ans = getData(recvBuffer, recvSize);

    if (ans == RESULT_FAIL) {
      return RESULT_FAIL;
    }

    for (size_t pos = 0; pos < recvSize; ++pos) {
      uint8_t currentByte = recvBuffer[pos];

      switch (recvPos) {
      case 0:
        if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
          continue;
        }

        break;

      case 1:
        if (currentByte != LIDAR_ANS_SYNC_BYTE2) {
          recvPos = 0;
          continue;
        }

        break;
      }

      headerBuffer[recvPos++] = currentByte;

      if (recvPos == sizeof(lidar_ans_header)) {
        return RESULT_OK;
      }
    }
  }

  return RESULT_FAIL;
}

result_t YDlidarDriver::waitForData(size_t data_count, uint32_t timeout, size_t *returned_size) {
  size_t length = 0;

  if (returned_size == NULL) {
    returned_size = (size_t *)&length;
  }

  return (result_t)_serial->waitfordata(data_count, timeout, returned_size);
}

int YDlidarDriver::cacheScanData() {
  touch_info      local_point_buf[128];
  size_t         count = 128;
  touch_info     local_point[MAX_SCAN_NODES];
  size_t         point_count = 0;
  result_t            ans;
  memset(local_point, 0, sizeof(local_point));
  waitScanData(local_point_buf, count);

  uint32_t start_ts = getms();
  uint32_t end_ts = start_ts;
  int timeout_count = 0;

  while (isScanning) {
    if ((ans = waitScanData(local_point_buf, count)) != RESULT_OK) {
      if (ans != RESULT_TIMEOUT || timeout_count > DEFAULT_TIMEOUT_COUNT) {
        if (!isAutoReconnect) { //不重新连接, 退出线程
          fprintf(stderr, "exit scanning thread!!\n");
          {
            isScanning = false;
          }
          return RESULT_FAIL;
        } else {//做异常处理, 重新连接
          isAutoconnting = true;

          while (isAutoReconnect && isAutoconnting) {
            {
              ScopedLocker l(_serial_lock);

              if (_serial) {
                if (_serial->isOpen()) {
                  _serial->closePort();

                }

                delete _serial;
                _serial = NULL;
                isConnected = false;
              }
            }

            while (isAutoReconnect && connect(serial_port.c_str(), _baudrate) != RESULT_OK) {
              delay(500);
              uint64_t time = getTime();
              fprintf(stderr, "[%lu]: wait %s is available\n", time, serial_port.c_str());
              fflush(stderr);
            }

            if (!isAutoReconnect) {
              isScanning = false;
              return RESULT_FAIL;
            }

            if (isconnected()) {
              {
                ScopedLocker lk(_serial_lock);
                ans = startAutoScan();


              }

              if (ans == RESULT_OK) {
                timeout_count = 0;
                isAutoconnting = false;
                continue;
              } else {
                uint64_t time = getTime();
                fprintf(stderr, "[%lu]:Failed to start lidar scan\n", time);
                fflush(stderr);
                delay(500);
              }

            }
          }
        }


      } else {
        timeout_count++;
      }
    } else {
      timeout_count = 0;
    }


    for (size_t pos = 0; pos < count; ++pos) {
      if (local_point_buf[pos].new_frame) {
        if (local_point[0].new_frame) {
          _lock.lock();//timeout lock, wait resource copy
          memcpy(touch_point_buf, local_point, point_count * sizeof(touch_info));
          touch_point_count = point_count;
          _dataEvent.set();
          _lock.unlock();
        }

        point_count = 0;
      }

      local_point[point_count++] = local_point_buf[pos];

      if (point_count == _countof(local_point)) {
        point_count -= 1;
      }
    }


    //heartbeat function
    if (isHeartbeat) {
      end_ts = getms();

      if (end_ts - start_ts > DEFAULT_HEART_BEAT) {
        sendHeartBeat();
        start_ts = end_ts;
      }
    }
  }

  {
    isScanning = false;
  }

  return RESULT_OK;
}

result_t YDlidarDriver::waitPackage(touch_info *point, uint32_t timeout) {
  int recvPos = 0;
  node_info  node;
  uint32_t startTs = getms();
  uint32_t size = (m_intensities) ? sizeof(node_package) : sizeof(node_packages);
  uint8_t *recvBuffer = new uint8_t[size];

  uint32_t waitTime = 0;
  uint8_t *packageBuffer = (m_intensities) ? (uint8_t *)&package.package_Head :
                           (uint8_t *)&packages.package_Head;
  uint8_t  package_Sample_Num = 0;
  int32_t AngleCorrectForDistance;
  int  package_recvPos = 0;
  (*point).new_frame = false;

  if (package_Sample_Index == 0) {
    recvPos = 0;

    while ((waitTime = getms() - startTs) <= timeout) {
      size_t remainSize = PackagePaidBytes - recvPos;
      size_t recvSize;
      result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);

      if (ans != RESULT_OK) {
        delete[] recvBuffer;
        return ans;
      }

      if (recvSize > remainSize) {
        recvSize = remainSize;
      }

      getData(recvBuffer, recvSize);

      for (size_t pos = 0; pos < recvSize; ++pos) {
        uint8_t currentByte = recvBuffer[pos];

        switch (recvPos) {
        case 0:
          if (currentByte == (PH & 0xFF)) {

          } else {
            continue;
          }

          break;

        case 1:
          CheckSunCal = PH;

          if (currentByte == (PH >> 8)) {

          } else {
            recvPos = 0;
            continue;
          }

          break;

        case 2:
          SampleNumlAndCTCal = currentByte;

          if ((currentByte == CT_Normal) || (currentByte == CT_RingStart)) {

          } else {
            recvPos = 0;
            continue;
          }

          break;

        case 3:
          SampleNumlAndCTCal += (currentByte * 0x100);
          package_Sample_Num = currentByte;
          break;

        case 4:
          if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
            FirstSampleAngle = currentByte;
          } else {
            recvPos = 0;
            continue;
          }

          break;

        case 5:
          FirstSampleAngle += currentByte * 0x100;
          CheckSunCal ^= FirstSampleAngle;
          FirstSampleAngle = FirstSampleAngle >> 1;
          break;

        case 6:
          if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
            LastSampleAngle = currentByte;
          } else {
            recvPos = 0;
            continue;
          }

          break;

        case 7:
          LastSampleAngle = currentByte * 0x100 + LastSampleAngle;
          LastSampleAngleCal = LastSampleAngle;
          LastSampleAngle = LastSampleAngle >> 1;

          if (package_Sample_Num == 1) {
            IntervalSampleAngle = 0;
          } else {
            if (LastSampleAngle < FirstSampleAngle) {
              if ((FirstSampleAngle >= 180 * 64) &&
                  (LastSampleAngle <= 180 * 64)) { //实际雷达跨度不超过60度
                IntervalSampleAngle = (float)((360 * 64 + LastSampleAngle - FirstSampleAngle) / ((
                                                package_Sample_Num - 1) * 1.0));
                IntervalSampleAngle_LastPackage = IntervalSampleAngle;
              } else { //这里不应该发生
                if (FirstSampleAngle > 360) {///< 负数
                  IntervalSampleAngle = ((float)(LastSampleAngle - ((int16_t)FirstSampleAngle))) /
                                        (package_Sample_Num - 1);
                } else {//起始角大于结束角
                  uint16_t temp = FirstSampleAngle;
                  FirstSampleAngle = LastSampleAngle;
                  LastSampleAngle = temp;
                  IntervalSampleAngle = (float)((LastSampleAngle - FirstSampleAngle) / ((
                                                  package_Sample_Num - 1) * 1.0));
                }

                IntervalSampleAngle_LastPackage = IntervalSampleAngle;
                //IntervalSampleAngle = IntervalSampleAngle_LastPackage;
              }

            } else {
              IntervalSampleAngle = (float)((LastSampleAngle - FirstSampleAngle) / ((
                                              package_Sample_Num - 1) * 1.0));
              IntervalSampleAngle_LastPackage = IntervalSampleAngle;
            }
          }

          break;

        case 8:
          CheckSun = currentByte;
          break;

        case 9:
          CheckSun += (currentByte * 0x100);
          break;
        }

        packageBuffer[recvPos++] = currentByte;
      }

      if (recvPos  == PackagePaidBytes) {
        package_recvPos = recvPos;
        break;
      }
    }

    if (PackagePaidBytes == recvPos) {
      startTs = getms();
      recvPos = 0;

      while ((waitTime = getms() - startTs) <= timeout) {
        size_t remainSize = package_Sample_Num * PackageSampleBytes - recvPos;
        size_t recvSize;
        result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);

        if (ans != RESULT_OK) {
          delete[] recvBuffer;
          return ans;
        }

        if (recvSize > remainSize) {
          recvSize = remainSize;
        }

        getData(recvBuffer, recvSize);

        for (size_t pos = 0; pos < recvSize; ++pos) {
          if (m_intensities) {
            if (recvPos % 3 == 2) {
              Valu8Tou16 += recvBuffer[pos] * 0x100;
              CheckSunCal ^= Valu8Tou16;
            } else if (recvPos % 3 == 1) {
              Valu8Tou16 = recvBuffer[pos];
            } else {
              CheckSunCal ^= recvBuffer[pos];
            }
          } else {
            if (recvPos % 2 == 1) {
              Valu8Tou16 += recvBuffer[pos] * 0x100;
              CheckSunCal ^= Valu8Tou16;
            } else {
              Valu8Tou16 = recvBuffer[pos];
            }
          }

          packageBuffer[package_recvPos + recvPos] = recvBuffer[pos];
          recvPos++;
        }

        if (package_Sample_Num * PackageSampleBytes == recvPos) {
          package_recvPos += recvPos;
          break;
        }
      }

      if (package_Sample_Num * PackageSampleBytes != recvPos) {
        delete[] recvBuffer;
        return RESULT_FAIL;
      }
    } else {
      delete[] recvBuffer;
      return RESULT_FAIL;
    }

    CheckSunCal ^= SampleNumlAndCTCal;
    CheckSunCal ^= LastSampleAngleCal;

    if (CheckSunCal != CheckSun) {
      CheckSunResult = false;
    } else {
      CheckSunResult = true;
    }

  }

  uint8_t package_CT;

  if (m_intensities) {
    package_CT = package.package_CT;
  } else {
    package_CT = packages.package_CT;
  }

  if (package_CT == CT_Normal) {
    (node).sync_flag =  Node_NotSync;
  } else {
    (node).sync_flag =  Node_Sync;
  }

  (node).sync_quality = Node_Default_Quality;

  if (CheckSunResult) {
    if (m_intensities) {
      (node).sync_quality = (((package.packageSample[package_Sample_Index].PakageSampleDistance & 0x03) <<
                              LIDAR_RESP_MEASUREMENT_SYNC_QUALITY_SHIFT) |
                             package.packageSample[package_Sample_Index].PakageSampleQuality);
      (node).distance_q = package.packageSample[package_Sample_Index].PakageSampleDistance >>
                          LIDAR_RESP_MEASUREMENT_DISTANCE_SHIFT;
    } else {
      (node).distance_q = packages.packageSampleDistance[package_Sample_Index] >>
                          LIDAR_RESP_MEASUREMENT_DISTANCE_SHIFT;
    }

    if ((node).distance_q != 0) {
      AngleCorrectForDistance = (int32_t)(((atan(((21.8 * (155.3 - ((node).distance_q))) / 155.3) / ((
                                              node).distance_q))) * 180.0 / 3.1415) * 64.0);
    } else {
      AngleCorrectForDistance = 0;
    }

    if ((FirstSampleAngle + IntervalSampleAngle * package_Sample_Index + AngleCorrectForDistance) < 0) {
      (node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + IntervalSampleAngle *
                                              package_Sample_Index + AngleCorrectForDistance + 360 * 64)) << 1) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
    } else {
      if ((FirstSampleAngle + IntervalSampleAngle * package_Sample_Index + AngleCorrectForDistance) > 360
          * 64) {
        (node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + IntervalSampleAngle *
                                                package_Sample_Index + AngleCorrectForDistance - 360 * 64)) << 1) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
      } else {
        (node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + IntervalSampleAngle *
                                                package_Sample_Index + AngleCorrectForDistance)) << 1) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
      }
    }
  } else {
    (node).sync_flag = Node_NotSync;
    (node).sync_quality = Node_Default_Quality;
    (node).angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
    (node).distance_q = 0;
  }


  uint8_t nowPackageNum;

  if (m_intensities) {
    nowPackageNum = package.nowPackageNum;
  } else {
    nowPackageNum = packages.nowPackageNum;
  }

  if ((node).sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT) {
    m_ns = getTime() - (nowPackageNum * 3 + 10) * trans_delay - (nowPackageNum - 1) * m_pointTime;
    touchid = 0;
    (*point).new_frame = true;
  }

  (node).stamp = m_ns + package_Sample_Index * m_pointTime;

  (*point).isvalid = false;
  (*point).stamp = node.stamp;

  float angle = (float)((node.angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);

  if (angle > 360) {
    angle -= 360;
  }

  if (angle < 0) {
    angle += 360;
  }


  float dis = (float)node.distance_q;
  (*point).laser_x = dis * cos(angle * M_PI / 180.f);
  (*point).laser_y = dis * sin(angle * M_PI / 180.f);

  {

    ScopedLocker l(_plock);
    int coeff = laser_pose.reversion ? -1 : 1;

    (*point).screen_x = laser_pose.x + (*point).laser_x * cos(laser_pose.theta * M_PI / 180.0) -
                        coeff * (*point).laser_y * sin(laser_pose.theta * M_PI / 180.0);
    (*point).screen_y = laser_pose.y + coeff * (*point).laser_y * cos(laser_pose.theta * M_PI / 180.0)
                        + (*point).laser_x * sin(laser_pose.theta * M_PI / 180.0);


    if (inBox((*point).screen_x, (*point).screen_y) &&
        (fabs((*point).screen_x - laser_pose.x) > 0.001 ||
         fabs((*point).screen_y - laser_pose.y) > 0.001)) {
      touchid++;
      (*point).touchid = touchid;
      (*point).isvalid = true;
    }

  }

  package_Sample_Index++;

  if (package_Sample_Index >= nowPackageNum) {
    package_Sample_Index = 0;
    m_ns = (node).stamp + m_pointTime;
  }

  delete[] recvBuffer;
  return RESULT_OK;
}

result_t YDlidarDriver::waitScanData(touch_info *pointbuffer, size_t &count, uint32_t timeout) {
  if (!isConnected) {
    count = 0;
    return RESULT_FAIL;
  }

  size_t     recvPointCount =  0;
  uint32_t   startTs = getms();
  uint32_t   waitTime;
  result_t ans;

  while ((waitTime = getms() - startTs) <= timeout && recvPointCount < count) {
    touch_info point;

    if ((ans = this->waitPackage(&point, timeout - waitTime)) != RESULT_OK) {
      return ans;
    }

    pointbuffer[recvPointCount++] = point;

    if (recvPointCount == count) {
      return RESULT_OK;
    }
  }

  count = recvPointCount;
  return RESULT_FAIL;
}


result_t YDlidarDriver::grabScanData(touch_info *pointbuffer, size_t &count, uint32_t timeout) {
  switch (_dataEvent.wait(timeout)) {
  case Event::EVENT_TIMEOUT:
    count = 0;
    return RESULT_TIMEOUT;

  case Event::EVENT_OK: {
    if (touch_point_count == 0) {
      return RESULT_FAIL;
    }

    ScopedLocker l(_lock);
    size_t size_to_copy = min(touch_point_count, count);
    size_t point_count = 0;

    for (size_t i = 0; i < size_to_copy; i++) {
      if (touch_point_buf[i].isvalid) {
        pointbuffer[point_count] = touch_point_buf[i];
        point_count++;
      }
    }

    count = point_count;
    touch_point_count = 0;
  }

  return RESULT_OK;

  default:
    count = 0;
    return RESULT_FAIL;
  }

}


/************************************************************************/
/* get health state of lidar                                            */
/************************************************************************/
result_t YDlidarDriver::getHealth(device_health &health, uint32_t timeout) {
  result_t ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_DEVICE_HEALTH)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVHEALTH) {
      return RESULT_FAIL;
    }

    if (response_header.size < sizeof(device_health)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&health), sizeof(health));
  }
  return RESULT_OK;
}

/************************************************************************/
/* get device info of lidar                                             */
/************************************************************************/
result_t YDlidarDriver::getDeviceInfo(device_info &info, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_DEVICE_INFO)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size < sizeof(lidar_ans_header)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&info), sizeof(info));
    model = info.model;
  }

  return RESULT_OK;
}

/************************************************************************/
/* the set to signal quality                                            */
/************************************************************************/
void YDlidarDriver::setIntensities(const bool isintensities) {
  m_intensities = isintensities;

  if (m_intensities) {
    PackageSampleBytes = 3;
  } else {
    PackageSampleBytes = 2;
  }
}

/************************************************************************/
/* Get heartbeat function status                                        */
/************************************************************************/
const bool YDlidarDriver::getHeartBeat() const {
  return isHeartbeat;

}

/************************************************************************/
/* set heartbeat function status                                        */
/************************************************************************/
void YDlidarDriver::setHeartBeat(const bool enable) {
  isHeartbeat = enable;

}

/************************************************************************/
/* send heartbeat function package                                      */
/************************************************************************/
result_t YDlidarDriver::sendHeartBeat() {
  if (!isConnected) {
    return RESULT_FAIL;
  }

  ScopedLocker lock(_lock);
  result_t ans = sendCommand(LIDAR_CMD_SCAN);
  return ans;
}


/**
* @brief 设置雷达异常自动重新连接 \n
* @param[in] enable    是否开启自动重连:
*     true	开启
*	  false 关闭
*/
void YDlidarDriver::setAutoReconnect(const bool &enable) {
  isAutoReconnect = enable;
}

/************************************************************************/
/*  start to scan                                                       */
/************************************************************************/
result_t YDlidarDriver::startScan(bool force, uint32_t timeout) {
  result_t ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  if (isScanning) {
    return RESULT_OK;
  }

  stop();
  startMotor();

  {
    //calc stamp
    m_pointTime = 1e9 / 4000;
    trans_delay = 0;
    {
      if (model != -1) {
        switch (model) {
        case 1://f4
          trans_delay = _serial->getByteTime();
          break;

        case 5: { //g4
          if (_sampling_rate == -1) {
            sampling_rate _rate;
            getSamplingRate(_rate);
            _sampling_rate = _rate.rate;
          }

          switch (_sampling_rate) {
          case 1:
            m_pointTime = 1e9 / 8000;
            break;

          case 2:
            m_pointTime = 1e9 / 9000;
            break;
          }

        }

        trans_delay = _serial->getByteTime();
        break;

        case 6://x4
          m_pointTime = 1e9 / 5000;
          break;

        case 8: { //f4pro
          if (_sampling_rate == -1) {
            sampling_rate _rate;
            getSamplingRate(_rate);
            _sampling_rate = _rate.rate;
          }

          if (_sampling_rate == 1) {
            m_pointTime = 1e9 / 6000;
          }

        }

        trans_delay = _serial->getByteTime();
        break;

        case 9://g4c
          trans_delay = _serial->getByteTime();
          break;
        }
      }
    }
  }

  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(force ? LIDAR_CMD_FORCE_SCAN : LIDAR_CMD_SCAN)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
      return RESULT_FAIL;
    }

    if (response_header.size < 5) {
      return RESULT_FAIL;
    }

    ans = this->createThread();
    return ans;
  }
  return RESULT_OK;
}

result_t YDlidarDriver::createThread() {
  _thread = CLASS_THREAD(YDlidarDriver, cacheScanData);

  if (_thread.getHandle() == 0) {
    return RESULT_FAIL;
  }

  isScanning = true;
  return RESULT_OK;
}

/************************************************************************/
/*   startAutoScan                                                      */
/************************************************************************/
result_t YDlidarDriver::startAutoScan(bool force, uint32_t timeout) {
  result_t ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(force ? LIDAR_CMD_FORCE_SCAN : LIDAR_CMD_SCAN)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
      return RESULT_FAIL;
    }

    if (response_header.size < 5) {
      return RESULT_FAIL;
    }

  }

  startMotor();
  return RESULT_OK;
}


/************************************************************************/
/*   stop scan                                                   */
/************************************************************************/
result_t YDlidarDriver::stop() {
  if (isAutoconnting) {
    isAutoReconnect = false;
    isScanning = false;
    disableDataGrabbing();
    return RESULT_OK;

  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);
    sendCommand(LIDAR_CMD_FORCE_STOP);
    sendCommand(LIDAR_CMD_STOP);
  }

  stopMotor();

  return RESULT_OK;
}

/************************************************************************/
/*  reset device                                                        */
/************************************************************************/
result_t YDlidarDriver::reset(uint32_t timeout) {
  UNUSED(timeout);
  result_t ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  ScopedLocker l(_lock);

  if ((ans = sendCommand(LIDAR_CMD_RESET)) != RESULT_OK) {
    return ans;
  }

  return RESULT_OK;
}

/************************************************************************/
/* get the current scan frequency of lidar                              */
/************************************************************************/
result_t YDlidarDriver::getScanFrequency(scan_frequency &frequency, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_AIMSPEED)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 4) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/* add the scan frequency by 1Hz each time                              */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyAdd(scan_frequency &frequency, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_ADD)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 4) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/* decrease the scan frequency by 1Hz each time                         */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyDis(scan_frequency &frequency, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_DIS)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 4) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/* add the scan frequency by 0.1Hz each time                            */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyAddMic(scan_frequency &frequency, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_ADDMIC)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 4) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/* decrease the scan frequency by 0.1Hz each time                       */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyDisMic(scan_frequency &frequency, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_DISMIC)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 4) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/*  get the sampling rate of lidar                                      */
/************************************************************************/
result_t YDlidarDriver::getSamplingRate(sampling_rate &rate, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_SAMPLING_RATE)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&rate), sizeof(rate));
    _sampling_rate = rate.rate;
  }
  return RESULT_OK;
}

/************************************************************************/
/*  the set to sampling rate                                            */
/************************************************************************/
result_t YDlidarDriver::setSamplingRate(sampling_rate &rate, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_SAMPLING_RATE)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&rate), sizeof(rate));
    _sampling_rate = rate.rate;
  }
  return RESULT_OK;
}

std::string YDlidarDriver::getSDKVersion() {
  return SDKVerision;
}


result_t YDlidarDriver::setRotationPositive(scan_rotation &roation, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_RUN_POSITIVE)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&roation), sizeof(roation));
  }
  return RESULT_OK;
}

result_t YDlidarDriver::setRotationInversion(scan_rotation &roation, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_RUN_INVERSION)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&roation), sizeof(roation));
  }
  return RESULT_OK;
}

/************************************************************************/
/* enable lower power                                                   */
/************************************************************************/
result_t YDlidarDriver::enableLowerPower(function_state &state, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_ENABLE_LOW_POWER)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&state), sizeof(state));
  }
  return RESULT_OK;
}

/************************************************************************/
/*  disable lower power                                                 */
/************************************************************************/
result_t YDlidarDriver::disableLowerPower(function_state &state, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_DISABLE_LOW_POWER)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&state), sizeof(state));
  }
  return RESULT_OK;
}

/************************************************************************/
/* get the state of motor                                               */
/************************************************************************/
result_t YDlidarDriver::getMotorState(function_state &state, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_STATE_MODEL_MOTOR)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&state), sizeof(state));
  }
  return RESULT_OK;
}

/************************************************************************/
/* enable constant frequency                                            */
/************************************************************************/
result_t YDlidarDriver::enableConstFreq(function_state &state, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_ENABLE_CONST_FREQ)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&state), sizeof(state));
  }
  return RESULT_OK;
}

/************************************************************************/
/* disable constant frequency                                           */
/************************************************************************/
result_t YDlidarDriver::disableConstFreq(function_state &state, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_DISABLE_CONST_FREQ)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&state), sizeof(state));
  }
  return RESULT_OK;
}

/************************************************************************/
/*  the save current low exposure value for S4 which has signal quality                */
/************************************************************************/
result_t YDlidarDriver::setSaveLowExposure(scan_exposure &low_exposure, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SAVE_SET_EXPOSURE)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&low_exposure), sizeof(low_exposure));
  }
  return RESULT_OK;


}

/************************************************************************/
/*  the set to low exposure for S4 which has signal quality                */
/************************************************************************/
result_t YDlidarDriver::setLowExposure(scan_exposure &low_exposure, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_LOW_EXPOSURE)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&low_exposure), sizeof(low_exposure));
  }
  return RESULT_OK;


}

/************************************************************************/
/*  add scan exposure for S4 which has signal quality                      */
/************************************************************************/
result_t YDlidarDriver::setLowExposureAdd(scan_exposure &exposure, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_ADD_EXPOSURE)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&exposure), sizeof(exposure));
  }
  return RESULT_OK;

}

/************************************************************************/
/*  decrease scan exposure for S4 which has signal quality                 */
/************************************************************************/
result_t YDlidarDriver::setLowExposurerDis(scan_exposure &exposure, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_DIS_EXPOSURE)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&exposure), sizeof(exposure));
  }
  return RESULT_OK;


}

/************************************************************************/
/*  set heartbeat function for G4 F4Pro                  */
/************************************************************************/
result_t YDlidarDriver::setScanHeartbeat(scan_heart_beat &beat, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker lock(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_HEART_BEAT)) != RESULT_OK) {
      return ans;

    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&beat), sizeof(beat));



  }

  return RESULT_OK;

}

/************************************************************************/
/*  set a circle of data fixed points for S4                  */
/************************************************************************/
result_t YDlidarDriver::setPointsForOneRingFlag(scan_points &points, uint32_t timeout) {
  result_t  ans;

  if (!isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_SETPOINTSFORONERINGFLAG)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&points), sizeof(points));
  }
  return RESULT_OK;

}

}
