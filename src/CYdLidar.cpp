#include "CYdLidar.h"
#include "common.h"
#include <map>

using namespace std;
using namespace ydlidar;

/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar(): lidarPtr(nullptr)
{
    m_SerialPort        = "";
    m_SerialBaudrate    = 153600;
    m_Intensities       = true;
    m_AutoReconnect     = true;
    m_MaxAngle          = 180.f;
    m_MinAngle          = -180.f;
    m_MaxRange          = 12.0;
    m_MinRange          = 0.08;
    m_EnableDebug       = false;
    isScanning          = false;
    m_AbnormalCheckCount= 2;

}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar()
{
    disconnecting();
}

void CYdLidar::disconnecting()
{
    if (lidarPtr) {
        lidarPtr->disconnect();
        delete lidarPtr;
        lidarPtr = nullptr;
    } 
    isScanning = false;
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(node_info *nodes, size_t& count, bool &hardwareError){
	hardwareError			= false;
	// Bound?
	if (!checkHardware()) {
			hardwareError = true;
			delay(80);
			return false;
	}
	if(count == 0) {
			count = 2048;
	}
	//wait Scan data:
	result_t op_result =  lidarPtr->grabScanData(nodes, count);
// Fill in scan data:
  if (IS_OK(op_result)) {
      return true;
  } else {
      if (IS_FAIL(op_result)) {

      }

	}
	return false;

}

bool CYdLidar::ascendScanData(node_info *nodebuffer, size_t count) {
    bool ret = false;
    result_t op_result = lidarPtr->ascendScanData(nodebuffer, count);
    if(IS_OK(op_result)) {
        ret = true;
    }
    return ret;
}


/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn()
{
  if (isScanning && lidarPtr->isscanning()) {
    return true;
  }
  // start scan...
  result_t op_result = lidarPtr->startScan();
  if (!IS_OK(op_result)) {
    op_result = lidarPtr->startScan();
    if (!IS_OK(op_result)) {
      fprintf(stderr, "[CYdLidar] Failed to start scan mode: %x\n", op_result);
      isScanning = false;
      return false;
    }
  }
  if (checkLidarAbnormal()) {
      lidarPtr->stop();
      fprintf(stderr, "[CYdLidar] Failed to turn on the Lidar, because the lidar is blocked or the lidar hardware is faulty.\n");
      isScanning = false;
      return false;
  }
  isScanning = true;
  lidarPtr->setAutoReconnect(m_AutoReconnect);
  printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
  fflush(stdout);
  return true;
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff()
{
  if (lidarPtr) {
    lidarPtr->stop();
    lidarPtr->stopMotor();
  }
  if (isScanning) {
    printf("[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......\n");
  }
  isScanning = false;
  return true;
}

bool CYdLidar::checkLidarAbnormal() {
  node_info nodes[2048];
  size_t   count = _countof(nodes);
  int check_abnormal_count = 0;
  if (m_AbnormalCheckCount < 2) {
    m_AbnormalCheckCount = 2;
  }
  result_t op_result = RESULT_FAIL;
  while (check_abnormal_count < m_AbnormalCheckCount) {
    //Ensure that the voltage is insufficient or the motor resistance is high, causing an abnormality.
    if (check_abnormal_count > 0) {
      delay(check_abnormal_count*1000);
    }
    op_result =  lidarPtr->grabScanData(nodes, count);
    if (IS_OK(op_result)) {
      return false;
    }
    check_abnormal_count++;
  }
  return !IS_OK(op_result);
}


/** Returns true if the device is connected & operative */
bool CYdLidar::getDeviceHealth() const {
    if (!lidarPtr) return false;

	result_t op_result;
    device_health healthinfo;
    printf("[YDLIDAR]:SDK Version: %s\n", YDlidarDriver::getSDKVersion().c_str());
    op_result = lidarPtr->getHealth(healthinfo);
    if (IS_OK(op_result)) {
        printf("[YDLIDAR]:Lidar running correctly ! The health status: %s\n", (int)healthinfo.status==0?"good":"bad");

        if (healthinfo.status == 2) {
            if (print_error >= 2){
                fprintf(stderr, "Error, Yd Lidar internal error detected. Please reboot the device to retry.\n");
            }
            return false;
        } else {   
            return true;
        }

    } else {
        if (print_error >= 2) {
            fprintf(stderr, "Error, cannot retrieve Yd Lidar health code: %x\n", op_result);
        }
        return false;
    }

}

bool CYdLidar::getDeviceInfo() {
  if (!lidarPtr) return false;
  device_info devinfo;
  result_t op_result = lidarPtr->getDeviceInfo(devinfo);
  if (!IS_OK(op_result)) {
    fprintf(stderr, "get Device Information Error\n");
    return false;
  }
  if (devinfo.model != YDlidarDriver::YDLIDAR_S4) {
    printf("[YDLIDAR INFO] Current SDK does not support current lidar models[%d]\n", devinfo.model);
    return false;
  }
  std::string model = "S4B";
  int m_samp_rate = 4;
  switch (devinfo.model) {
  case YDlidarDriver::YDLIDAR_S4:
    model = "S4";
    if(m_SerialBaudrate==153600) {
        model="S4B";
    }
  break;
  default:
  break;
  }

  uint8_t Major = (uint8_t)(devinfo.firmware_version>>8);
  uint8_t Minjor = (uint8_t)(devinfo.firmware_version&0xff);
  printf("[YDLIDAR] Connection established in [%s][%d]:\n"
             "Firmware version: %u.%u\n"
       "Hardware version: %u\n"
       "Model: %s\n"
       "Serial: ",
              m_SerialPort.c_str(),
              m_SerialBaudrate,
              Major,
              Minjor,
        (unsigned int)devinfo.hardware_version,
        model.c_str());
  for (int i=0;i<16;i++) {
      printf("%01X",devinfo.serialnum[i]&0xff);
  }
  printf("\n");
  printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n" , m_samp_rate);
  return true;



}


/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs()
{
    if (!lidarPtr) {
        // create the driver instance
        lidarPtr = new YDlidarDriver();
        if (!lidarPtr) {
             fprintf(stderr, "Create Driver fail\n");
             return false;
        }
    }
    if (lidarPtr->isconnected()) {
        return true;
    }

	// Is it COMX, X>4? ->  "\\.\COMX"
    if (m_SerialPort.size()>=3) {
        if ( tolower( m_SerialPort[0]) =='c' && tolower( m_SerialPort[1]) =='o' && tolower( m_SerialPort[2]) =='m' ) {
			// Need to add "\\.\"?
            if (m_SerialPort.size()>4 || m_SerialPort[3]>'4')
                m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
		}
	}

	// make connection...
    lidarPtr->setSaveParse(m_EnableDebug, "ydldiar_scan.txt");
    result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);
    if (!IS_OK(op_result)) {
        fprintf(stderr, "[CYdLidar] Error, cannot bind to the specified serial port[%s] and baudrate[%d]\n",  m_SerialPort.c_str(), m_SerialBaudrate );
		return false;
	}

	return true;
}

/*-------------------------------------------------------------
                        checkStatus
-------------------------------------------------------------*/
bool CYdLidar::checkStatus()
{
  if (!checkCOMMs()) {
    return false;
  }
  bool ret = getDeviceHealth();
  if (!ret) {
    delay(1000);
  }

  if (!getDeviceInfo()) {
    delay(2000);
    ret = getDeviceInfo();
    if (!ret) {
      return false;
    }
  }
  lidarPtr->setIntensity(m_Intensities);
  return true;

}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware()
{
  if (!lidarPtr) {
    return false;
  }
  if (isScanning && lidarPtr->isscanning()) {
    return true;
  }
  return false;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize()
{
	bool ret = true;
	if (!checkCOMMs()) {
        fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner.\n");
        fflush(stderr);
        return false;
	}
	if(!checkStatus()) {
			fprintf(stderr, "[CYdLidar::initialize] Error initializing YDLIDAR check status.\n");
			fflush(stderr);
			return false;
	}

  if (!turnOn()) {
      fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner. Because the motor falied to start.\n");
      fflush(stderr);
      return false;
	} 
	return ret;
	
}
