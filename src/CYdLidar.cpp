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
    isScanning          = false;

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
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(node_info *nodes, size_t& count, bool &hardwareError){
	hardwareError			= false;
	// Bound?
    if (!checkHardware()) {
        hardwareError = true;
        return false;
	}
    if(count == 0) {
        count = 2048;
    }
    //wait Scan data:
    result_t op_result =  lidarPtr->grabScanData(nodes, count);
	// Fill in scan data:
    if (IS_OK(op_result)) {
        op_result = lidarPtr->ascendScanData(nodes, count);
        if (IS_OK(op_result)) {
            return true;
		}

    } else {
        if (IS_FAIL(op_result)) {
		}
	}

	return false;

}


/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn()
{
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
bool  CYdLidar::turnOff()
{
    if (lidarPtr) {
        lidarPtr->stop();
        lidarPtr->stopMotor();
        isScanning = false;
	}
	return true;
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

bool CYdLidar::getDeviceInfo(int &lidar_model) {

    if (!lidarPtr) return false;

	device_info devinfo;
    result_t op_result = lidarPtr->getDeviceInfo(devinfo);
    if (!IS_OK(op_result)) {
        if (print_error >= 2) {
            fprintf(stderr, "get Device Information Error\n" );
        }
		return false;
	}	 
	std::string model;
    int m_samp_rate =4;
    lidar_model = devinfo.model;
    switch (devinfo.model) {
    case YDlidarDriver::YDLIDAR_F4:
        model="F4";
        break;
    case YDlidarDriver::YDLIDAR_T1:
        model="T1";
        break;
    case YDlidarDriver::YDLIDAR_F2:
        model="F2";
        break;
    case YDlidarDriver::YDLIDAR_S4:
        model="S4";
        if(m_SerialBaudrate==153600) {
            model="S4B";
        }
        break;
    case YDlidarDriver::YDLIDAR_G4:
    {
        model="G4";
        m_samp_rate = 9;

    }
        break;
    case YDlidarDriver::YDLIDAR_X4:
        model="X4";
        m_samp_rate = 5;
        break;
    case YDlidarDriver::YDLIDAR_G4PRO:
        model="G4Pro";
        break;
    case YDlidarDriver::YDLIDAR_F4PRO:
    {
        model="F4Pro";
        m_samp_rate = 4;
    }
        break;
    case YDlidarDriver::YDLIDAR_G2_SS_1:
        model="G2-SS-1";
        m_samp_rate = 5;
        break;
    case YDlidarDriver::YDLIDAR_G10:
        model = "G10";
        break;
    case YDlidarDriver::YDLIDAR_S4B:
        model = "S4B";
        break;
    case YDlidarDriver::YDLIDAR_S2:
        model = "S2";
        break;
    case YDlidarDriver::YDLIDAR_G25:
        model = "G25";
        break;
    default:
        model = "Unknown";
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

    if (!lidarPtr)
        return false;
    if (lidarPtr->isscanning())
        return true;
    int m_model = -1;
    while (print_error < 3) {
        bool ret = getDeviceHealth();
        if(!ret) {
            delay(1000);
        }
        if(!getDeviceInfo(m_model)&&!ret) {
            print_error++;
            lidarPtr->disconnect();
            bool ret = checkCOMMs();
            if (!ret) {
                return false;
            }
        }else {
            break;
        }

    }
    result_t op_result;
    if(m_SerialBaudrate == 153600) {
        m_Intensities = true;
    }
    lidarPtr->setIntensity(m_Intensities);

     // start scan...
    op_result = lidarPtr->startScan();
    if (!IS_OK(op_result)) {
        op_result = lidarPtr->startScan();
        if(!IS_OK(op_result)) {
            fprintf(stderr, "[CYdLidar] Error starting scanning mode: %x\n", op_result);
            isScanning = false;
            return false;
        }
    }
    lidarPtr->setAutoReconnect(m_AutoReconnect);
    printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
    fflush(stdout);
    fflush(stderr);
    isScanning = true;
    delay(50);
    return true;

}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware()
{
    bool ret = true;
    if (!isScanning) {
        ret = checkCOMMs();
        if (ret) {
            ret = checkStatus();
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
bool CYdLidar::initialize()
{
	bool ret = true;
    if (!checkCOMMs()) {
        fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner.\n");
        fflush(stderr);
        return false;
	}
    if (!checkStatus()) {
        fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner.because of failure in scan mode.\n");
        fflush(stderr);
    }
    if (!turnOn()) {
        fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner. Because the motor falied to start.\n");
        fflush(stderr);
		
	}
    return ret;
	
}
