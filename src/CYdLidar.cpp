#include "CYdLidar.h"
#include "common.h"
#include <map>


using namespace std;
using namespace ydlidar;
using namespace impl;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar(): lidarPtr(nullptr)
{
    m_SerialPort        = "";
    m_SerialBaudrate    = 115200;
    m_Intensities       = false;
    m_FixedResolution   = false;
    m_Exposure          = false;
    m_HeartBeat         = false;
    m_Reversion         = false;
    m_AutoReconnect     = false;
    m_MaxAngle          = 180.f;
    m_MinAngle          = -180.f;
    m_MaxRange          = 16.0;
    m_MinRange          = 0.08;
    m_SampleRate        = 9;
    m_ScanFrequency     = 7;
    m_AngleOffset       = 0.0;
    m_GlassNoise        = true;
    m_SunNoise          = true;
    isScanning          = false;
    node_counts         = 720;
    each_angle          = 0.5;
    print_error         = 0;
    frequencyOffset     = 0.4;
    m_CalibrationFileName = "";
    m_IgnoreArray.clear();

    ini.SetUnicode();
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
bool  CYdLidar::doProcessSimple(LaserScan &outscan, bool &hardwareError){
	hardwareError			= false;

	// Bound?
    if (!checkHardware()) {
        hardwareError = true;
        return false;
	}

    node_info nodes[2048];
    size_t   count = _countof(nodes);
    size_t all_nodes_counts = node_counts;

    //wait Scan data:
    uint64_t tim_scan_start = getTime();
    result_t op_result =  lidarPtr->grabScanData(nodes, count);
    uint64_t tim_scan_end = getTime();

	// Fill in scan data:
    if (IS_OK(op_result)) {
        op_result = lidarPtr->ascendScanData(nodes, count);
        tim_scan_start = nodes[0].stamp;
        tim_scan_end   = nodes[0].stamp;

        double scan_time = tim_scan_end - tim_scan_start;
        if (IS_OK(op_result)) {
            if (!m_FixedResolution) {
                all_nodes_counts = count;
            } else {
                all_nodes_counts = node_counts;
            }
            each_angle = 360.0/all_nodes_counts;

            node_info *angle_compensate_nodes = new node_info[all_nodes_counts];
            memset(angle_compensate_nodes, 0, all_nodes_counts*sizeof(node_info));
            unsigned int i = 0;
            for( ; i < count; i++) {
                if (nodes[i].distance_q != 0) {
                    float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f) + m_AngleOffset;
                    if(m_Reversion){
                       angle=angle+180;
                       if(angle>=360){ angle=angle-360;}
                        nodes[i].angle_q6_checkbit = ((uint16_t)(angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT;
                    }
                    int inter =(int)( angle / each_angle );
                    float angle_pre = angle - inter * each_angle;
                    float angle_next = (inter+1) * each_angle - angle;
                    if (angle_pre < angle_next) {
                        if(inter < all_nodes_counts)
                            angle_compensate_nodes[inter]=nodes[i];
                    } else {
                        if (inter < all_nodes_counts -1)
                            angle_compensate_nodes[inter+1]=nodes[i];
                    }
                }

                if(nodes[i].stamp < tim_scan_start) {
                    tim_scan_start = nodes[i].stamp;
                }
                if(nodes[i].stamp > tim_scan_end) {
                    tim_scan_end = nodes[i].stamp;
                }

             }

            LaserScan scan_msg;

            if (m_MaxAngle< m_MinAngle) {
                float temp = m_MinAngle;
                m_MinAngle = m_MaxAngle;
                m_MaxAngle = temp;
            }


            int counts = all_nodes_counts*((m_MaxAngle-m_MinAngle)/360.0f);
            int angle_start = 180+m_MinAngle;
            int node_start = all_nodes_counts*(angle_start/360.0f);

            scan_time = tim_scan_end - tim_scan_start;
            scan_msg.ranges.resize(counts);
            scan_msg.intensities.resize(counts);
            float range = 0.0;
            float intensity = 0.0;
            int index = 0;


            for (size_t i = 0; i < all_nodes_counts; i++) {
                range = (float)angle_compensate_nodes[i].distance_q/4000.f;
                uint8_t intensities = (uint8_t)(angle_compensate_nodes[i].sync_quality >> LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                intensities = (float)intensities;
                if(m_GlassNoise&&intensities == GLASSNOISEINTENSITY ) {
                    intensity = 0.0;
                    range     = 0.0;
                }
                if(m_SunNoise&&intensities == SUNNOISEINTENSITY) {
                    intensity = 0.0;
                    range     = 0.0;
                }

                if (i<all_nodes_counts/2) {
                    index = all_nodes_counts/2-1-i;
                } else {
                    index =all_nodes_counts-1-(i-all_nodes_counts/2);
                }

                if (m_IgnoreArray.size() != 0) {
                    float angle = (float)((angle_compensate_nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                    if (angle>180) {
                        angle=360-angle;
                    } else {
                        angle=-angle;
                    }

                    for (uint16_t j = 0; j < m_IgnoreArray.size();j = j+2) {
                        if ((m_IgnoreArray[j] < angle) && (angle <= m_IgnoreArray[j+1])) {
                           range = 0.0;
                           break;
                        }
                    }
                }

                if (range > m_MaxRange|| range < m_MinRange) {
                    range = 0.0;
                }

                int pos = index - node_start ;
                if (0<= pos && pos < counts) {
                    scan_msg.ranges[pos] =  range;
                    scan_msg.intensities[pos] = intensity;
                }
            }

            scan_msg.system_time_stamp = tim_scan_start;
            scan_msg.self_time_stamp = tim_scan_start;
            scan_msg.config.min_angle = DEG2RAD(m_MinAngle);
            scan_msg.config.max_angle = DEG2RAD(m_MaxAngle);
            scan_msg.config.ang_increment = (scan_msg.config.max_angle - scan_msg.config.min_angle) / (double)counts;
            scan_msg.config.time_increment = scan_time / (double)counts;
            scan_msg.config.scan_time = scan_time;
            scan_msg.config.min_range = m_MinRange;
            scan_msg.config.max_range = m_MaxRange;
            outscan = scan_msg;
            delete[] angle_compensate_nodes;
            return true;


		}

    } else {
        if (IS_FAIL(op_result)) {
			// Error? Retry connection
			//this->disconnect();
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
            if (print_error == 3){
                fprintf(stderr, "Error, Yd Lidar internal error detected. Please reboot the device to retry.\n");
            }
            return false;
        } else {   
            return true;
        }

    } else {
        if (print_error == 3) {
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
        if (print_error == 3) {
            fprintf(stderr, "get Device Information Error\n" );
        }
		return false;
	}	 
	std::string model;
    sampling_rate _rate;
    int m_samp_rate =4;
    int error_time = 0;

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
        break;
    case YDlidarDriver::YDLIDAR_G4:
    {
        model="G4";
        op_result = lidarPtr->getSamplingRate(_rate);
        if (IS_OK(op_result)) {
            switch (m_SampleRate) {
            case 4:
                m_samp_rate=YDlidarDriver::YDLIDAR_RATE_4K;
                break;
            case 8:
                m_samp_rate=YDlidarDriver::YDLIDAR_RATE_8K;
                break;
            case 9:
                m_samp_rate=YDlidarDriver::YDLIDAR_RATE_9K;
                break;
            default:
                m_samp_rate = _rate.rate;
                break;
            }

            while (m_samp_rate != _rate.rate) {
                op_result = lidarPtr->setSamplingRate(_rate);
                if (!IS_OK(op_result)) {
                    error_time++;
                    if(error_time >5 ){
                        break;
                    }
                }
            }

            switch (_rate.rate) {
                case YDlidarDriver::YDLIDAR_RATE_4K:
                    m_samp_rate = 4;
                    break;
                case YDlidarDriver::YDLIDAR_RATE_8K:
                    node_counts = 1440;
                    each_angle  = 0.25;
                    m_samp_rate =8;
                    break;
                case YDlidarDriver::YDLIDAR_RATE_9K:
                    node_counts = 1440;
                    each_angle  = 0.25;
                    m_samp_rate =9;
                    break;
            }


        }
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
        op_result = lidarPtr->getSamplingRate(_rate);
        if (IS_OK(op_result)) {
            switch (m_SampleRate) {
            case 4:
                m_samp_rate =0;
                break;
            case 6:
                m_samp_rate =1;
                break;
            default:
                m_samp_rate = _rate.rate;
                break;
            }
            while (m_samp_rate != _rate.rate) {
                op_result = lidarPtr->setSamplingRate(_rate);
                if (!IS_OK(op_result)) {
                    error_time++;
                    if(error_time >5 ){
                        break;
                    }
                }
            }

            switch (_rate.rate) {
                case 0:
                    m_samp_rate = 4;
                    break;
                case 1:
                    node_counts = 1440;
                    each_angle = 0.25;
                    m_samp_rate=6;
                    break;
            }

        }
    }
        break;
    case YDlidarDriver::YDLIDAR_G4C:
        model="G2-SS-1";
        m_samp_rate = 5;
        break;
    case YDlidarDriver::YDLIDAR_G10:
        model = "G10";
        node_counts = 1440;
        each_angle = 0.25;
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

    m_SampleRate = m_samp_rate;



    uint8_t Major = (uint8_t)(devinfo.firmware_version>>8);
    uint8_t Minjor = (uint8_t)(devinfo.firmware_version&0xff);
    std::string serial_number;
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
        //printf("%01X",devinfo.serialnum[i]&0xff);
        serial_number += format("%01X",devinfo.serialnum[i]&0xff);
    }
    printf("%s\n", serial_number.c_str());
    printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n" , m_samp_rate);
    checkCalibrationAngle(serial_number);

    float frequency = 7.0f;
    if (devinfo.model == YDlidarDriver::YDLIDAR_G4 ||
            devinfo.model ==YDlidarDriver::YDLIDAR_F4PRO ||
            devinfo.model == YDlidarDriver::YDLIDAR_G4C ||
            devinfo.model == YDlidarDriver::YDLIDAR_G10 ||
            devinfo.model == YDlidarDriver::YDLIDAR_G25) {
        checkScanFrequency();
        checkHeartBeat();
    } else {
        printf("[YDLIDAR INFO] Current Scan Frequency : %fHz\n" , frequency);
    }

    return true;


	

}

/*-------------------------------------------------------------
                        checkScanFrequency
-------------------------------------------------------------*/
bool CYdLidar::checkScanFrequency()
{
    float frequency = 7.0f;
    scan_frequency _scan_frequency;
    int hz = 0;
    if (5 <= m_ScanFrequency && m_ScanFrequency <= 13) {
        result_t ans = lidarPtr->getScanFrequency(_scan_frequency) ;
        if (IS_OK(ans)) {
            frequency = _scan_frequency.frequency/100.f;
            hz = m_ScanFrequency + frequencyOffset - frequency;
            if (hz >0 ) {
                while (hz != 0) {
                    lidarPtr->setScanFrequencyAdd(_scan_frequency);
                    hz--;
                }
                frequency = _scan_frequency.frequency/100.0f;
            } else {
                while (hz != 0) {
                    lidarPtr->setScanFrequencyDis(_scan_frequency);
                    hz++;
                }
                frequency = _scan_frequency.frequency/100.0f;
            }
        }
        if(fabs(m_ScanFrequency+ frequencyOffset - frequency) < 0.95) {
            hz = (m_ScanFrequency+ frequencyOffset - frequency)*10;
            if (hz>0) {
                while (hz != 0) {
                    lidarPtr->setScanFrequencyAddMic(_scan_frequency);
                    hz--;
                }
                frequency = _scan_frequency.frequency/100.0f;
            } else {
                while (hz != 0) {
                    lidarPtr->setScanFrequencyDisMic(_scan_frequency);
                    hz++;
                }
                frequency = _scan_frequency.frequency/100.0f;
            }

        }

        if (m_ScanFrequency+ frequencyOffset < 7 && m_SampleRate>6) {
            node_counts = 1600;

        } else if ( m_ScanFrequency+ frequencyOffset < 6 && m_SampleRate == 9) {
            node_counts = 2000;

        } else if ( m_ScanFrequency+ frequencyOffset < 6 && m_SampleRate == 4) {
            node_counts = 900;
        }
        node_counts = m_SampleRate*1000/((float)m_ScanFrequency -frequencyOffset);
        each_angle = 360.0/node_counts;
    }
    printf("[YDLIDAR INFO] Current Scan Frequency : %fHz\n" , (float)m_ScanFrequency);

    return true;

}

/*-------------------------------------------------------------
                        checkHeartBeat
-------------------------------------------------------------*/

bool CYdLidar::checkHeartBeat() const
{
    bool ret = false;
    scan_heart_beat beat;
    if( m_HeartBeat ) {
        do{
            result_t ans = lidarPtr->setScanHeartbeat(beat);
            if(IS_OK(ans)) {
                if( beat.enable ) {
                    ans = lidarPtr->setScanHeartbeat(beat);
                    if(IS_OK(ans)) {
                        if(!beat.enable) {
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

        }while(true);

    }else {
        lidarPtr->setHeartBeat(false);
        ret = true;
    }

    return ret;

}
/*-------------------------------------------------------------
                        checkCalibrationAngle
-------------------------------------------------------------*/
void CYdLidar::checkCalibrationAngle(const std::string& serialNumber) {
    m_AngleOffset = 0.0;
    if(ydlidar::fileExists(m_CalibrationFileName)) {
        SI_Error rc = ini.LoadFile(m_CalibrationFileName.c_str());
        if (rc >= 0) {
            m_AngleOffset = ini.GetDoubleValue("CALIBRATION", serialNumber.c_str(), m_AngleOffset);
            printf("[YDLIDAR INFO] Successfully obtained the calibration value[%f] from the calibration file[%s]\n" ,m_AngleOffset, m_CalibrationFileName.c_str());

        }else {
            printf("[YDLIDAR INFO] Failed to open calibration file[%s]\n" , m_CalibrationFileName.c_str());
        }
    }else {
        printf("[YDLIDAR INFO] Calibration file[%s] does not exist\n" , m_CalibrationFileName.c_str());
    }
    printf("[YDLIDAR INFO] Current AngleOffset : %f°\n" , m_AngleOffset);
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

    std::map<int, bool> checkmodel;
    checkmodel.insert(std::map<int, bool>::value_type(115200, false));
    checkmodel.insert(std::map<int, bool>::value_type(128000, false));
    checkmodel.insert(std::map<int, bool>::value_type(153600, false));
    checkmodel.insert(std::map<int, bool>::value_type(230400, false));

    again:
    // check health:
    bool ret = getDeviceHealth();

    int m_model;
    if (!ret||!getDeviceInfo(m_model)){
        checkmodel[m_SerialBaudrate] = true;
        map<int,bool>::iterator it;
        for (it=checkmodel.begin(); it!=checkmodel.end(); ++it) {
            if(it->second)
                continue;
            print_error++;
            lidarPtr->disconnect();
            delete lidarPtr;
            lidarPtr = nullptr;
            lidarPtr = new YDlidarDriver();
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

    result_t op_result;
    print_error = 0;
    m_Intensities = false;
    if (m_model == YDlidarDriver::YDLIDAR_S4 || m_model == YDlidarDriver::YDLIDAR_S4B) {
        if (m_SerialBaudrate == 153600||m_model == YDlidarDriver::YDLIDAR_S4B)
            m_Intensities = true;
        if (m_Intensities) {
            scan_exposure exposure;
            int error_count = 0;
            op_result = lidarPtr->setLowExposure(exposure);
            while (IS_OK(op_result) && (error_count < 3)) {
                if (exposure.exposure != m_Exposure) {  
                    printf("set exposure model success!!!\n");
                    break;
                }
                error_count++;
            }
            if (error_count >= 3 ) {
                fprintf(stderr,"set low exposure model failed!!!\n");
            }
        }
    }

    lidarPtr->setIntensities(m_Intensities);

     // start scan...
    op_result = lidarPtr->startScan();
    if (!IS_OK(op_result)) {
        fprintf(stderr, "[CYdLidar] Error starting scanning mode: %x\n", op_result);
        isScanning = false;
        return false;
    }
    lidarPtr->setAutoReconnect(m_AutoReconnect);
    printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
    fflush(stdout);
    fflush(stderr);
    isScanning = true;
    delay(1000);
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
        return false;
	}
    if (!checkStatus()) {
        fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner.because of failure in scan mode.\n");
    }
    if (!turnOn()) {
        fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner. Because the motor falied to start.\n");
		
	}
    return ret;
	
}
