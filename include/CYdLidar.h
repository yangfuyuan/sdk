
#ifndef CYDLIDAR_H
#define CYDLIDAR_H
#include "utils.h"
#include "ydlidar_driver.h"
#include <math.h>

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The YDLIDAR SDK requires a C++ compiler to be built"
#endif
#endif

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifndef M_PI
#define M_PI 3.1415926
#endif


#define PropertyBuilderByName(type, name, access_permission)\
    access_permission:\
        type m_##name;\
    public:\
    inline void set##name(type v) {\
        m_##name = v;\
    }\
    inline type get##name() {\
        return m_##name;\
}\

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace ydlidar;

class YDLIDAR_API CYdLidar {
  PropertyBuilderByName(float, Max_x, private)
  PropertyBuilderByName(float, Min_x, private)
  PropertyBuilderByName(float, Max_y, private)
  PropertyBuilderByName(float, Min_y, private)
  PropertyBuilderByName(LaserPose, pose, private)
  PropertyBuilderByName(int, ScanFrequency, private)

  PropertyBuilderByName(bool, Intensities, private)
  PropertyBuilderByName(bool, Exposure, private)
  PropertyBuilderByName(bool, HeartBeat, private)
  PropertyBuilderByName(bool, AutoReconnect, private)
  PropertyBuilderByName(int, SerialBaudrate, private)
  PropertyBuilderByName(int, SampleRate, private)
  PropertyBuilderByName(int, DeviceType, private)

  PropertyBuilderByName(std::string, SerialPort, private)


 public:
  CYdLidar(); //!< Constructor
  virtual ~CYdLidar();  //!< Destructor: turns the laser off.

  bool initialize();  //!< Attempts to connect and turns the laser on. Raises an exception on error.

  // Return true if laser data acquistion succeeds, If it's not
  bool doProcessSimple(std::vector<touch_info> &outPoints, bool &hardwareError);

  //Turn on the motor enable
  bool  turnOn();  //!< See base class docs
  //Turn off the motor enable and close the scan
  bool  turnOff(); //!< See base class docs

  /** Returns true if the device is in good health, If it's not*/
  bool getDeviceHealth() const;

  /** Returns true if the device information is correct, If it's not*/
  bool getDeviceInfo(int &type);

  /** Retruns true if the heartbeat function is set to heart is successful, If it's not*/
  bool checkHeartBeat() const;

  /** Retruns true if the scan frequency is set to user's frequency is successful, If it's not*/
  bool checkScanFrequency();

  //Turn off lidar connection
  void disconnecting(); //!< Closes the comms with the laser. Shouldn't have to be directly needed by the user

 protected:
  /** Returns true if communication has been established with the device. If it's not,
    *  try to create a comms channel.
    * \return false on error.
    */
  bool  checkCOMMs();

  /** Returns true if health status and device information has been obtained with the device. If it's not,
    * \return false on error.
    */
  bool  checkStatus();

  /** Returns true if the normal scan runs with the device. If it's not,
    * \return false on error.
    */
  bool checkHardware();



 private:
  bool isScanning;
  int show_error;
  YDlidarDriver *lidarPtr;

};	// End of class

#endif // CYDLIDAR_H


