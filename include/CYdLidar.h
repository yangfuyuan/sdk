#pragma once
#include "ydlidar_driver.h"
#include <math.h>

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


using namespace ydlidar;

class CYdLidar
{
    PropertyBuilderByName(float,MaxRange,private)///< 设置和获取激光最大测距范围(m)
    PropertyBuilderByName(float,MinRange,private)///< 设置和获取激光最小测距范围(m)
    PropertyBuilderByName(float,MaxAngle,private)///< 设置和获取激光最大角度, 最大值180度(度)
    PropertyBuilderByName(float,MinAngle,private)///< 设置和获取激光最小角度, 最小值-180度(度)

    PropertyBuilderByName(bool,Intensities,private)///< 设置和获取激光带信号质量(只有S4B雷达支持)
    PropertyBuilderByName(bool,AutoReconnect, private)///< 设置异常是否开启重新连接
    PropertyBuilderByName(int,SerialBaudrate,private)///< 设置和获取激光通讯波特率
    PropertyBuilderByName(std::string,SerialPort,private)///< 设置和获取激光端口号

    PropertyBuilderByName(bool,EnableDebug, private)///< 设置是否开启调试把解析数据保存到文件


public:
	CYdLidar(); //!< Constructor
	virtual ~CYdLidar();  //!< Destructor: turns the laser off.

    bool initialize();  //!< Attempts to connect and turns the laser on. Raises an exception on error.

    // Return true if laser data acquistion succeeds, If it's not
    bool doProcessSimple(node_info *nodes, size_t& count, bool &hardwareError);
    /**
    * @brief 补偿激光角度 \n
    * 把角度限制在0到360度之间
    * @param[in] nodebuffer 激光点信息
    * @param[in] count      一圈激光点数
    * @return 返回执行结果
    * @retval true       成功
    * @retval false    失败
    * @note 补偿之前，必须使用::grabScanData函数获取激光数据成功
    */
    bool ascendScanData(node_info * nodebuffer, size_t count);

    //Turn on the motor enable
	bool  turnOn();  //!< See base class docs
    //Turn off the motor enable and close the scan
	bool  turnOff(); //!< See base class docs

    /** Returns true if the device is in good health, If it's not*/
	bool getDeviceHealth() const;

    /** Returns true if the device information is correct, If it's not*/
    bool getDeviceInfo(int &lidar_model);

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
    bool    isScanning;
    int     print_error;
    YDlidarDriver *lidarPtr;

};	// End of class

