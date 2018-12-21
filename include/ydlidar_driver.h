#ifndef YDLIDAR_DRIVER_H
#define YDLIDAR_DRIVER_H
#include <stdlib.h>
#include <atomic>
#include "locker.h"
#include "serial/serial.h"
#include "thread.h"
#include "sockets/ActiveSocket.h"

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The YDLIDAR SDK requires a C++ compiler to be built"
#endif
#endif
#if !defined(_countof)
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifndef M_PI
#define M_PI 3.1415926
#endif

#define LIDAR_CMD_STOP                      0x65
#define LIDAR_CMD_SCAN                      0x60
#define LIDAR_CMD_FORCE_SCAN                0x61
#define LIDAR_CMD_RESET                     0x80
#define LIDAR_CMD_FORCE_STOP                0x00
#define LIDAR_CMD_GET_EAI                   0x55
#define LIDAR_CMD_GET_DEVICE_INFO           0x90
#define LIDAR_CMD_GET_DEVICE_HEALTH         0x92
#define LIDAR_ANS_TYPE_DEVINFO              0x4
#define LIDAR_ANS_TYPE_DEVHEALTH            0x6
#define LIDAR_CMD_SYNC_BYTE                 0xA5
#define LIDAR_CMDFLAG_HAS_PAYLOAD           0x80
#define LIDAR_ANS_SYNC_BYTE1                0xA5
#define LIDAR_ANS_SYNC_BYTE2                0x5A
#define LIDAR_ANS_TYPE_MEASUREMENT          0x81
#define LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_SYNC_QUALITY_SHIFT  8
#define LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1
#define LIDAR_RESP_MEASUREMENT_DISTANCE_SHIFT    2


#define LIDAR_CMD_RUN_POSITIVE             0x06
#define LIDAR_CMD_RUN_INVERSION            0x07
#define LIDAR_CMD_SET_AIMSPEED_ADDMIC      0x09
#define LIDAR_CMD_SET_AIMSPEED_DISMIC      0x0A
#define LIDAR_CMD_SET_AIMSPEED_ADD         0x0B
#define LIDAR_CMD_SET_AIMSPEED_DIS         0x0C
#define LIDAR_CMD_GET_AIMSPEED             0x0D

#define LIDAR_CMD_SET_SAMPLING_RATE        0xD0
#define LIDAR_CMD_GET_SAMPLING_RATE        0xD1
#define LIDAR_STATUS_OK                    0x0
#define LIDAR_STATUS_WARNING               0x1
#define LIDAR_STATUS_ERROR                 0x2

#define LIDAR_CMD_ENABLE_LOW_POWER         0x01
#define LIDAR_CMD_DISABLE_LOW_POWER        0x02
#define LIDAR_CMD_STATE_MODEL_MOTOR        0x05
#define LIDAR_CMD_ENABLE_CONST_FREQ        0x0E
#define LIDAR_CMD_DISABLE_CONST_FREQ       0x0F

#define LIDAR_CMD_SAVE_SET_EXPOSURE         0x94
#define LIDAR_CMD_SET_LOW_EXPOSURE          0x95
#define LIDAR_CMD_ADD_EXPOSURE       	    0x96
#define LIDAR_CMD_DIS_EXPOSURE       	    0x97

#define LIDAR_CMD_SET_HEART_BEAT        0xD9
#define LIDAR_CMD_SET_SETPOINTSFORONERINGFLAG  0xae

#define PackageSampleMaxLngth 0x100
typedef enum {
  CT_Normal = 0,
  CT_RingStart  = 1,
  CT_Tail,
} CT;
#define Node_Default_Quality (10)
#define Node_Sync 1
#define Node_NotSync 2
#define PackagePaidBytes 10
#define PH 0x55AA

#if defined(_WIN32)
#pragma pack(1)
#endif


struct touch_info {
  uint64_t    stamp;      /**< 当前触点时间戳. */
  uint16_t    touchid;    /**< 屏幕box中当前触点ID. */
  bool        isvalid;     /**< 当前点是否在设定的屏幕范围内. */
  bool        new_frame;  /**<是否是新的一帧激光. */
  float       screen_x;   /**<屏幕坐标系X(mm). */
  float       screen_y;   /**<屏幕坐标系Y(mm). */
  float       laser_x;    /**<雷达坐标系X(mm). */
  float       laser_y;   /**<雷达坐标系Y(mm). */
} __attribute__((packed)) ;




struct node_info {
  uint8_t    sync_flag;
  uint16_t    sync_quality;
  uint16_t   angle_q6_checkbit;
  uint16_t   distance_q;
  uint64_t   stamp;
} __attribute__((packed)) ;

struct PackageNode {
  uint8_t PakageSampleQuality;
  uint16_t PakageSampleDistance;
} __attribute__((packed));

struct node_package {
  uint16_t  package_Head;
  uint8_t   package_CT;
  uint8_t   nowPackageNum;
  uint16_t  packageFirstSampleAngle;
  uint16_t  packageLastSampleAngle;
  uint16_t  checkSum;
  PackageNode  packageSample[PackageSampleMaxLngth];
} __attribute__((packed)) ;

struct node_packages {
  uint16_t  package_Head;
  uint8_t   package_CT;
  uint8_t   nowPackageNum;
  uint16_t  packageFirstSampleAngle;
  uint16_t  packageLastSampleAngle;
  uint16_t  checkSum;
  uint16_t  packageSampleDistance[PackageSampleMaxLngth];
} __attribute__((packed)) ;


struct device_info {
  uint8_t   model; /**< 雷达型号. */
  uint16_t  firmware_version; /**< 固件版本号. */
  uint8_t   hardware_version; /**< 硬件版本号. */
  uint8_t   serialnum[16];    /**< 系列号. */
} __attribute__((packed)) ;

struct device_health {
  uint8_t   status; /**< 健康状体. */
  uint16_t  error_code; /**< 错误代码. */
} __attribute__((packed))  ;

struct sampling_rate {
  uint8_t rate;	/**< 采样频率. */
} __attribute__((packed))  ;

struct scan_frequency {
  uint32_t frequency;	/**< 扫描频率. */
} __attribute__((packed))  ;

struct scan_rotation {
  uint8_t rotation;
} __attribute__((packed))  ;

struct scan_exposure {
  uint8_t exposure;	/**< 低光功率模式. */
} __attribute__((packed))  ;

struct scan_heart_beat {
  uint8_t enable;	/**< 掉电保护状态. */
} __attribute__((packed));

struct scan_points {
  uint8_t flag;
} __attribute__((packed))  ;

struct function_state {
  uint8_t state;
} __attribute__((packed))  ;

struct cmd_packet {
  uint8_t syncByte;
  uint8_t cmd_flag;
  uint8_t size;
  uint8_t data;
} __attribute__((packed)) ;

struct lidar_ans_header {
  uint8_t  syncByte1;
  uint8_t  syncByte2;
  uint32_t size: 30;
  uint32_t subType: 2;
  uint8_t  type;
} __attribute__((packed));


struct LaserPose {
  float   x;      /**< 雷达在屏幕坐标系中的X位置(mm). */
  float   y;      /**< 雷达在屏幕坐标系中的Y位置(mm). */
  float   theta;  /**< 雷达在屏幕坐标系中的朝向(度). */
  bool    reversion; /**< 雷达表面朝向(false: 朝外, true:朝里). */
};

using namespace std;
using namespace serial;

namespace ydlidar {

class YDlidarDriver {
 public:
  /**
  * A constructor.
  * A more elaborate description of the constructor.
  */
  explicit YDlidarDriver(uint8_t drivertype = DEVICE_DRIVER_TYPE_SERIALPORT);

  /**
  * A destructor.
  * A more elaborate description of the destructor.
  */
  virtual ~YDlidarDriver();

  /**
  * @brief 连接雷达 \n
  * 连接成功后，必须使用::disconnect函数关闭
  * @param[in] port_path    串口号
  * @param[in] fileMode    波特率，YDLIDAR雷达有以下几个波特率：
  *     115200 F4, G4C, S4A
  *     128000 X4
  *     153600 S4B
  *     230600 F4PRO, G4
  * @return 返回连接状态
  * @retval 0     成功
  * @retval < 0   失败
  * @note连接成功后，必须使用::disconnect函数关闭
  * @see 函数::YDlidarDriver::disconnect (“::”是指定有连接功能,可以看文档里的disconnect变成绿,点击它可以跳转到disconnect.)
  */
  result_t connect(const char *port_path, uint32_t baudrate);

  /**
  * @brief 断开雷达连接
  */
  void disconnect();

  /**
  * @brief 获取当前SDK版本号 \n
  * 静态函数
  * @return 返回当前SKD 版本号
  */
  static std::string getSDKVersion();

  /**
  * @brief 扫图状态 \n
  * @return 返回当前雷达扫图状态
  * @retval true     正在扫图
  * @retval false    扫图关闭
  */
  const bool isscanning() const;

  /**
  * @brief 连接雷达状态 \n
  * @return 返回连接状态
  * @retval true     成功
  * @retval false    失败
  */
  const bool isconnected() const;

  /**
  * @brief 设置雷达是否带信号质量 \n
  * 连接成功后，必须使用::disconnect函数关闭
  * @param[in] isintensities    是否带信号质量:
  *     true	带信号质量
  *	  false 无信号质量
  * @note只有S4B雷达支持带信号质量, 别的型号雷达暂不支持
  */
  void setIntensities(const bool isintensities);

  /**
  * @brief 获取当前雷达掉电保护功能 \n
  * @return 返回掉电保护是否开启
  * @retval true     掉电保护开启
  * @retval false    掉电保护关闭
  */
  const bool getHeartBeat() const;

  /**
  * @brief 设置雷达掉电保护使能 \n
  * @param[in] enable    是否开启掉电保护:
  *     true	开启
  *	  false 关闭
  * @note只有(G4, G4C, F4PRO)雷达支持掉电保护功能, 别的型号雷达暂不支持
  */
  void setHeartBeat(const bool enable);

  /**
  * @brief 设置雷达异常自动重新连接 \n
  * @param[in] enable    是否开启自动重连:
  *     true	开启
  *	  false 关闭
  */
  void setAutoReconnect(const bool &enable);


  /**
  * @brief 发送掉电保护命令 \n
  * @return 返回执行结果
  * @retval RESULT_OK       发送成功
  * @retval RESULT_FAILE    发送失败
  * @note只有(G4, G4C, F4PRO)雷达支持掉电保护功能, 别的型号雷达暂不支持
  */
  result_t sendHeartBeat();

  /**
  * @brief 设置屏幕所在区域大小 \n
  * @return 无
  * @param[in] max_x    屏幕最大X轴值
  * @param[in] max_y    屏幕最大Y轴值
  * @param[in] min_x    屏幕最小X轴值
  * @param[in] min_y    屏幕最小Y轴值
  * @note左上角是屏幕坐标原点, X轴右, Y轴朝下
  *
  *   ||----------------\ X
  *   ||----------------/
  *   ||
  *   ||
  *   ||
  *   ||
  *   \/
  *    Y
  */
  void setScreenBox(const float &max_x, const float &max_y, const float &min_x, const float &min_y);

  /**
  * @brief 设置雷达在屏幕坐标系中的位置 \n
  * @return 无
  * @param[in] LaserPose   雷达位置
  */
  void setLaserPose(const LaserPose &pose);

  /**
  * @brief 获取雷达设备健康状态 \n
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_FAILE or RESULT_TIMEOUT   获取失败
  */
  result_t getHealth(device_health &health, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 获取雷达设备信息 \n
  * @param[in] info     设备信息
  * @param[in] timeout  超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_FAILE or RESULT_TIMEOUT   获取失败
  */
  result_t getDeviceInfo(device_info &info, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 开启扫描 \n
  * @param[in] force    扫描模式
  * @param[in] timeout  超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       开启成功
  * @retval RESULT_FAILE    开启失败
  * @note 只用开启一次成功即可
  */
  result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;

  /**
  * @brief 关闭扫描 \n
  * @return 返回执行结果
  * @retval RESULT_OK       关闭成功
  * @retval RESULT_FAILE    关闭失败
  */
  result_t stop();


  /**
  * @brief 获取激光在屏幕box中的数据 \n
  * @param[in] pointbuffer 激光点在屏幕上的坐标信息
  * @param[in] count      box中有效激光点数
  * @param[in] timeout    超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_FAILE    获取失败
  * @note 获取之前，必须使用::startScan函数开启扫描
  */
  result_t grabScanData(touch_info *pointbuffer, size_t &count, uint32_t timeout = DEFAULT_TIMEOUT) ;



  /**
  * @brief 重置激光雷达 \n
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作, 如果在扫描中调用::stop函数停止扫描
  */
  result_t reset(uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 打开电机 \n
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  */
  result_t startMotor();

  /**
  * @brief 关闭电机 \n
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  */
  result_t stopMotor();


  /**
  * @brief 获取激光雷达当前扫描频率 \n
  * @param[in] frequency    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t getScanFrequency(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 设置增加扫描频率1HZ \n
  * @param[in] frequency    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setScanFrequencyAdd(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 设置减小扫描频率1HZ \n
  * @param[in] frequency    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setScanFrequencyDis(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 设置增加扫描频率0.1HZ \n
  * @param[in] frequency    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setScanFrequencyAddMic(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 设置减小扫描频率0.1HZ \n
  * @param[in] frequency    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setScanFrequencyDisMic(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 获取激光雷达当前采样频率 \n
  * @param[in] frequency    采样频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t getSamplingRate(sampling_rate &rate, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 设置激光雷达当前采样频率 \n
  * @param[in] frequency    采样频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setSamplingRate(sampling_rate &rate, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 设置电机顺时针旋转 \n
  * @param[in] rotation    旋转方向
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setRotationPositive(scan_rotation &rotation, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 设置电机逆顺时针旋转 \n
  * @param[in] rotation    旋转方向
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t setRotationInversion(scan_rotation &rotation, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 低功耗使能 \n
  * @param[in] state    低功耗状态
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作,低功耗关闭,关闭后 G4 在空闲模式下电\n
  * 机和测距单元仍然工作
  */
  result_t enableLowerPower(function_state &state, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 关闭低功耗 \n
  * @param[in] state    低功耗状态
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作,关闭后 G4 在空闲模式下电\n
  * 机和测距单元仍然工作
  */
  result_t disableLowerPower(function_state &state, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 获取电机状态 \n
  * @param[in] state    电机状态
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t getMotorState(function_state &state, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 开启恒频功能 \n
  * @param[in] state    	  恒频状态
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t enableConstFreq(function_state &state, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 关闭恒频功能 \n
  * @param[in] state    	  恒频状态
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
  result_t disableConstFreq(function_state &state, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 保存当前激光曝光值 \n
  * @param[in] low_exposure    低光功能状态
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作, 当前操作需在非低光功率模式下, \n
  * 只有S4雷达支持此功能
  */
  result_t setSaveLowExposure(scan_exposure &low_exposure, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 设置低光功率模式 \n
  * @param[in] low_exposure    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作, 当前操作是开关量,只有S4雷达支持此功能
  */
  result_t setLowExposure(scan_exposure &low_exposure, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 增加激光曝光值 \n
  * @param[in] exposure     曝光值
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作,只有S4雷达支持此功能
  */
  result_t setLowExposureAdd(scan_exposure &exposure, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 减小激光曝光值 \n
  * @param[in] exposure     曝光值
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作,只有S4雷达支持此功能
  */
  result_t setLowExposurerDis(scan_exposure &exposure, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 设置雷达掉电保护状态 \n
  * @param[in] beat    	  掉电保护状态
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作, 当前操作是开关量, (G4, G4C, F4PRO)支持
  */
  result_t setScanHeartbeat(scan_heart_beat &beat, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 设置扫描一圈固定激光点数 \n
  * @param[in] points    	  固定点数状态
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作, 当前操作是开关量,只有S4雷达支持此功能
  */
  result_t setPointsForOneRingFlag(scan_points &points, uint32_t timeout = DEFAULT_TIMEOUT);


 protected:

  /**
  * @brief 创建解析雷达数据线程 \n
  * @note 创建解析雷达数据线程之前，必须使用::startScan函数开启扫图成功
  */
  result_t createThread();

  /**
  * @brief 重新连接开启扫描 \n
  * @param[in] force    扫描模式
  * @param[in] timeout  超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       开启成功
  * @retval RESULT_FAILE    开启失败
  * @note sdk 自动重新连接调用
  */
  result_t startAutoScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;


  /**
  * @brief 解包激光数据 \n
  * @param[in] point 解包后激光点在屏幕坐标系中的信息
  * @param[in] timeout     超时时间
  */
  result_t waitPackage(touch_info *point, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 等待激光数据 \n
  * @param[in] pointbuffer 激光点在屏幕上的坐标信息
  * @param[in] count        激光在屏幕坐标系中的大小
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_TIMEOUT  等待超时
  * @retval RESULT_FAILE    失败
  */
  result_t waitScanData(touch_info *pointbuffer, size_t &count, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 激光数据解析线程 \n
  */
  int cacheScanData();

  /**
  * @brief 发送数据到雷达 \n
  * @param[in] cmd 	 命名码
  * @param[in] payload      payload
  * @param[in] payloadsize      payloadsize
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  */
  result_t sendCommand(uint8_t cmd, const void *payload = NULL, size_t payloadsize = 0);

  /**
  * @brief 等待激光数据包头 \n
  * @param[in] header 	 包头
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_TIMEOUT  等待超时
  * @retval RESULT_FAILE    获取失败
  * @note 当timeout = -1 时, 将一直等待
  */
  result_t waitResponseHeader(lidar_ans_header *header, uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief 等待固定数量串口数据 \n
  * @param[in] data_count 	 等待数据大小
  * @param[in] timeout    	 等待时间
  * @param[in] returned_size   实际数据大小
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_TIMEOUT  等待超时
  * @retval RESULT_FAILE    获取失败
  * @note 当timeout = -1 时, 将一直等待
  */
  result_t waitForData(size_t data_count, uint32_t timeout = DEFAULT_TIMEOUT,
                       size_t *returned_size = NULL);

  /**
  * @brief 获取串口数据 \n
  * @param[in] data 	 数据指针
  * @param[in] size    数据大小
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_FAILE    获取失败
  */
  result_t getData(uint8_t *data, size_t size);

  /**
  * @brief 串口发送数据 \n
  * @param[in] data 	 发送数据指针
  * @param[in] size    数据大小
  * @return 返回执行结果
  * @retval RESULT_OK       发送成功
  * @retval RESULT_FAILE    发送失败
  */
  result_t sendData(const uint8_t *data, size_t size);


  /**
  * @brief 关闭数据获取通道 \n
  */
  void disableDataGrabbing();

  /**
  * @brief 设置串口DTR \n
  */
  void setDTR();

  /**
  * @brief 清除串口DTR \n
  */
  void clearDTR();

  /**
  *@brief 判断点是否在屏幕中 \n
  * @param[in] x 	激光屏幕坐标x值
  * @param[in] y    激光屏幕坐标y值
  *
  */
  bool inBox(const float &x, const float &y);


 public:
  std::atomic<bool>     isConnected;  ///< 串口连接状体
  std::atomic<bool>     isScanning;   ///< 扫图状态
  std::atomic<bool>     isHeartbeat;  ///< 掉电保护状态
  std::atomic<bool>     isAutoReconnect;  ///< 异常自动从新连接
  std::atomic<bool>     isAutoconnting; ///< 是否正在自动连接中

  enum {
    DEFAULT_TIMEOUT = 2000,    /**< 默认超时时间. */
    DEFAULT_HEART_BEAT = 1000, /**< 默认检测掉电功能时间. */
    MAX_SCAN_NODES = 2048,	   /**< 最大扫描点数. */
    DEFAULT_TIMEOUT_COUNT = 7,
  };
  enum {
    YDLIDAR_F4 = 1, /**< F4雷达型号代号. */
    YDLIDAR_T1 = 2, /**< T1雷达型号代号. */
    YDLIDAR_F2 = 3, /**< F2雷达型号代号. */
    YDLIDAR_S4 = 4, /**< S4雷达型号代号. */
    YDLIDAR_G4 = 5, /**< G4雷达型号代号. */
    YDLIDAR_X4 = 6, /**< X4雷达型号代号. */
    YDLIDAR_F4PRO = 6, /**< F4PRO雷达型号代号. */
    YDLIDAR_G4C = 9, /**< G4C雷达型号代号. */

  };
  touch_info     touch_point_buf[2048];  ///< 触摸点信息
  size_t         touch_point_count;      ///< 触摸点数
  Event          _dataEvent;			 ///< 数据同步事件
  Locker         _lock;				///< 线程锁
  Locker         _serial_lock;       ///< 串口锁
  Locker         _plock;				///< 参数线程锁
  Thread 	       _thread;				///< 线程id

 private:
  int PackageSampleBytes;             ///< 一个包包含的激光点数
  ChannelDevice *_serial;			    ///< 串口
  bool m_intensities;					///< 信号质量状体
  int _sampling_rate;					///< 采样频率
  int model;							///< 雷达型号
  uint32_t _baudrate;					///< 波特率
  bool isSupportMotorCtrl;			///< 是否支持电机控制
  uint64_t m_ns;						///< 时间戳
  uint32_t m_pointTime;				///< 激光点直接时间间隔
  uint32_t trans_delay;				///< 串口传输一个byte时间

  node_package package;
  node_packages packages;

  uint16_t package_Sample_Index;
  float IntervalSampleAngle;
  float IntervalSampleAngle_LastPackage;
  uint16_t FirstSampleAngle;
  uint16_t LastSampleAngle;
  uint16_t CheckSun;

  uint16_t CheckSunCal;
  uint16_t SampleNumlAndCTCal;
  uint16_t LastSampleAngleCal;
  bool CheckSunResult;
  uint16_t Valu8Tou16;
  uint16_t touchid;///< 触点ID

  float screen_max_x;///< 屏幕X轴最大值
  float screen_max_y;///< 屏幕Y轴最大值
  float screen_min_x;///< 屏幕X轴最小值
  float screen_min_y;///< 屏幕Y轴最小值

  LaserPose laser_pose; ///< 雷达在屏幕坐标系的位置

  bool  reversion;///< 雷达反转安装
  std::string serial_port;///< 雷达端口
  uint8_t     m_driver_type;
};
}

#endif // YDLIDAR_DRIVER_H
