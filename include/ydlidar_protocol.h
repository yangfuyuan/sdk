
#pragma once

#include <v8stdint.h>
#include <vector>
#include <ydlidar_cmd.h>

#if defined(_WIN32)
#pragma pack(1)
#endif

struct node_info {
    uint8_t    sync_flag;  //sync flag
    uint8_t    sync_quality;//!信号质量
    uint16_t   angle_q6_checkbit; //!测距点角度
    uint16_t   distance_q2; //! 当前测距点距离
    uint64_t   stamp; //! 时间戳
    uint8_t    scan_frequence;//! 特定版本此值才有效,无效值是0
} __attribute__((packed)) ;

struct PackageNode {
    uint8_t PakageSampleQuality;
    uint16_t PakageSampleDistance;
}__attribute__((packed));

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


struct device_info{
    uint8_t   model; ///< 雷达型号
    uint16_t  firmware_version; ///< 固件版本号
    uint8_t   hardware_version; ///< 硬件版本号
    uint8_t   serialnum[16];    ///< 系列号
} __attribute__((packed)) ;

struct device_health {
    uint8_t   status; ///< 健康状体
    uint16_t  error_code; ///< 错误代码
} __attribute__((packed))  ;

struct sampling_rate {
    uint8_t rate;	///< 采样频率
} __attribute__((packed))  ;

struct scan_frequency {
    uint32_t frequency;	///< 扫描频率
} __attribute__((packed))  ;

struct scan_rotation {
    uint8_t rotation;
} __attribute__((packed))  ;

struct scan_exposure {
    uint8_t exposure;	///< 低光功率模式
} __attribute__((packed))  ;

struct scan_heart_beat {
    uint8_t enable;	///< 掉电保护状态
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
    uint32_t size:30;
    uint32_t subType:2;
    uint8_t  type;
} __attribute__((packed));


//! A struct for returning configuration from the YDLIDAR
struct LaserConfig {
    //! Start angle for the laser scan [rad].  0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
    float min_angle;
    //! Stop angle for the laser scan [rad].   0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
    float max_angle;
    //! Scan resolution [rad].
    float ang_increment;
    //! Scan resoltuion [s]
    float time_increment;
    //! Time between scans
    float scan_time;
    //! Minimum range [m]
    float min_range;
    //! Maximum range [m]
    float max_range;
    //! Range Resolution [m]
    float range_res;
};


//! A struct for returning laser readings from the YDLIDAR
//! currentAngle = min_angle + ang_increment*index
//! for( int i =0; i < ranges.size(); i++) {
//!     double currentAngle = config.min_angle + i*config.ang_increment;
//!     double currentDistance = ranges[i];
//! }
//!
//!
//!
struct LaserScan {
    //! Array of ranges
    std::vector<float> ranges;
    //! Array of intensities
    std::vector<float> intensities;
    //! Self reported time stamp in nanoseconds
    uint64_t self_time_stamp;
    //! System time when first range was measured in nanoseconds
    uint64_t system_time_stamp;
    //! Configuration of scan
    LaserConfig config;
};
