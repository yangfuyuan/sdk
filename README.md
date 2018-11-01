YDLIDAR SDK PACKAGE V1.3.6
=====================================================================

SDK [test](https://github.com/yangfuyuan/sdk/tree/s2) application for YDLIDAR

Visit EAI Website for more details about [YDLIDAR](http://www.ydlidar.com/) .

How to build YDLIDAR SDK samples
=====================================================================
   
    $ git clone https://github.com/yangfuyuan/sdk
    
    $ cd sdk
    
    $ git checkout s2
    
    $ cd ..
    
    $ mkdir build
    
    $ cd build
    
    $ cmake ../sdk
    
    $ make			###linux
    
    $ vs open Project.sln	###windows
    
How to run YDLIDAR SDK samples
=====================================================================
    $ cd samples

linux:

    $ ./ydlidar_test
    $Please enter the lidar serial port :/dev/ttyUSB0
    $Please enter the lidar serial baud rate:115200

windows:

    $ ydlidar_test.exe
    $Please enter the lidar serial port:COM3
    $Please enter the lidar serial baud rate:115200


You should see YDLIDAR's scan result in the console:

     	YDLIDAR C++ TEST
     	
	[YDLIDAR INFO] Now YDLIDAR SDK VERSION: 1.3.6
	
	fhs_lock: creating lockfile:      11796

	[YDLIDAR INFO] Connection established in /dev/ttyUSB0[115200]:
	
	[YDLIDAR INFO] Now YDLIDAR is scanning ......
	
	Scan received: 272 ranges
	


Lidar point data structure
=====================================================================

data structure:

    struct node_info {

       uint8_t    sync_quality;//!intensity

       uint16_t   angle_q6_checkbit; //!angle

       uint16_t   distance_q2; //! distance

       uint64_t   stamp; //! time stamp

       uint8_t    scan_frequence;//! current_frequence = scan_frequence/10.0, If the current value equals zero, it is an invalid value
 
    } __attribute__((packed)) ;

example:

    if(data[i].scan_frequence != 0) {

        current_frequence = data[i].scan_frequence/10.0;
    }

    current_time_stamp = data[i].stamp;

    current_distance = data[i].distance_q2/4.f;

    current_angle = ((data[i].angle_q6_checkbit>>LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);

    current_intensity = (float)(data[i].sync_quality);

    ###note:current_frequence = data[0].scan_frequence/10.0.

    ###if the current_frequence value equals zero, it is an invalid value.

code:
        
        void ParseScan(node_info* data, const size_t& size) {

            double current_frequence, current_distance, current_angle, current_intensity;

            uint64_t current_time_stamp;

            for (size_t i = 0; i < size; i++ ) {

                if( data[i].scan_frequence != 0) {

                    current_frequence =  data[i].scan_frequence;//or current_frequence = data[0].scan_frequence

                }

                current_time_stamp = data[i].stamp;

                current_angle = ((data[i].angle_q6_checkbit>>LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);//LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT equals 8

                current_distance =  data[i].distance_q2/4.f;

                current_intensity = (float)(data[i].sync_quality);

            }

            if (current_frequence != 0 ) {

                printf("current lidar scan frequency: %f\n", current_frequence);

            } else {

                printf("Current lidar does not support return scan frequency\n");

            }
        }


Upgrade Log
=====================================================================

2018-11-1 version:1.3.6

   1.remove unwanted code.

2018-05-23 version:1.3.4

   1.add automatic reconnection if there is an exception

   2.add serial file lock.

2018-05-14 version:1.3.3

   1.add the heart function constraint.

   2.add packet type with scan frequency support.

2018-04-16 version:1.3.2

   1.add multithreading support.

2018-04-16 version:1.3.1

   1.Compensate for each laser point timestamp.

