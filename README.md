YDLIDAR SDK PACKAGE V1.3.7
=====================================================================

SDK [test](https://github.com/yangfuyuan/sdk/tree/samsung) application for YDLIDAR

Visit EAI Website for more details about [YDLIDAR](http://www.ydlidar.com/) .

How to build YDLIDAR SDK samples
=====================================================================
    $ git clone https://github.com/yangfuyuan/sdk
    $ cd sdk
    $ git checkout samsung
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

    $ ./ydlidar_test LidarAngleCalibration.ini
    $Please enter the lidar serial port:/dev/ttyUSB0
    $Please enter the lidar serial baud rate:230400
    &Please enter the lidar intensity:0

windows:

    $ ydlidar_test.exe LidarAngleCalibration.ini
    $Please enter the lidar serial port:/dev/ttyUSB0
    $Please enter the lidar serial baud rate:230400
    &Please enter the lidar intensity:0

=====================================================================

You should see YDLIDAR's scan result in the console:

      [YDLIDAR]:SDK Version: 1.3.7

      [YDLIDAR]:Lidar running correctly ! The health status: good

      [YDLIDAR] Connection established in [/dev/ttyUSB0][230400]:

      Firmware version: 1.1

      Hardware version: 3

      Model: G2-SS-1

      Serial: 2018072000000203

      [YDLIDAR INFO] Current Sampling Rate : 9K

      [YDLIDAR INFO] Successfully obtained the calibration value[-0.500000] from the calibration file[LidarAngleCalibration.ini]

      [YDLIDAR INFO] Current AngleOffset : -0.500000°

      [YDLIDAR INFO] Current Scan Frequency : 8.000000Hz

      [YDLIDAR INFO] Now YDLIDAR is scanning ......

      Scan received: 801 ranges

      Scan received: 938 ranges



Upgrade Log
=====================================================================
2018-10-15 version:1.3.7

   1.add input angle calibration file.
   
   2.remove network.

2018-10-15 version:1.3.6

   1.add network support.

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
