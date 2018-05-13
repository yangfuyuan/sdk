YDLIDAR SDK PACKAGE V1.3.2
=====================================================================

SDK [test](https://github.com/yangfuyuan/sdk/tree/non-singleton) application for YDLIDAR

Visit EAI Website for more details about [YDLIDAR](http://www.ydlidar.com/) .

How to build YDLIDAR SDK samples
=====================================================================
    $ git clone https://github.com/yangfuyuan/sdk
    $ cd sdk
    $ git checkout non-singleton
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
    $Please enter the lidar port:/dev/ttyUSB0
    $Please enter the lidar baud rate:230400

windows:

    $ ydlidar_test.exe
    $Please enter the lidar port:COM3
    $Please enter the lidar baud rate:230400

=====================================================================
You should see YDLIDAR's scan result in the console:

    Yd Lidar running correctly ! The health status: good
    [YDLIDAR] Connection established in [/dev/ttyUSB0]:
    Firmware version: 2.0.9
    Hardware version: 2
    Model: G4
    Serial: 2018022700000003
    [YDLIDAR INFO] Current Sampling Rate : 9K
    [YDLIDAR INFO] Current Scan Frequency : 7.400000Hz
    [YDLIDAR INFO] Now YDLIDAR is scanning ......
    Scan received: 43 ranges
    Scan received: 1361 ranges
    Scan received: 1412 ranges

Upgrade Log
=====================================================================

2018-04-16 version:1.3.2

   1.add multithreading support.

2018-04-16 version:1.3.1

   1.Compensate for each laser point timestamp.

