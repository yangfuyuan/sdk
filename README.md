YDLIDAR SDK BIG SCREEN INTERACTION PACKAGE V1.1.0
=====================================================================

SDK test application for YDLIDAR

Visit EAI Website for more details about YDLIDAR.

How to build YDLIDAR SDK samples
=====================================================================
    1) Clone this project to your computer folder
    2) Running cmake to build ydlidar_test
    
How to run YDLIDAR SDK samples
=====================================================================
    --$ cd samples

linux:

    --$ ./ydlidar_test

windows:

    --$ ydlidar_test.exe

You should see YDLIDAR's scan result in the console


Upgrade Log
=====================================================================

2018-05-05 version:1.1.0

   1.increase mouse event c++ mutil-platform library.

   2.remove python mouse event.

   3.add simple right mouse button, left button, middle button demo.


2018-05-02 version:1.0.0

   1.Output screen corrdinates.


坐标系统
=====================================================================

<p align="middle">
    <a href="https://github.com/yangfuyuan
    " target="_blank"><img src="image.png"
    alt="IMAGE ALT TEXT HERE" width="705" height="823" border="10"/></a>
</p>

参数设定样例:

上图安装方式1:

	屏幕X轴最小值 = 0mm;

	屏幕 Y 轴最小值 = 0mm;

	屏幕X轴最大值 = 1920mm;

	屏幕Y轴最大值 = 1080mm;

	雷达安装位置X值 = 900mm;

	雷达安装位置Y值 = -100mm;

	雷达安装角度Theta = -90;//雷达坐标系相对屏幕坐标系逆时针旋转了90度.

	CYdLidar laser;

	laser.setMax_x(1920);
    
	laser.setMax_y(1080);
    
	laser.setMin_x(0);
    
	laser.setMin_y(0);

	LaserPose pose;

	pose.x = 900;

	pose.y = -100;

	pose.theta = -90;

	pose.reversion = false; //雷达表面朝外

	laser.setpose(pose);

上图安装方式2:

	屏幕X轴最小值 = 0mm;

	屏幕 Y 轴最小值 = 0mm;

	屏幕X轴最大值 = 1920mm;

	屏幕Y轴最大值 = 1080mm;

	雷达安装位置X值 = -100mm;

	雷达安装位置Y值 = 500mm;

	雷达安装角度Theta = -180;//雷达坐标系相对屏幕坐标系逆时针旋转了90度.

	CYdLidar laser;

	laser.setMax_x(1920);
    
	laser.setMax_y(1080);
    
	laser.setMin_x(0);
    
	laser.setMin_y(0);

	LaserPose pose;

	pose.x = -100;

	pose.y = 500;

	pose.theta = -180;

	pose.reversion = false; //雷达表面朝外

	laser.setpose(pose);


####备注:雷达安装位置和角度根据实际情况设定, 注意实际屏幕和操作屏幕的换算比例. 如果安装角度是120度, 直接把雷达安装角度设置为120, 不用更改雷达零点.
