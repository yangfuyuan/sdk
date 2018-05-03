YDLIDAR SDK BIG SCREEN INTERACTION PACKAGE V1.0.0
=====================================================================

SDK test application for YDLIDAR

Visit EAI Website for more details about YDLIDAR.

How to build YDLIDAR SDK samples
=====================================================================
    1) Clone this project to your computer folder
    2) Clone this project to your computer folder
    2) Running cmake to build ydlidar_test
    
How to run YDLIDAR SDK samples
=====================================================================
    --$ cd samples
    --$ ./ydlidar_test /dev/ttyUSB0 230400 0

You should see YDLIDAR's scan result in the console


Upgrade Log
=====================================================================

2018-05-02 version:1.0.0

   1.Output screen corrdinates.

Dependencies:

1.python 

2.pymouse(samples)

linux:
   1.pip install pymouse


windows:

Install required dependencies:

   1.pip install pywin32
  
   2.pip install pyhook ##failed, go to http://www.lfd.uci.edu/~gohlke/pythonlibs/#pyhook

   3.pip install pymouse

Here is how to install pyHook:

   1.You can find the download link here http://www.lfd.uci.edu/~gohlke/pythonlibs/#pyhook

   2.If you have python 32bit you want pyhook 32 bit (Download #1 for Python 2.7).

   3.If you have python 64bit you want pyHook 64 bit (Download #2 for Python 2.7).

   4.Extract the zip file.

   5.In the command prompt navigate to the folder where you extracted the .zip.

   6.Type the command 'pip install pyHook-1.5.1-cp27-none-win32.whl' for the 32 bit version or 'pip install pyHook-1.5.1-cp27-none-win_amd64.whl'


Error:

   1. ModuleNotFoundError: No module named 'windows'

   Under pymoused __init__，
	elif sys.platform == 'win32':
		from windows import PyMouse, PyMouseEvent
   Change to 'from pymouse.windows import PyMouse, PyMouseEvent

