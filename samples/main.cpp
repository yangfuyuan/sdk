
#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <signal.h>
#include <memory>
#include <unistd.h>
using namespace std;
using namespace ydlidar;
CYdLidar laser;
static bool running = false;

#ifdef _WIN32
#include <Windows.h>

inline int GetScreenSize( int* x, int* y )
{
    *x = GetSystemMetrics( SM_CXSCREEN );
    *y = GetSystemMetrics( SM_CYSCREEN );
    return 0;
}

#else
#include <X11/Xlib.h>

inline int GetScreenSize(int *w, int*h)
{
    Display* pdsp = NULL;
    Screen* pscr = NULL;

    pdsp = XOpenDisplay( NULL );
    if ( !pdsp ) {
        fprintf(stderr, "Failed to open default display.\n");
        return -1;
    }

    pscr = DefaultScreenOfDisplay( pdsp );
    if ( !pscr ) {
        fprintf(stderr, "Failed to obtain the default screen of given display.\n");
        return -2;
    }

    *w = pscr->width;
    *h = pscr->height;

    XCloseDisplay( pdsp );
    return 0;
}
#endif

static void Stop(int signo)   
{  
    
    printf("Received exit signal\n");
    running = true;
     
}  

int main(int argc, char * argv[])
{

  bool showHelp  = argc>1 && !strcmp(argv[1],"--help");
	printf(" YDLIDAR C++ TEST\n");
    if (argc<4 || showHelp )
	{
      
			printf("Usage: %s <serial_port> <baudrate> <intensities>\n\n",argv[0]);
            printf("Example:%s /dev/ttyUSB0 115200 0\n\n",argv[0]);
			if (!showHelp)
			{				
                return -1;
			}
            else
                return 0;
    }

    const std::string port = string(argv[1]);
    const int baud =  atoi(argv[2]);
    const int intensities =  atoi(argv[3]);

    signal(SIGINT, Stop);
    signal(SIGTERM, Stop);

   int width, height;
   if(GetScreenSize(&width, &height) != 0){
       width = 1920;
       height = 1080;
   }
   float resolution_x =10; //电脑屏幕映射到操作屏上的分辨率, X轴上
   float resolution_y =10; //电脑屏幕映射到操作屏上的分辨率, Y轴上


  laser.setSerialPort(port);
  laser.setSerialBaudrate(baud);
  laser.setIntensities(intensities);
  laser.setMax_x(width/resolution_x);
  laser.setMax_y(height/resolution_y);
  laser.setMin_x(0);
  laser.setMin_y(0);
  LaserPose pose;
  pose.x = width/(2*resolution_x);
  pose.y = -260;
  pose.theta = -90;
  std::cout<<"pose: "<<pose.x<<std::endl;
  laser.setpose(pose);


  ////////////////////////////////////////////////////////////////////////////////////////////
  //                     G4雷达零点
  //                      |
  //                      |______

//|-------540----------|////                ------
//                //雷达安装位置//               |
//                                             | 260
//|----------------1920----------------------| |
  ////////////////////////////////////////////----
  // (0,0)                                  //  |
  //                                        //  |
  //                                        //  |
  //                                        //  |
  //                                        // 1080
  //                屏幕                    //  |
  //                                        //  |
  //                                        //  |
  //                                        //  |
  //                                        //  |
  ////////////////////////////////////////////-----
  std::vector<touch_info> oldPoints;
  int cnt = 0;
  bool click = false;
  laser.initialize();
  while(!running){
		bool hardError;
        std::vector<touch_info> outPoints;
        int x, y;
        if(laser.doProcessSimple(outPoints, hardError )){
            for (auto it = outPoints.begin(); it != outPoints.end(); it++) {
                touch_info point = *it;
                for (auto its = oldPoints.begin(); its != oldPoints.end(); its++) {
                    touch_info pt = *it;
                    if((pow(pt.screen_x - point.screen_x, 2) + pow(pt.screen_y - point.screen_y ,2)) < 100){
                        click = true;;
                        x = (int)point.screen_x*resolution_x;
                        y = (int)point.screen_y*resolution_y;
                    }

                }

            }

            oldPoints = outPoints;
            if(click){
                cnt++;
                char* buf;
                if(cnt>10){
                    if (asprintf(&buf, "scripts/pmouse.py click %i %i", x, y) < 0) {
                    }else{
                        string str(buf);
                        free(buf);
                        system(str.c_str()) ;
                        click = false;
                        cnt =0;
                    }

                }else{
                    if (asprintf(&buf, "scripts/pmouse.py move %i %i", x, y) < 0) {
                    }else{
                        string str(buf);
                        free(buf);
                        system(str.c_str()) ;
                        click = false;
                    }
                }
            }

		}

    usleep(50*1000);

		
	}
  laser.turnOff();
  laser.disconnecting();

  return 0;


}
