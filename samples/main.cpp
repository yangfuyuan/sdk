
#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <signal.h>
#include <memory>
#include <unistd.h>
#include <array>
#include "kmeans.h"

using namespace std;
using namespace ydlidar;
using namespace ydlidar::math::detail;
using namespace ydlidar::math;

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

int main(int argc, char * argv[]) {


    bool showHelp  = argc>1 && !strcmp(argv[1],"--help");
	printf(" YDLIDAR C++ TEST\n");
    if (argc<4 || showHelp ) {
        printf("Usage: %s <serial_port> <baudrate> <intensities>\n\n",argv[0]);
        printf("Example:%s /dev/ttyUSB0 115200 0\n\n",argv[0]);
        if (!showHelp) {
            return -1;
        } else {
            return 0;
        }

    }

    typedef CArrayDouble<2>  CPointType;
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
    float resolution_x =1920/960.0; //电脑屏幕映射到操作屏上的分辨率, X轴上
    float resolution_y =1; //电脑屏幕映射到操作屏上的分辨率, Y轴上

    double pre_x, pre_y;

    laser.setSerialPort(port);
    laser.setSerialBaudrate(baud);
    laser.setIntensities(intensities);


    laser.setMax_x(width/resolution_x);
    laser.setMax_y(height/resolution_y);
    laser.setMin_x(0);
    laser.setMin_y(0);
    LaserPose pose;
    pose.x = -100;//width/(2*resolution_x);//G4在9k时盲区是<260mm, 8k<=240mm, 4k<=100mm
    pose.y = height/(2*resolution_y);
    pose.theta = 150;
    pose.reversion = true;//雷达朝向  朝里还是朝外, 朝里是true, 朝外是false
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

    int cnt = 0;
    laser.initialize();
    while(!running){
		bool hardError;
        std::vector<touch_info> outPoints;
        int x, y;
        if(laser.doProcessSimple(outPoints, hardError )){
            const size_t nClusters = 1;
            aligned_containers<CPointType>::vector_t  points;
            for (auto it = outPoints.begin(); it != outPoints.end(); it++) {
                touch_info point = *it;
                CPointType v;
                v[0] = point.screen_x;
                v[1] = point.screen_y;
                points.push_back(v);

            }

            // do k-means
            aligned_containers<CPointType>::vector_t	centers;

            vector<int>				assignments;
            const double cost = kmeanspp(nClusters, points, assignments, &centers);

            for(auto it = centers.begin(); it != centers.end(); it++) {
                x = (*it)[0]*resolution_x;
                y = (*it)[1]*resolution_y;
                //std::cout<<"center_x: "<<x<<"center_y: "<<y<<std::endl;
                if((pow(pre_x - x,2) + pow(pre_y-y, 2)) < pow(8*resolution_x, 2)) {
                    cnt++;
                    x = pre_x;
                    y = pre_y;
                }else{
                    pre_x = x;
                    pre_y = y;
                    cnt=0;
                }

                char* buf;
                if (cnt >5 ) {
                    if (asprintf(&buf, "scripts/pmouse.py click %i %i", x, y) < 0) {

                    } else {
                        string str(buf);
                        free(buf);
                        system(str.c_str()) ;
                        cnt =0;
                    }

                } else {
                    if (asprintf(&buf, "scripts/pmouse.py move %i %i", x, y) < 0) {

                    } else {
                        string str(buf);
                        free(buf);
                        system(str.c_str()) ;
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
