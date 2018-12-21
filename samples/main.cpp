
#include "include/CYdLidar.h"
#include <iostream>
#include <string>
#include <signal.h>
#include <chrono>
#include <thread>
#include <memory>
#include <array>
#include <regex>
#include "include/kmeans.h"
#include "input_lite/include/InputManager.h"
#include "input_lite/include/Input_Lite.h"




#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif

using namespace std;
using namespace ydlidar;
using namespace ydlidar::math::detail;
using namespace ydlidar::math;
using namespace SL::Input_Lite;


CYdLidar laser;

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
inline int GetScreenSize(int *x, int *y) {
  *x = GetSystemMetrics(SM_CXSCREEN);
  *y = GetSystemMetrics(SM_CYSCREEN);
  return 0;
}
#elif __APPLE__
#include <Carbon/Carbon.h>
#include <Windows.h>

#else
#include <X11/Xlib.h>

inline int GetScreenSize(int *w, int *h) {
  Display *pdsp = NULL;
  Screen *pscr = NULL;
  pdsp = XOpenDisplay(NULL);

  if (!pdsp) {
    fprintf(stderr, "Failed to open default display.\n");
    return -1;
  }

  pscr = DefaultScreenOfDisplay(pdsp);

  if (!pscr) {
    fprintf(stderr, "Failed to obtain the default screen of given display.\n");
    return -2;
  }

  *w = pscr->width;
  *h = pscr->height;
  XCloseDisplay(pdsp);
  return 0;
}
#endif


int main(int argc, char *argv[]) {

  printf(" YDLIDAR BIG SCRREN C++ TEST\n\n");
  char str[40];
  char baudrate[4];
  printf("请输入串口号或雷达IP:");
  std::cin >> str;
  const std::string port = string(str);

  regex reg("(\\d{1,3}).(\\d{1,3}).(\\d{1,3}).(\\d{1,3})");
  smatch m;

  uint8_t driver_type;

  if (regex_match(port, m, reg)) {
    driver_type = 1;
  } else {
    driver_type = 0;
  }

  printf("请输入雷达波特率或网络端口:");
  std::cin >> baudrate;

  const int baud =  atoi(baudrate);


#ifdef WIN32
  // windows ascii codes are the char representations
  assert(SL::Input_Lite::ConvertToKeyCode('0') == SL::Input_Lite::KeyCodes::KEY_0);
  assert(SL::Input_Lite::ConvertToKeyCode('1') == SL::Input_Lite::KeyCodes::KEY_1);
  assert(SL::Input_Lite::ConvertToKeyCode('2') == SL::Input_Lite::KeyCodes::KEY_2);
  assert(SL::Input_Lite::ConvertToKeyCode('3') == SL::Input_Lite::KeyCodes::KEY_3);
  assert(SL::Input_Lite::ConvertToKeyCode('4') == SL::Input_Lite::KeyCodes::KEY_4);
  assert(SL::Input_Lite::ConvertToKeyCode('5') == SL::Input_Lite::KeyCodes::KEY_5);
  assert(SL::Input_Lite::ConvertToKeyCode('6') == SL::Input_Lite::KeyCodes::KEY_6);
  assert(SL::Input_Lite::ConvertToKeyCode('7') == SL::Input_Lite::KeyCodes::KEY_7);
  assert(SL::Input_Lite::ConvertToKeyCode('8') == SL::Input_Lite::KeyCodes::KEY_8);
  assert(SL::Input_Lite::ConvertToKeyCode('9') == SL::Input_Lite::KeyCodes::KEY_9);

  assert(SL::Input_Lite::ConvertToKeyCode('A') == SL::Input_Lite::KeyCodes::KEY_A);
  assert(SL::Input_Lite::ConvertToKeyCode('B') == SL::Input_Lite::KeyCodes::KEY_B);
  assert(SL::Input_Lite::ConvertToKeyCode('C') == SL::Input_Lite::KeyCodes::KEY_C);
  assert(SL::Input_Lite::ConvertToKeyCode('D') == SL::Input_Lite::KeyCodes::KEY_D);
  assert(SL::Input_Lite::ConvertToKeyCode('E') == SL::Input_Lite::KeyCodes::KEY_E);
  assert(SL::Input_Lite::ConvertToKeyCode('F') == SL::Input_Lite::KeyCodes::KEY_F);
  assert(SL::Input_Lite::ConvertToKeyCode('G') == SL::Input_Lite::KeyCodes::KEY_G);
  assert(SL::Input_Lite::ConvertToKeyCode('H') == SL::Input_Lite::KeyCodes::KEY_H);
  assert(SL::Input_Lite::ConvertToKeyCode('I') == SL::Input_Lite::KeyCodes::KEY_I);
  assert(SL::Input_Lite::ConvertToKeyCode('J') == SL::Input_Lite::KeyCodes::KEY_J);
  assert(SL::Input_Lite::ConvertToKeyCode('K') == SL::Input_Lite::KeyCodes::KEY_K);
  assert(SL::Input_Lite::ConvertToKeyCode('L') == SL::Input_Lite::KeyCodes::KEY_L);
  assert(SL::Input_Lite::ConvertToKeyCode('M') == SL::Input_Lite::KeyCodes::KEY_M);
  assert(SL::Input_Lite::ConvertToKeyCode('N') == SL::Input_Lite::KeyCodes::KEY_N);
  assert(SL::Input_Lite::ConvertToKeyCode('O') == SL::Input_Lite::KeyCodes::KEY_O);
  assert(SL::Input_Lite::ConvertToKeyCode('P') == SL::Input_Lite::KeyCodes::KEY_P);
  assert(SL::Input_Lite::ConvertToKeyCode('Q') == SL::Input_Lite::KeyCodes::KEY_Q);
  assert(SL::Input_Lite::ConvertToKeyCode('R') == SL::Input_Lite::KeyCodes::KEY_R);
  assert(SL::Input_Lite::ConvertToKeyCode('S') == SL::Input_Lite::KeyCodes::KEY_S);
  assert(SL::Input_Lite::ConvertToKeyCode('T') == SL::Input_Lite::KeyCodes::KEY_T);
  assert(SL::Input_Lite::ConvertToKeyCode('U') == SL::Input_Lite::KeyCodes::KEY_U);
  assert(SL::Input_Lite::ConvertToKeyCode('V') == SL::Input_Lite::KeyCodes::KEY_V);
  assert(SL::Input_Lite::ConvertToKeyCode('W') == SL::Input_Lite::KeyCodes::KEY_W);
  assert(SL::Input_Lite::ConvertToKeyCode('X') == SL::Input_Lite::KeyCodes::KEY_X);
  assert(SL::Input_Lite::ConvertToKeyCode('Y') == SL::Input_Lite::KeyCodes::KEY_Y);
  assert(SL::Input_Lite::ConvertToKeyCode('Z') == SL::Input_Lite::KeyCodes::KEY_Z);

  assert(SL::Input_Lite::ConvertToKeyCode(VK_RETURN) == SL::Input_Lite::KeyCodes::KEY_Enter);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_ESCAPE) == SL::Input_Lite::KeyCodes::KEY_Escape);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_BACK) == SL::Input_Lite::KeyCodes::KEY_Backspace);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_TAB) == SL::Input_Lite::KeyCodes::KEY_Tab);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_SPACE) == SL::Input_Lite::KeyCodes::KEY_Space);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_OEM_MINUS) == SL::Input_Lite::KeyCodes::KEY_Minus);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_OEM_PLUS) ==
         SL::Input_Lite::KeyCodes::KEY_Equals); // this is correct and not a mistype

  assert(SL::Input_Lite::ConvertToKeyCode(VK_OEM_4) == SL::Input_Lite::KeyCodes::KEY_LeftBracket);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_OEM_6) == SL::Input_Lite::KeyCodes::KEY_RightBracket);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_OEM_5) == SL::Input_Lite::KeyCodes::KEY_Backslash);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_OEM_1) == SL::Input_Lite::KeyCodes::KEY_Semicolon);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_OEM_7) == SL::Input_Lite::KeyCodes::KEY_Quote);

  assert(SL::Input_Lite::ConvertToKeyCode(VK_OEM_3) == SL::Input_Lite::KeyCodes::KEY_Grave);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_OEM_COMMA) == SL::Input_Lite::KeyCodes::KEY_Comma);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_OEM_PERIOD) == SL::Input_Lite::KeyCodes::KEY_Period);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_OEM_2) == SL::Input_Lite::KeyCodes::KEY_Slash);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_CAPITAL) == SL::Input_Lite::KeyCodes::KEY_CapsLock);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F1) == SL::Input_Lite::KeyCodes::KEY_F1);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F2) == SL::Input_Lite::KeyCodes::KEY_F2);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F3) == SL::Input_Lite::KeyCodes::KEY_F3);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F4) == SL::Input_Lite::KeyCodes::KEY_F4);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F5) == SL::Input_Lite::KeyCodes::KEY_F5);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F6) == SL::Input_Lite::KeyCodes::KEY_F6);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F7) == SL::Input_Lite::KeyCodes::KEY_F7);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F8) == SL::Input_Lite::KeyCodes::KEY_F8);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F9) == SL::Input_Lite::KeyCodes::KEY_F9);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F10) == SL::Input_Lite::KeyCodes::KEY_F10);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F11) == SL::Input_Lite::KeyCodes::KEY_F11);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F12) == SL::Input_Lite::KeyCodes::KEY_F12);

  assert(SL::Input_Lite::ConvertToKeyCode(VK_SNAPSHOT) == SL::Input_Lite::KeyCodes::KEY_PrintScreen);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_SCROLL) == SL::Input_Lite::KeyCodes::KEY_ScrollLock);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_PAUSE) == SL::Input_Lite::KeyCodes::KEY_Pause);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_INSERT) == SL::Input_Lite::KeyCodes::KEY_Insert);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_HOME) == SL::Input_Lite::KeyCodes::KEY_Home);

  assert(SL::Input_Lite::ConvertToKeyCode(VK_PRIOR) == SL::Input_Lite::KeyCodes::KEY_PageUp);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_DELETE) == SL::Input_Lite::KeyCodes::KEY_Delete);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_END) == SL::Input_Lite::KeyCodes::KEY_End);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_NEXT) == SL::Input_Lite::KeyCodes::KEY_PageDown);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_RIGHT) == SL::Input_Lite::KeyCodes::KEY_Right);

  assert(SL::Input_Lite::ConvertToKeyCode(VK_LEFT) == SL::Input_Lite::KeyCodes::KEY_Left);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_DOWN) == SL::Input_Lite::KeyCodes::KEY_Down);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_UP) == SL::Input_Lite::KeyCodes::KEY_Up);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_NUMLOCK) == SL::Input_Lite::KeyCodes::KP_NumLock);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_DIVIDE) == SL::Input_Lite::KeyCodes::KP_Divide);

  assert(SL::Input_Lite::ConvertToKeyCode(VK_MULTIPLY) == SL::Input_Lite::KeyCodes::KP_Multiply);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_SUBTRACT) == SL::Input_Lite::KeyCodes::KP_Subtract);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_ADD) == SL::Input_Lite::KeyCodes::KP_Add);
  // assert(SL::Input_Lite::ConvertToKeyCode(VK_RETURN) == SL::Input_Lite::KeyCodes::KP_Enter);

  assert(SL::Input_Lite::ConvertToKeyCode(VK_NUMPAD1) == SL::Input_Lite::KeyCodes::KP_1);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_NUMPAD2) == SL::Input_Lite::KeyCodes::KP_2);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_NUMPAD3) == SL::Input_Lite::KeyCodes::KP_3);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_NUMPAD4) == SL::Input_Lite::KeyCodes::KP_4);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_NUMPAD5) == SL::Input_Lite::KeyCodes::KP_5);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_NUMPAD6) == SL::Input_Lite::KeyCodes::KP_6);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_NUMPAD7) == SL::Input_Lite::KeyCodes::KP_7);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_NUMPAD8) == SL::Input_Lite::KeyCodes::KP_8);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_NUMPAD9) == SL::Input_Lite::KeyCodes::KP_9);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_NUMPAD0) == SL::Input_Lite::KeyCodes::KP_0);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_DECIMAL) == SL::Input_Lite::KeyCodes::KP_Point);

  // assert(SL::Input_Lite::ConvertToKeyCode(255) == SL::Input_Lite::KeyCodes::KEY_NonUSBackslash);
  // assert(SL::Input_Lite::ConvertToKeyCode(255) == SL::Input_Lite::KeyCodes::KP_Equals);

  assert(SL::Input_Lite::ConvertToKeyCode(VK_F13) == SL::Input_Lite::KeyCodes::KEY_F13);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F14) == SL::Input_Lite::KeyCodes::KEY_F14);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F15) == SL::Input_Lite::KeyCodes::KEY_F15);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F16) == SL::Input_Lite::KeyCodes::KEY_F16);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F17) == SL::Input_Lite::KeyCodes::KEY_F17);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F18) == SL::Input_Lite::KeyCodes::KEY_F18);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F19) == SL::Input_Lite::KeyCodes::KEY_F19);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F20) == SL::Input_Lite::KeyCodes::KEY_F20);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F21) == SL::Input_Lite::KeyCodes::KEY_F21);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F22) == SL::Input_Lite::KeyCodes::KEY_F22);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F23) == SL::Input_Lite::KeyCodes::KEY_F23);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_F24) == SL::Input_Lite::KeyCodes::KEY_F24);

  assert(SL::Input_Lite::ConvertToKeyCode(VK_HELP) == SL::Input_Lite::KeyCodes::KEY_Help);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_MENU) == SL::Input_Lite::KeyCodes::KEY_Menu);

  assert(SL::Input_Lite::ConvertToKeyCode(VK_CONTROL) == SL::Input_Lite::KeyCodes::KEY_LeftControl);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_SHIFT) == SL::Input_Lite::KeyCodes::KEY_LeftShift);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_MENU) == SL::Input_Lite::KeyCodes::KEY_Menu);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_LWIN) == SL::Input_Lite::KeyCodes::KEY_LeftMeta);

  // assert(SL::Input_Lite::ConvertToKeyCode(VK_CONTROL) == SL::Input_Lite::KeyCodes::KEY_RightControl);
  // assert(SL::Input_Lite::ConvertToKeyCode(VK_SHIFT) == SL::Input_Lite::KeyCodes::KEY_RightShift);
  // assert(SL::Input_Lite::ConvertToKeyCode(VK_MENU) == SL::Input_Lite::KeyCodes::KEY_RightAlt);
  assert(SL::Input_Lite::ConvertToKeyCode(VK_RWIN) == SL::Input_Lite::KeyCodes::KEY_RightMeta);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_0) == '0');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_1) == '1');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_2) == '2');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_3) == '3');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_4) == '4');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_5) == '5');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_6) == '6');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_7) == '7');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_8) == '8');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_9) == '9');

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_A) == 'A');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_B) == 'B');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_C) == 'C');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_D) == 'D');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_E) == 'E');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F) == 'F');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_G) == 'G');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_H) == 'H');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_I) == 'I');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_J) == 'J');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_K) == 'K');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_L) == 'L');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_M) == 'M');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_N) == 'N');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_O) == 'O');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_P) == 'P');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Q) == 'Q');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_R) == 'R');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_S) == 'S');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_T) == 'T');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_U) == 'U');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_V) == 'V');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_W) == 'W');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_X) == 'X');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Y) == 'Y');
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Z) == 'Z');

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Enter) == VK_RETURN);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Escape) == VK_ESCAPE);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Backspace) == VK_BACK);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Tab) == VK_TAB);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Space) == VK_SPACE);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Minus) == VK_OEM_MINUS);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Equals) ==
         VK_OEM_PLUS); // this is correct and not a mistype

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_LeftBracket) == VK_OEM_4);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_RightBracket) == VK_OEM_6);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Backslash) == VK_OEM_5);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Semicolon) == VK_OEM_1);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Quote) == VK_OEM_7);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Grave) == VK_OEM_3);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Comma) == VK_OEM_COMMA);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Period) == VK_OEM_PERIOD);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Slash) == VK_OEM_2);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_CapsLock) == VK_CAPITAL);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F1) == VK_F1);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F2) == VK_F2);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F3) == VK_F3);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F4) == VK_F4);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F5) == VK_F5);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F6) == VK_F6);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F7) == VK_F7);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F8) == VK_F8);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F9) == VK_F9);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F10) == VK_F10);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F11) == VK_F11);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F12) == VK_F12);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_PrintScreen) == VK_SNAPSHOT);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_ScrollLock) == VK_SCROLL);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Pause) == VK_PAUSE);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Insert) == VK_INSERT);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Home) == VK_HOME);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_PageUp) == VK_PRIOR);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Delete) == VK_DELETE);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_End) == VK_END);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_PageDown) == VK_NEXT);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Right) == VK_RIGHT);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Left) == VK_LEFT);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Down) == VK_DOWN);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Up) == VK_UP);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_NumLock) == VK_NUMLOCK);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Divide) == VK_DIVIDE);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Multiply) == VK_MULTIPLY);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Subtract) == VK_SUBTRACT);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Add) == VK_ADD);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Enter) == VK_RETURN);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_1) == VK_NUMPAD1);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_2) == VK_NUMPAD2);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_3) == VK_NUMPAD3);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_4) == VK_NUMPAD4);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_5) == VK_NUMPAD5);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_6) == VK_NUMPAD6);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_7) == VK_NUMPAD7);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_8) == VK_NUMPAD8);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_9) == VK_NUMPAD9);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_0) == VK_NUMPAD0);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Point) == VK_DECIMAL);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_NonUSBackslash) == 255);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Equals) == 255);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F13) == VK_F13);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F14) == VK_F14);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F15) == VK_F15);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F16) == VK_F16);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F17) == VK_F17);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F18) == VK_F18);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F19) == VK_F19);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F20) == VK_F20);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F21) == VK_F21);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F22) == VK_F22);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F23) == VK_F23);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F24) == VK_F24);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Help) == VK_HELP);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Menu) == VK_MENU);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_LeftControl) == VK_CONTROL);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_LeftShift) == VK_SHIFT);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_LeftAlt) == VK_MENU);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_LeftMeta) == VK_LWIN);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_RightControl) == VK_CONTROL);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_RightShift) == VK_SHIFT);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_RightAlt) == VK_MENU);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_RightMeta) == VK_RWIN);

#elif __APPLE__

  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_0) == SL::Input_Lite::KeyCodes::KEY_0);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_1) == SL::Input_Lite::KeyCodes::KEY_1);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_2) == SL::Input_Lite::KeyCodes::KEY_2);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_3) == SL::Input_Lite::KeyCodes::KEY_3);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_4) == SL::Input_Lite::KeyCodes::KEY_4);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_5) == SL::Input_Lite::KeyCodes::KEY_5);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_6) == SL::Input_Lite::KeyCodes::KEY_6);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_7) == SL::Input_Lite::KeyCodes::KEY_7);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_8) == SL::Input_Lite::KeyCodes::KEY_8);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_9) == SL::Input_Lite::KeyCodes::KEY_9);

  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_A) == SL::Input_Lite::KeyCodes::KEY_A);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_B) == SL::Input_Lite::KeyCodes::KEY_B);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_C) == SL::Input_Lite::KeyCodes::KEY_C);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_D) == SL::Input_Lite::KeyCodes::KEY_D);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_E) == SL::Input_Lite::KeyCodes::KEY_E);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_F) == SL::Input_Lite::KeyCodes::KEY_F);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_G) == SL::Input_Lite::KeyCodes::KEY_G);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_H) == SL::Input_Lite::KeyCodes::KEY_H);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_I) == SL::Input_Lite::KeyCodes::KEY_I);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_J) == SL::Input_Lite::KeyCodes::KEY_J);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_K) == SL::Input_Lite::KeyCodes::KEY_K);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_L) == SL::Input_Lite::KeyCodes::KEY_L);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_M) == SL::Input_Lite::KeyCodes::KEY_M);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_N) == SL::Input_Lite::KeyCodes::KEY_N);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_O) == SL::Input_Lite::KeyCodes::KEY_O);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_P) == SL::Input_Lite::KeyCodes::KEY_P);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Q) == SL::Input_Lite::KeyCodes::KEY_Q);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_R) == SL::Input_Lite::KeyCodes::KEY_R);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_S) == SL::Input_Lite::KeyCodes::KEY_S);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_T) == SL::Input_Lite::KeyCodes::KEY_T);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_U) == SL::Input_Lite::KeyCodes::KEY_U);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_V) == SL::Input_Lite::KeyCodes::KEY_V);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_W) == SL::Input_Lite::KeyCodes::KEY_W);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_X) == SL::Input_Lite::KeyCodes::KEY_X);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Y) == SL::Input_Lite::KeyCodes::KEY_Y);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Z) == SL::Input_Lite::KeyCodes::KEY_Z);

  assert(SL::Input_Lite::ConvertToKeyCode(kVK_Return) == SL::Input_Lite::KeyCodes::KEY_Enter);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_Escape) == SL::Input_Lite::KeyCodes::KEY_Escape);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_Delete) ==
         SL::Input_Lite::KeyCodes::KEY_Backspace); // not a type
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_Tab) == SL::Input_Lite::KeyCodes::KEY_Tab);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_Space) == SL::Input_Lite::KeyCodes::KEY_Space);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Minus) == SL::Input_Lite::KeyCodes::KEY_Minus);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Equal) ==
         SL::Input_Lite::KeyCodes::KEY_Equals); // this is correct and not a mistype

  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_LeftBracket) ==
         SL::Input_Lite::KeyCodes::KEY_LeftBracket);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_RightBracket) ==
         SL::Input_Lite::KeyCodes::KEY_RightBracket);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Backslash) ==
         SL::Input_Lite::KeyCodes::KEY_Backslash);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Semicolon) ==
         SL::Input_Lite::KeyCodes::KEY_Semicolon);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Quote) == SL::Input_Lite::KeyCodes::KEY_Quote);

  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Grave) == SL::Input_Lite::KeyCodes::KEY_Grave);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Comma) == SL::Input_Lite::KeyCodes::KEY_Comma);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Period) == SL::Input_Lite::KeyCodes::KEY_Period);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Slash) == SL::Input_Lite::KeyCodes::KEY_Slash);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_CapsLock) == SL::Input_Lite::KeyCodes::KEY_CapsLock);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F1) == SL::Input_Lite::KeyCodes::KEY_F1);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F2) == SL::Input_Lite::KeyCodes::KEY_F2);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F3) == SL::Input_Lite::KeyCodes::KEY_F3);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F4) == SL::Input_Lite::KeyCodes::KEY_F4);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F5) == SL::Input_Lite::KeyCodes::KEY_F5);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F6) == SL::Input_Lite::KeyCodes::KEY_F6);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F7) == SL::Input_Lite::KeyCodes::KEY_F7);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F8) == SL::Input_Lite::KeyCodes::KEY_F8);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F9) == SL::Input_Lite::KeyCodes::KEY_F9);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F10) == SL::Input_Lite::KeyCodes::KEY_F10);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F11) == SL::Input_Lite::KeyCodes::KEY_F11);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F12) == SL::Input_Lite::KeyCodes::KEY_F12);

  // assert(SL::Input_Lite::ConvertToKeyCode(VK_SNAPSHOT) == SL::Input_Lite::KeyCodes::KEY_PrintScreen);
  // assert(SL::Input_Lite::ConvertToKeyCode(VK_SCROLL) == SL::Input_Lite::KeyCodes::KEY_ScrollLock);
  // assert(SL::Input_Lite::ConvertToKeyCode(VK_PAUSE) == SL::Input_Lite::KeyCodes::KEY_Pause);
  // assert(SL::Input_Lite::ConvertToKeyCode(VK_INSERT) == SL::Input_Lite::KeyCodes::KEY_Insert);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_Home) == SL::Input_Lite::KeyCodes::KEY_Home);

  assert(SL::Input_Lite::ConvertToKeyCode(kVK_PageUp) == SL::Input_Lite::KeyCodes::KEY_PageUp);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ForwardDelete) == SL::Input_Lite::KeyCodes::KEY_Delete);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_End) == SL::Input_Lite::KeyCodes::KEY_End);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_PageDown) == SL::Input_Lite::KeyCodes::KEY_PageDown);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_RightArrow) == SL::Input_Lite::KeyCodes::KEY_Right);

  assert(SL::Input_Lite::ConvertToKeyCode(kVK_LeftArrow) == SL::Input_Lite::KeyCodes::KEY_Left);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_DownArrow) == SL::Input_Lite::KeyCodes::KEY_Down);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_UpArrow) == SL::Input_Lite::KeyCodes::KEY_Up);
  // assert(SL::Input_Lite::ConvertToKeyCode(VK_NUMLOCK) == SL::Input_Lite::KeyCodes::KP_NumLock);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_KeypadDivide) ==
         SL::Input_Lite::KeyCodes::KP_Divide);

  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_KeypadMultiply) ==
         SL::Input_Lite::KeyCodes::KP_Multiply);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_KeypadMinus) ==
         SL::Input_Lite::KeyCodes::KP_Subtract);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_KeypadPlus) == SL::Input_Lite::KeyCodes::KP_Add);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_KeypadEnter) ==
         SL::Input_Lite::KeyCodes::KP_Enter);

  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Keypad0) == SL::Input_Lite::KeyCodes::KP_0);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Keypad1) == SL::Input_Lite::KeyCodes::KP_1);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Keypad2) == SL::Input_Lite::KeyCodes::KP_2);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Keypad3) == SL::Input_Lite::KeyCodes::KP_3);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Keypad4) == SL::Input_Lite::KeyCodes::KP_4);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Keypad5) == SL::Input_Lite::KeyCodes::KP_5);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Keypad6) == SL::Input_Lite::KeyCodes::KP_6);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Keypad7) == SL::Input_Lite::KeyCodes::KP_7);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Keypad8) == SL::Input_Lite::KeyCodes::KP_8);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_Keypad9) == SL::Input_Lite::KeyCodes::KP_9);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_KeypadDecimal) ==
         SL::Input_Lite::KeyCodes::KP_Point);

  // assert(SL::Input_Lite::ConvertToKeyCode(255) == SL::Input_Lite::KeyCodes::KEY_NonUSBackslash);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_ANSI_KeypadEquals) ==
         SL::Input_Lite::KeyCodes::KP_Equals);

  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F13) == SL::Input_Lite::KeyCodes::KEY_F13);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F14) == SL::Input_Lite::KeyCodes::KEY_F14);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F15) == SL::Input_Lite::KeyCodes::KEY_F15);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F16) == SL::Input_Lite::KeyCodes::KEY_F16);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F17) == SL::Input_Lite::KeyCodes::KEY_F17);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F18) == SL::Input_Lite::KeyCodes::KEY_F18);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F19) == SL::Input_Lite::KeyCodes::KEY_F19);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_F20) == SL::Input_Lite::KeyCodes::KEY_F20);
  // assert(SL::Input_Lite::ConvertToKeyCode(VK_F21) == SL::Input_Lite::KeyCodes::KEY_F21);
  // assert(SL::Input_Lite::ConvertToKeyCode(VK_F22) == SL::Input_Lite::KeyCodes::KEY_F22);
  // assert(SL::Input_Lite::ConvertToKeyCode(VK_F23) == SL::Input_Lite::KeyCodes::KEY_F23);
  // assert(SL::Input_Lite::ConvertToKeyCode(VK_F24) == SL::Input_Lite::KeyCodes::KEY_F24);

  assert(SL::Input_Lite::ConvertToKeyCode(kVK_Help) == SL::Input_Lite::KeyCodes::KEY_Help);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_Option) == SL::Input_Lite::KeyCodes::KEY_Menu);

  assert(SL::Input_Lite::ConvertToKeyCode(kVK_Control) == SL::Input_Lite::KeyCodes::KEY_LeftControl);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_Shift) == SL::Input_Lite::KeyCodes::KEY_LeftShift);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_Option) == SL::Input_Lite::KeyCodes::KEY_LeftMeta);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_Command) == SL::Input_Lite::KeyCodes::KEY_LeftAlt);

  assert(SL::Input_Lite::ConvertToKeyCode(kVK_RightControl) ==
         SL::Input_Lite::KeyCodes::KEY_RightControl);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_RightShift) ==
         SL::Input_Lite::KeyCodes::KEY_RightShift);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_RightOption) ==
         SL::Input_Lite::KeyCodes::KEY_RightMeta);
  assert(SL::Input_Lite::ConvertToKeyCode(kVK_RightControl) ==
         SL::Input_Lite::KeyCodes::KEY_RightMeta);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_0) == kVK_ANSI_0);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_1) == kVK_ANSI_1);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_2) == kVK_ANSI_2);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_3) == kVK_ANSI_3);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_4) == kVK_ANSI_4);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_5) == kVK_ANSI_5);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_6) == kVK_ANSI_6);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_7) == kVK_ANSI_7);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_8) == kVK_ANSI_8);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_9) == kVK_ANSI_9);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_A) == kVK_ANSI_A);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_B) == kVK_ANSI_B);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_C) == kVK_ANSI_C);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_D) == kVK_ANSI_D);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_E) == kVK_ANSI_E);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F) == kVK_ANSI_F);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_G) == kVK_ANSI_G);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_H) == kVK_ANSI_H);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_I) == kVK_ANSI_I);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_J) == kVK_ANSI_J);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_K) == kVK_ANSI_K);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_L) == kVK_ANSI_L);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_M) == kVK_ANSI_M);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_N) == kVK_ANSI_N);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_O) == kVK_ANSI_O);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_P) == kVK_ANSI_P);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Q) == kVK_ANSI_Q);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_R) == kVK_ANSI_R);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_S) == kVK_ANSI_S);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_T) == kVK_ANSI_T);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_U) == kVK_ANSI_U);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_V) == kVK_ANSI_V);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_W) == kVK_ANSI_W);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_X) == kVK_ANSI_X);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Y) == kVK_ANSI_Y);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Z) == kVK_ANSI_Z);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Enter) == kVK_Return);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Escape) == kVK_Escape);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Backspace) == kVK_Delete);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Tab) == kVK_Tab);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Space) == kVK_Space);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Minus) == kVK_ANSI_Minus);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Equals) == kVK_ANSI_Equal);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_LeftBracket) ==
         kVK_ANSI_LeftBracket);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_RightBracket) ==
         kVK_ANSI_RightBracket);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Backslash) ==
         kVK_ANSI_Backslash);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Semicolon) ==
         kVK_ANSI_Semicolon);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Quote) == kVK_ANSI_Quote);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Grave) == kVK_ANSI_Grave);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Comma) == kVK_ANSI_Comma);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Period) == kVK_ANSI_Period);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Slash) == kVK_ANSI_Slash);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_CapsLock) == kVK_CapsLock);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F1) == kVK_F1);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F2) == kVK_F2);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F3) == kVK_F3);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F4) == kVK_F4);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F5) == kVK_F5);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F6) == kVK_F6);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F7) == kVK_F7);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F8) == kVK_F8);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F9) == kVK_F9);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F10) == kVK_F10);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F11) == kVK_F11);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F12) == kVK_F12);

  /*   assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_PrintScreen) == VK_SNAPSHOT);
     assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_ScrollLock) == VK_SCROLL);
     assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Pause) == VK_PAUSE);
     assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Insert) == VK_INSERT);*/
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Home) == kVK_Home);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_PageUp) == kVK_PageUp);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Delete) == kVK_ForwardDelete);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_End) == kVK_End);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_PageDown) == kVK_PageDown);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Right) == kVK_RightArrow);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Left) == kVK_LeftArrow);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Down) == kVK_PageDown);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Up) == kVK_UpArrow);
  // assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_NumLock) == VK_NUMLOCK);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Divide) ==
         kVK_ANSI_KeypadDivide);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Multiply) ==
         kVK_ANSI_KeypadMultiply);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Subtract) ==
         kVK_ANSI_KeypadMinus);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Add) == kVK_ANSI_KeypadPlus);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Enter) == kVK_ANSI_KeypadEnter);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_1) == kVK_ANSI_Keypad1);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_2) == kVK_ANSI_Keypad2);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_3) == kVK_ANSI_Keypad3);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_4) == kVK_ANSI_Keypad4);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_5) == kVK_ANSI_Keypad5);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_6) == kVK_ANSI_Keypad6);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_7) == kVK_ANSI_Keypad7);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_8) == kVK_ANSI_Keypad8);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_9) == kVK_ANSI_Keypad9);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_0) == kVK_ANSI_Keypad0);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Point) ==
         kVK_ANSI_KeypadDecimal);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_NonUSBackslash) == 255);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Equals) ==
         kVK_ANSI_KeypadEquals);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F13) == kVK_F13);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F14) == kVK_F14);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F15) == kVK_F15);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F16) == kVK_F16);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F17) == kVK_F17);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F18) == kVK_F18);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F19) == kVK_F19);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F20) == kVK_F20);
  /*
   assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F21) == VK_F21);
   assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F22) == VK_F22);
   assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F23) == VK_F23);
   assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F24) == VK_F24);
   */

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Help) == kVK_Help);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Menu) == kVK_Option);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_LeftControl) == kVK_Control);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_LeftShift) == kVK_Shift);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_LeftAlt) == kVK_Option);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_LeftMeta) == kVK_Command);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_RightControl) ==
         kVK_RightControl);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_RightShift) == kVK_RightShift);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_RightAlt) == kVK_RightOption);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_RightMeta) ==
         kVK_RightControl);

#elif __linux__

  assert(SL::Input_Lite::ConvertToKeyCode(19) == SL::Input_Lite::KeyCodes::KEY_0);
  assert(SL::Input_Lite::ConvertToKeyCode(10) == SL::Input_Lite::KeyCodes::KEY_1);
  assert(SL::Input_Lite::ConvertToKeyCode(11) == SL::Input_Lite::KeyCodes::KEY_2);
  assert(SL::Input_Lite::ConvertToKeyCode(12) == SL::Input_Lite::KeyCodes::KEY_3);
  assert(SL::Input_Lite::ConvertToKeyCode(13) == SL::Input_Lite::KeyCodes::KEY_4);
  assert(SL::Input_Lite::ConvertToKeyCode(14) == SL::Input_Lite::KeyCodes::KEY_5);
  assert(SL::Input_Lite::ConvertToKeyCode(15) == SL::Input_Lite::KeyCodes::KEY_6);
  assert(SL::Input_Lite::ConvertToKeyCode(16) == SL::Input_Lite::KeyCodes::KEY_7);
  assert(SL::Input_Lite::ConvertToKeyCode(17) == SL::Input_Lite::KeyCodes::KEY_8);
  assert(SL::Input_Lite::ConvertToKeyCode(18) == SL::Input_Lite::KeyCodes::KEY_9);

  assert(SL::Input_Lite::ConvertToKeyCode(38) == SL::Input_Lite::KeyCodes::KEY_A);
  assert(SL::Input_Lite::ConvertToKeyCode(56) == SL::Input_Lite::KeyCodes::KEY_B);
  assert(SL::Input_Lite::ConvertToKeyCode(54) == SL::Input_Lite::KeyCodes::KEY_C);
  assert(SL::Input_Lite::ConvertToKeyCode(40) == SL::Input_Lite::KeyCodes::KEY_D);
  assert(SL::Input_Lite::ConvertToKeyCode(26) == SL::Input_Lite::KeyCodes::KEY_E);
  assert(SL::Input_Lite::ConvertToKeyCode(41) == SL::Input_Lite::KeyCodes::KEY_F);
  assert(SL::Input_Lite::ConvertToKeyCode(42) == SL::Input_Lite::KeyCodes::KEY_G);
  assert(SL::Input_Lite::ConvertToKeyCode(43) == SL::Input_Lite::KeyCodes::KEY_H);
  assert(SL::Input_Lite::ConvertToKeyCode(31) == SL::Input_Lite::KeyCodes::KEY_I);
  assert(SL::Input_Lite::ConvertToKeyCode(44) == SL::Input_Lite::KeyCodes::KEY_J);
  assert(SL::Input_Lite::ConvertToKeyCode(45) == SL::Input_Lite::KeyCodes::KEY_K);
  assert(SL::Input_Lite::ConvertToKeyCode(46) == SL::Input_Lite::KeyCodes::KEY_L);
  assert(SL::Input_Lite::ConvertToKeyCode(58) == SL::Input_Lite::KeyCodes::KEY_M);
  assert(SL::Input_Lite::ConvertToKeyCode(57) == SL::Input_Lite::KeyCodes::KEY_N);
  assert(SL::Input_Lite::ConvertToKeyCode(32) == SL::Input_Lite::KeyCodes::KEY_O);
  assert(SL::Input_Lite::ConvertToKeyCode(33) == SL::Input_Lite::KeyCodes::KEY_P);
  assert(SL::Input_Lite::ConvertToKeyCode(24) == SL::Input_Lite::KeyCodes::KEY_Q);
  assert(SL::Input_Lite::ConvertToKeyCode(27) == SL::Input_Lite::KeyCodes::KEY_R);
  assert(SL::Input_Lite::ConvertToKeyCode(39) == SL::Input_Lite::KeyCodes::KEY_S);
  assert(SL::Input_Lite::ConvertToKeyCode(28) == SL::Input_Lite::KeyCodes::KEY_T);
  assert(SL::Input_Lite::ConvertToKeyCode(30) == SL::Input_Lite::KeyCodes::KEY_U);
  assert(SL::Input_Lite::ConvertToKeyCode(55) == SL::Input_Lite::KeyCodes::KEY_V);
  assert(SL::Input_Lite::ConvertToKeyCode(25) == SL::Input_Lite::KeyCodes::KEY_W);
  assert(SL::Input_Lite::ConvertToKeyCode(53) == SL::Input_Lite::KeyCodes::KEY_X);
  assert(SL::Input_Lite::ConvertToKeyCode(29) == SL::Input_Lite::KeyCodes::KEY_Y);
  assert(SL::Input_Lite::ConvertToKeyCode(52) == SL::Input_Lite::KeyCodes::KEY_Z);

  assert(SL::Input_Lite::ConvertToKeyCode(36) == SL::Input_Lite::KeyCodes::KEY_Enter);
  assert(SL::Input_Lite::ConvertToKeyCode(9) == SL::Input_Lite::KeyCodes::KEY_Escape);
  assert(SL::Input_Lite::ConvertToKeyCode(22) == SL::Input_Lite::KeyCodes::KEY_Backspace);
  assert(SL::Input_Lite::ConvertToKeyCode(23) == SL::Input_Lite::KeyCodes::KEY_Tab);
  assert(SL::Input_Lite::ConvertToKeyCode(65) == SL::Input_Lite::KeyCodes::KEY_Space);
  assert(SL::Input_Lite::ConvertToKeyCode(20) == SL::Input_Lite::KeyCodes::KEY_Minus);
  assert(SL::Input_Lite::ConvertToKeyCode(21) ==
         SL::Input_Lite::KeyCodes::KEY_Equals); // this is correct and not a mistype

  assert(SL::Input_Lite::ConvertToKeyCode(34) == SL::Input_Lite::KeyCodes::KEY_LeftBracket);
  assert(SL::Input_Lite::ConvertToKeyCode(35) == SL::Input_Lite::KeyCodes::KEY_RightBracket);
  assert(SL::Input_Lite::ConvertToKeyCode(51) == SL::Input_Lite::KeyCodes::KEY_Backslash);
  assert(SL::Input_Lite::ConvertToKeyCode(47) == SL::Input_Lite::KeyCodes::KEY_Semicolon);
  assert(SL::Input_Lite::ConvertToKeyCode(48) == SL::Input_Lite::KeyCodes::KEY_Quote);

  assert(SL::Input_Lite::ConvertToKeyCode(49) == SL::Input_Lite::KeyCodes::KEY_Grave);
  assert(SL::Input_Lite::ConvertToKeyCode(59) == SL::Input_Lite::KeyCodes::KEY_Comma);
  assert(SL::Input_Lite::ConvertToKeyCode(60) == SL::Input_Lite::KeyCodes::KEY_Period);
  assert(SL::Input_Lite::ConvertToKeyCode(61) == SL::Input_Lite::KeyCodes::KEY_Slash);
  assert(SL::Input_Lite::ConvertToKeyCode(66) == SL::Input_Lite::KeyCodes::KEY_CapsLock);
  assert(SL::Input_Lite::ConvertToKeyCode(67) == SL::Input_Lite::KeyCodes::KEY_F1);
  assert(SL::Input_Lite::ConvertToKeyCode(68) == SL::Input_Lite::KeyCodes::KEY_F2);
  assert(SL::Input_Lite::ConvertToKeyCode(69) == SL::Input_Lite::KeyCodes::KEY_F3);
  assert(SL::Input_Lite::ConvertToKeyCode(70) == SL::Input_Lite::KeyCodes::KEY_F4);
  assert(SL::Input_Lite::ConvertToKeyCode(71) == SL::Input_Lite::KeyCodes::KEY_F5);
  assert(SL::Input_Lite::ConvertToKeyCode(72) == SL::Input_Lite::KeyCodes::KEY_F6);
  assert(SL::Input_Lite::ConvertToKeyCode(73) == SL::Input_Lite::KeyCodes::KEY_F7);
  assert(SL::Input_Lite::ConvertToKeyCode(74) == SL::Input_Lite::KeyCodes::KEY_F8);
  assert(SL::Input_Lite::ConvertToKeyCode(75) == SL::Input_Lite::KeyCodes::KEY_F9);
  assert(SL::Input_Lite::ConvertToKeyCode(76) == SL::Input_Lite::KeyCodes::KEY_F10);
  assert(SL::Input_Lite::ConvertToKeyCode(95) == SL::Input_Lite::KeyCodes::KEY_F11);
  assert(SL::Input_Lite::ConvertToKeyCode(96) == SL::Input_Lite::KeyCodes::KEY_F12);

  assert(SL::Input_Lite::ConvertToKeyCode(107) == SL::Input_Lite::KeyCodes::KEY_PrintScreen);
  assert(SL::Input_Lite::ConvertToKeyCode(78) == SL::Input_Lite::KeyCodes::KEY_ScrollLock);
  assert(SL::Input_Lite::ConvertToKeyCode(127) == SL::Input_Lite::KeyCodes::KEY_Pause);
  assert(SL::Input_Lite::ConvertToKeyCode(118) == SL::Input_Lite::KeyCodes::KEY_Insert);
  assert(SL::Input_Lite::ConvertToKeyCode(110) == SL::Input_Lite::KeyCodes::KEY_Home);
  assert(SL::Input_Lite::ConvertToKeyCode(125) == SL::Input_Lite::KeyCodes::KP_Equals);

  assert(SL::Input_Lite::ConvertToKeyCode(112) == SL::Input_Lite::KeyCodes::KEY_PageUp);
  assert(SL::Input_Lite::ConvertToKeyCode(119) == SL::Input_Lite::KeyCodes::KEY_Delete);
  assert(SL::Input_Lite::ConvertToKeyCode(115) == SL::Input_Lite::KeyCodes::KEY_End);
  assert(SL::Input_Lite::ConvertToKeyCode(117) == SL::Input_Lite::KeyCodes::KEY_PageDown);
  assert(SL::Input_Lite::ConvertToKeyCode(114) == SL::Input_Lite::KeyCodes::KEY_Right);

  assert(SL::Input_Lite::ConvertToKeyCode(113) == SL::Input_Lite::KeyCodes::KEY_Left);
  assert(SL::Input_Lite::ConvertToKeyCode(116) == SL::Input_Lite::KeyCodes::KEY_Down);
  assert(SL::Input_Lite::ConvertToKeyCode(111) == SL::Input_Lite::KeyCodes::KEY_Up);
  assert(SL::Input_Lite::ConvertToKeyCode(77) == SL::Input_Lite::KeyCodes::KP_NumLock);
  assert(SL::Input_Lite::ConvertToKeyCode(106) == SL::Input_Lite::KeyCodes::KP_Divide);

  assert(SL::Input_Lite::ConvertToKeyCode(63) == SL::Input_Lite::KeyCodes::KP_Multiply);
  assert(SL::Input_Lite::ConvertToKeyCode(82) == SL::Input_Lite::KeyCodes::KP_Subtract);
  assert(SL::Input_Lite::ConvertToKeyCode(86) == SL::Input_Lite::KeyCodes::KP_Add);
  assert(SL::Input_Lite::ConvertToKeyCode(104) == SL::Input_Lite::KeyCodes::KP_Enter);

  assert(SL::Input_Lite::ConvertToKeyCode(87) == SL::Input_Lite::KeyCodes::KP_1);
  assert(SL::Input_Lite::ConvertToKeyCode(88) == SL::Input_Lite::KeyCodes::KP_2);
  assert(SL::Input_Lite::ConvertToKeyCode(89) == SL::Input_Lite::KeyCodes::KP_3);
  assert(SL::Input_Lite::ConvertToKeyCode(83) == SL::Input_Lite::KeyCodes::KP_4);
  assert(SL::Input_Lite::ConvertToKeyCode(84) == SL::Input_Lite::KeyCodes::KP_5);
  assert(SL::Input_Lite::ConvertToKeyCode(85) == SL::Input_Lite::KeyCodes::KP_6);
  assert(SL::Input_Lite::ConvertToKeyCode(79) == SL::Input_Lite::KeyCodes::KP_7);
  assert(SL::Input_Lite::ConvertToKeyCode(80) == SL::Input_Lite::KeyCodes::KP_8);
  assert(SL::Input_Lite::ConvertToKeyCode(81) == SL::Input_Lite::KeyCodes::KP_9);
  assert(SL::Input_Lite::ConvertToKeyCode(90) == SL::Input_Lite::KeyCodes::KP_0);
  assert(SL::Input_Lite::ConvertToKeyCode(91) == SL::Input_Lite::KeyCodes::KP_Point);

  assert(SL::Input_Lite::ConvertToKeyCode(94) == SL::Input_Lite::KeyCodes::KEY_NonUSBackslash);
  // assert(SL::Input_Lite::ConvertToKeyCode(255) == SL::Input_Lite::KeyCodes::KP_Equals);

  assert(SL::Input_Lite::ConvertToKeyCode(191) == SL::Input_Lite::KeyCodes::KEY_F13);
  assert(SL::Input_Lite::ConvertToKeyCode(192) == SL::Input_Lite::KeyCodes::KEY_F14);
  assert(SL::Input_Lite::ConvertToKeyCode(193) == SL::Input_Lite::KeyCodes::KEY_F15);
  assert(SL::Input_Lite::ConvertToKeyCode(194) == SL::Input_Lite::KeyCodes::KEY_F16);
  assert(SL::Input_Lite::ConvertToKeyCode(195) == SL::Input_Lite::KeyCodes::KEY_F17);
  assert(SL::Input_Lite::ConvertToKeyCode(196) == SL::Input_Lite::KeyCodes::KEY_F18);
  assert(SL::Input_Lite::ConvertToKeyCode(197) == SL::Input_Lite::KeyCodes::KEY_F19);
  assert(SL::Input_Lite::ConvertToKeyCode(198) == SL::Input_Lite::KeyCodes::KEY_F20);
  assert(SL::Input_Lite::ConvertToKeyCode(199) == SL::Input_Lite::KeyCodes::KEY_F21);
  assert(SL::Input_Lite::ConvertToKeyCode(200) == SL::Input_Lite::KeyCodes::KEY_F22);
  assert(SL::Input_Lite::ConvertToKeyCode(201) == SL::Input_Lite::KeyCodes::KEY_F23);
  assert(SL::Input_Lite::ConvertToKeyCode(202) == SL::Input_Lite::KeyCodes::KEY_F24);

  assert(SL::Input_Lite::ConvertToKeyCode(146) == SL::Input_Lite::KeyCodes::KEY_Help);
  assert(SL::Input_Lite::ConvertToKeyCode(135) == SL::Input_Lite::KeyCodes::KEY_Menu);

  assert(SL::Input_Lite::ConvertToKeyCode(37) == SL::Input_Lite::KeyCodes::KEY_LeftControl);
  assert(SL::Input_Lite::ConvertToKeyCode(50) == SL::Input_Lite::KeyCodes::KEY_LeftShift);
  assert(SL::Input_Lite::ConvertToKeyCode(64) == SL::Input_Lite::KeyCodes::KEY_LeftAlt);
  assert(SL::Input_Lite::ConvertToKeyCode(133) == SL::Input_Lite::KeyCodes::KEY_LeftMeta);

  assert(SL::Input_Lite::ConvertToKeyCode(105) == SL::Input_Lite::KeyCodes::KEY_RightControl);
  assert(SL::Input_Lite::ConvertToKeyCode(62) == SL::Input_Lite::KeyCodes::KEY_RightShift);
  assert(SL::Input_Lite::ConvertToKeyCode(108) == SL::Input_Lite::KeyCodes::KEY_RightAlt);
  assert(SL::Input_Lite::ConvertToKeyCode(134) == SL::Input_Lite::KeyCodes::KEY_RightMeta);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_0) == 19);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_1) == 10);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_2) == 11);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_3) == 12);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_4) == 13);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_5) == 14);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_6) == 15);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_7) == 16);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_8) == 17);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_9) == 18);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_A) == 38);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_B) == 56);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_C) == 54);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_D) == 40);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_E) == 26);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F) == 41);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_G) == 42);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_H) == 43);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_I) == 31);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_J) == 44);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_K) == 45);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_L) == 46);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_M) == 58);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_N) == 57);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_O) == 32);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_P) == 33);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Q) == 24);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_R) == 27);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_S) == 39);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_T) == 28);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_U) == 30);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_V) == 55);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_W) == 25);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_X) == 53);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Y) == 29);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Z) == 52);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Enter) == 36);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Escape) == 9);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Backspace) == 22);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Tab) == 23);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Space) == 65);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Minus) == 20);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Equals) ==
         21); // this is correct and not a mistype

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_LeftBracket) == 34);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_RightBracket) == 35);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Backslash) == 51);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Semicolon) == 47);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Quote) == 48);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Grave) == 49);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Comma) == 59);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Period) == 60);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Slash) == 61);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_CapsLock) == 66);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F1) == 67);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F2) == 68);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F3) == 69);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F4) == 70);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F5) == 71);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F6) == 72);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F7) == 73);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F8) == 74);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F9) == 75);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F10) == 76);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F11) == 95);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F12) == 96);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_PrintScreen) == 107);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_ScrollLock) == 78);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Pause) == 127);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Insert) == 118);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Home) == 110);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_PageUp) == 112);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Delete) == 119);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_End) == 115);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_PageDown) == 117);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Right) == 114);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Left) == 113);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Down) == 116);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Up) == 111);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_NumLock) == 77);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Divide) == 106);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Multiply) == 63);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Subtract) == 82);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Add) == 86);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Enter) == 104);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_1) == 87);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_2) == 88);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_3) == 89);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_4) == 83);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_5) == 84);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_6) == 85);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_7) == 79);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_8) == 80);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_9) == 81);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_0) == 90);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Point) == 91);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_NonUSBackslash) == 255);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KP_Equals) == 255);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F13) == 191);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F14) == 192);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F15) == 193);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F16) == 194);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F17) == 195);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F18) == 196);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F19) == 197);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F20) == 198);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F21) == 199);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F22) == 200);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F23) == 201);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_F24) == 202);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Help) == 146);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_Menu) == 135);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_LeftControl) == 37);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_LeftShift) == 50);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_LeftAlt) == 64);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_LeftMeta) == 133);

  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_RightControl) == 105);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_RightShift) == 62);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_RightAlt) == 108);
  assert(SL::Input_Lite::ConvertToNative(SL::Input_Lite::KeyCodes::KEY_RightMeta) == 134);

#else
#error "Unknown Operating System!"
#endif


  typedef CArrayDouble<2>  CPointType;

  ydlidar::init(argc, argv);

  int width, height;

  if (GetScreenSize(&width, &height) != 0) {
    width = 1920;
    height = 1080;
  }

  float resolution_x = 1; //电脑屏幕映射到操作屏上的分辨率, X轴上
  float resolution_y = 1; //电脑屏幕映射到操作屏上的分辨率, Y轴上

  double cluster_rad = 10;//mm//区分多触点半径

  double pre_x = 0, pre_y = 0;//上次点击位置
  double pre_pre_x  = 0, pre_pre_y = 0; //上上次点击位置
  bool isempty = false;//是否操作屏幕
  bool select = true;//是否已经左键选择
  int pre_count = 0;
  double points_distance = 10.0;//

  laser.setSerialPort(port);
  laser.setSerialBaudrate(baud);
  laser.setAutoReconnect(true);//串口异常自动重新
  laser.setDeviceType(driver_type);
  laser.setHeartBeat(true);
  laser.setScanFrequency(8);
  laser.setIntensities(false);


  laser.setMax_x(width / resolution_x);
  laser.setMax_y(height / resolution_y);
  laser.setMin_x(0);
  laser.setMin_y(0);
  LaserPose pose;
  pose.x = width / (2 * resolution_x); //G4在9k时盲区是<260mm, 8k<=240mm, 4k<=100mm
  pose.y = -260 ;//height/(2*resolution_y);
  pose.theta = -90;
  pose.reversion = false;//雷达朝向  朝里还是朝外, 朝里是true, 朝外是false
  laser.setpose(pose);







  ////////////////////////////////////////////////////////////////////////////////////////////
  //                     G4雷达零点
  //                      |
  //                      |______

//|-------960----------|////                ------
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

  while (ydlidar::ok()) {
    bool hardError;
    std::vector<touch_info> outPoints;
    int x, y;//当前点击位置

    //获取雷达在屏幕区域中的点
    if (laser.doProcessSimple(outPoints, hardError)) {
      size_t nClusters = 1;
      aligned_containers<CPointType>::vector_t  points;
      std::vector<touch_info>::const_iterator it_first = outPoints.begin();

      for (auto it = outPoints.begin(); it != outPoints.end(); it++) {
        touch_info point = *it;
        CPointType v;
        v[0] = point.screen_x;
        v[1] = point.screen_y;
        points.push_back(v);

        if (pow((*it_first).screen_x - point.screen_x, 2) + pow((*it_first).screen_y - point.screen_y,
            2) > pow(cluster_rad * resolution_x, 2)) {
          nClusters++;
          it_first = it;
        }

      }

      //聚类, 把相邻的多个点聚成单点
      // do k-means
      aligned_containers<CPointType>::vector_t	centers;
      vector<int>				assignments;
      const double cost = kmeanspp(nClusters, points, assignments, &centers);
      bool history = false; //时候连续在同一位置
      bool isdouleclick = false;//是否是双击事件
      bool iswheel = false; //是否是滚轮事件
      bool scale_in = false;

      //聚类后的几个单点  再做处理
      for (auto it = centers.begin(); it != centers.end(); it++) {
        x = (*it)[0] * resolution_x;
        y = (*it)[1] * resolution_y;

        //std::cout<<"center_x: "<<x<<"center_y: "<<y<<std::endl;
        if ((pow(pre_x - x, 2) + pow(pre_y - y, 2)) < pow(cluster_rad * resolution_x, 2)) {
          cnt++;
          history = true;
          break;
        } else {
          cnt = 0;
        }
      }

      //处理后的单点, 做鼠标操作
      if (centers.size()) {
        //如果前一次和当前都有两个触点, 认为是滚轮事件
        if (centers.size() == 2 && pre_count == 2) {
          iswheel = true;
          double dis = sqrt((pow(centers[0][0] - centers[1][0], 2) + pow(centers[0][1] - centers[1][1], 2)));

          if (dis < points_distance) {
            scale_in = true;
          } else {
            scale_in = false;

          }

          points_distance = dis;
          cnt = 0;
          select = true;
          isdouleclick = false;
        } else {

          if (history) {
            x = pre_x;
            y = pre_y;

            //如果上一次是空, 这次在点击到同一个位置, 认为是双击事件
            if (!isempty) {
              if (!(pow(pre_pre_x - pre_x, 2) + pow(pre_pre_y - pre_y, 2) < pow(cluster_rad * resolution_x, 2))) {
                isdouleclick = true;
              }
            }

            pre_pre_x = pre_x;
            pre_pre_y = pre_y;
          } else {
            pre_pre_x = pre_x;
            pre_pre_y = pre_y;
            pre_x = x;
            pre_y = y;
          }

        }



        isempty = false;

        if (cnt > 8 && select) { //同一位置检测到大于8次, 认为是右击事件
          //移动鼠标到(x, y)
          MousePositionAbsoluteEvent event;
          event.X = x;
          event.Y = y;
          SL::Input_Lite::SendInput(event);

          //左键单击
          // SendInput(MouseButtonEvent{true, MouseButtons::LEFT});
          // SendInput(MouseButtonEvent{false, MouseButtons::LEFT});
          //右建单击事件
          SendInput(MouseButtonEvent{true, MouseButtons::RIGHT});
          SendInput(MouseButtonEvent{false, MouseButtons::RIGHT});
          select = false;

          //鼠标滚轮事件
          //SendInput(MouseScrollEvent{1});
          //SendInput(MouseScrollEvent{-1});

        } else if (isdouleclick) {
          SendInput(MouseButtonEvent{true, MouseButtons::LEFT});
          SendInput(MouseButtonEvent{false, MouseButtons::LEFT});
          SendInput(MouseButtonEvent{true, MouseButtons::LEFT});
          SendInput(MouseButtonEvent{false, MouseButtons::LEFT});
          select = true;

        } else if (iswheel) {
          //鼠标滚轮事件
          if (scale_in) {
            SendInput(MouseScrollEvent{-1});
          } else {
            SendInput(MouseScrollEvent{1});
          }

        } else {
          //鼠标移动事件
          MousePositionAbsoluteEvent event;
          event.X = x;
          event.Y = y;
          SendInput(event);
        }
      } else {
        cnt = 0;
        isempty = true;
      }

      pre_count = centers.size();
    }
  }

  //退出程序, 关闭雷达连接
  laser.turnOff();
  laser.disconnecting();

  return 0;


}
