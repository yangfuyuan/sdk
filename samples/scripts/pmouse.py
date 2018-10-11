# -*- coding: UTF-8 -*-

import sys, getopt
from pymouse import PyMouse 

def main(argv):
      cmd, x, y = argv
      if len(sys.argv) < 4:
            print "pymouse.py move x y"
            sys.exit()
      m = PyMouse()
      #print m.screen_size()
      if cmd == "move":
            m.move(int(x), int(y))
      elif cmd == "click":
            m.click(int(x), int(y),1|2)

if __name__ == "__main__":
      main(sys.argv[1:])

