
from subprocess import Popen, PIPE
import re
import numpy as np
from graphics import *
import time
import sys

# prog_path = "../build/demo"
# p = Popen([prog_path], stdout=PIPE, stdin=PIPE)
# p.stdin.write(b"start\n")
# p.stdin.flush()


# Data Path
# data_path = "./out.txt"
# f = open(data_path, "r")

# Live Data Display
window = GraphWin("Sensor", 1200, 1200)
window.setCoords(-100, -500, 500, 500)
backlog = []
clear = False

for line in sys.stdin:
    # print(line)
    line.strip()
    words = re.split(", |\s+", line)

    if len(backlog) == 720 and clear is False:
        clear = True

    circle = Circle(Point(float(words[0]), float(words[1])), 1)
    circle.setFill("red")
    circle.draw(window)
    backlog.append(circle)
    # time.sleep(0.1000)
    #circle.undraw()

    if clear:
        circ = backlog.pop(0)
        circ.undraw()




# # line = f.readline().strip()
# for line in f:
#     line.strip()
#     words = re.split(", |\s+", line)
#     # words = re.split(", |\s+", line.decode("utf-8"))
#     raw = []
#     # if len(words) == 3 and words[0] != 'Node':
#     raw.append(float(words[0])) # x-coord
#     raw.append(float(words[1])) # y-coord

#     if raw[0] != 0 and raw[1] != 0:
        
#         circle = Circle(Point(raw[0], raw[1]), 1)
#         circle.setFill("red")
#         circle.draw(window)
#         #time.sleep(0.1000)
#         # circle.undraw()


















