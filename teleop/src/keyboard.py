#!/usr/bin/env python

import rospy
from slip_control_communications.msg import input_model
import curses
#import signal
#TIMEOUT = 0.1 # number of seconds your want for timeout
forward = 0;
left = 0;

# def interrupted(signum, frame):
#     "called when read times out"
#     global forward
#     forward = 0
#     global left
#     left = 0
#     stdscr.addstr(2, 20, "Stop")
#     stdscr.addstr(2, 25, '%.2f' % forward)
#     stdscr.addstr(3, 20, "Stop")
#     stdscr.addstr(3, 25, '%.2f' % left)
# signal.signal(signal.SIGALRM, interrupted)

# def input():
#     try:
#             foo = stdscr.getch()
#             return foo
#     except:
#             # timeout
#             return

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('keyboard_talker_node', anonymous=True)
pub = rospy.Publisher('input_model_topic', input_model, queue_size=10)

# set alarm
#signal.alarm(TIMEOUT)
#s = input()
# disable the alarm after success
#signal.alarm(0)
#print 'You typed', s

stdscr.refresh()

maxl = 100
minl = -100
maxf = 20
minf = -20
step = 2

key = ''
while key != ord('q'):
#       signal.setitimer(signal.ITIMER_REAL,0.05)
#       key = input()
    key = stdscr.getch()
    stdscr.refresh()
    #       signal.alarm(0)

    if key == curses.KEY_UP:
        forward = forward + step
        forward = min(max(forward,minf),maxf)
        stdscr.addstr(2, 20, "Up  ")
        stdscr.addstr(2, 25, '%.2f' % forward)
        stdscr.addstr(5, 20, "    ")
    elif key == curses.KEY_DOWN:
        forward = forward - step;
        forward = min(max(forward,minf),maxf)
        stdscr.addstr(2, 20, "Down")
        stdscr.addstr(2, 25, '%.2f' % forward)
        stdscr.addstr(5, 20, "    ")
    if key == curses.KEY_LEFT:
        left = left - 5*step;
        left = min(max(left,minl),maxl)
        stdscr.addstr(3, 20, "left")
        stdscr.addstr(3, 25, '%.2f' % left)
        stdscr.addstr(5, 20, "    ")
    elif key == curses.KEY_RIGHT:
        left = left + 5*step;
        left = min(max(left,minl),maxl)
        stdscr.addstr(3, 20, "rgt ")
        stdscr.addstr(3, 25, '%.2f' % left)
        stdscr.addstr(5, 20, "    ")
    if key == curses.KEY_DC:
        left = 0
        forward = 0
        stdscr.addstr(5, 20, "Stop")

msg = input_model()
msg.velocity = forward
msg.angle = left
pub.publish(msg)
curses.endwin()
