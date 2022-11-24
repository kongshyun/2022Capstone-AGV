#!/usr/bin/env python
#-*- coding:utf-8 -*-
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from std_msgs.msg import Int16
import sys, select
import tty, termios
import time


msg = """


Control Your AGV
---------------------------
   q     w     e

   a           d
 
   z     x     c
 
d/a: 1ms long/short
q/e : left/right 90 turn
w : 180 turn

---------------------------
space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

# 키입력
def getkey():
    
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) 
    return key




pub=rospy.Publisher('/cmd_vel',Int16,queue_size=100)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('Key_profile', anonymous=True)
    try:
	print(msg)
 	pub_data=0
        sec=0
	while(1):
	    
	    key = getkey()
	    if key == 'd' : #'1'값 pub
		pub.publish(1)
		print('----AGV GO LONG!!----')
		pub.publish(0)
	    elif key == 'a' : #'2'값 pub
		pub.publish(2)
		print('----AGV GO SHORT!!----')
		pub.publish(0)
	    elif key == 'q' : #'3'값 pub
		pub.publish(5)
		print('----Left Turn----!!')
		pub.publish(0)
            
	    elif key == 'e' : #'5'값 pub
		pub.publish(3)
		print('----Right Turn----!!')
		pub.publish(0)

	    elif key == 'w' : #'6'값 pub
		pub.publish(6)
		print('----180 Turn----!!')
		pub.publish(0)
	    elif key == 'z' : 
		pub.publish(7)
		print('----small left----!!')
		pub.publish(0)
	    elif key == 'c' :
		pub.publish(8)
		print('----small right----!!')
		pub.publish(0)

	    else:
                if key == '\x03': # ctrl + c 처리
                    break

	    #pub_data=0 # pub_data한번 보내고 초기화
	    pub.publish(0)
	    if (sec>10) :
		print(msg)
		sec=0

    except:
	print(e)
    finally:
	pub_data=0
        pub.publish(pub_data)
	print('ALL STOP')
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
         





















