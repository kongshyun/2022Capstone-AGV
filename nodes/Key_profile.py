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
 
         x
 
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
	    elif key == 'a' : #'2'값 pub
		pub.publish(2)
		print('----AGV GO SHORT!!----')
	    elif key == 'q' : #'3'값 pub
		pub.publish(5)
		print('----Left Turn----!!')
            
	    elif key == 'e' : #'5'값 pub
		pub.publish(3)
		print('----Right Turn----!!')

	    elif key == 'w' : #'6'값 pub
		pub.publish(6)
		print('----180 Turn----!!')

	    else:
                if key == '\x03': # ctrl + c 처리
                    break

	    pub_data=0 # pub_data한번 보내고 초기화
	    pub.publish(pub_data)
	    if (sec>30) :
		print(msg)
		sec=0

    except:
	print(e)
    finally:
	pub_data=0
        pub.publish(pub_data)
	print('ALL STOP')
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
         
