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
    d: 1ms senario
    a: stop
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
	    if key == 'w' :
		try:
		    talker()
                      
		except:
		    print(e)
	    elif key == 'd' :
                pub_data=6
		sec=sec+1
		pub.publish(pub_data)
		print('AGV GO!!')
	    elif key == 'a' :
		pub_data=3
		sec=sec+1
		pub.publish(pub_data)
		print('OFF')



	    elif key == ' ' or key == 's' :
		target_linear_vel=0.0
		print(target_linear_vel)
	    else:
                if key == '\x03': # ctrl + c 처리
                    break
	   
	    
	    
	    if (sec>10) :
		print(msg)
		sec=0

    except:
	print(e)
    finally:
        pub.publish(pub_data)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
         





















