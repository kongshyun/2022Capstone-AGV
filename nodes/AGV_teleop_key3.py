#!/usr/bin/env python
#-*- coding:utf-8 -*-
# command에 따라 Twist(속도값)을 변화시키고, 
# Twist()를 /cmd_vel로 퍼블리시하는 node + Angle()를 /cmd_ang로 퍼블리시하는 node
# publish 되는 값은 linear.x , angular z
import rospy
from std_msgs.msg import Float64
import sys, select
import tty, termios
import time

# 최고 속도 설정
MAX_LIN_VEL = 0.22
MAX_ANG_VEL = 2.84

# 입력에 따라 증가/감소되는 선형/각속도 양
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your AGV
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

a = """
Velocity Send Success
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

#def velocity_profile(highest_theta, Length):


def talker():
    settings = termios.tcgetattr(sys.stdin)
    print(a)
    sec=0.0
    rate = rospy.Rate(10) # 루프를 10hz로 유지 
    
    while(sec<=0.5):
	    key = getkey()
	    velocity=-(16.0*sec*sec*sec)+(12.0*sec*sec)
	    target_linear_vel=velocity*60/(3.14*0.065)
	    sec=sec+0.02

	    #rospy.loginfo(target_linear_vel)
	   
	    pub.publish(target_linear_vel) # /cmd_vel 메시지 퍼블리시
            rospy.loginfo("Target_vel: %s" % target_linear_vel) 
	    rate.sleep() #rate대로 루프 속도 유지 
	    if key == ' ' or key == 's' :
		target_linear_vel=0.0
                break
    print("MAX!! ")
    while(sec>0.0):
            key = getkey()
            velocity=-(16.0*sec*sec*sec)+(12.0*sec*sec)

            target_linear_vel=velocity*60.0/(3.14*0.065)
            sec=sec-0.02
	    
	    pub.publish(target_linear_vel) # /cmd_vel 메시지 퍼블리시 
				
	    rospy.loginfo("Target_vel: %s" % target_linear_vel) 
	   
	    rate.sleep() #rate대로 루프 속도 유지 
	    if key == ' ' or key == 's' :
		target_linear_vel=0.0
		break
    sec=0.0
    target_linear_vel=0.0
    print(msg)
    
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('AGV_teleop_key3')
    pub=rospy.Publisher('/cmd_vel',Float64,queue_size=10)

    target_linear_vel=0.0
    try:
	print(msg)
	while(1):
	    key = getkey()
	    if key == 'w' :
		try:
		    talker()
                      
		except:
		    print(e)
	    elif key == 'd' :
		target_linear_vel=100.0
	    elif key == 'a' :
		target_linear_vel=60.0
	    elif key == ' ' or key == 's' :
		target_linear_vel=0.0
		print(target_linear_vel)
	    else:
                if key == '\x03': # ctrl + c 처리
                    break
            print("Velocity :  %s " % target_linear_vel)
	    pub.publish(target_linear_vel) #vel 값 publish하기

    except:
	print(e)
    finally:
        # ctrl + c 를 누르면 해당 구문 실행, 터틀봇이 이동하지 않도록 속도를 0으로 설정
        target_linear_vel=0.0
        pub.publish(target_linear_vel)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
         






















