#!/usr/bin/env python
#-*- coding:utf-8 -*-
# 두개의 토픽에 속도값 전달하는 노드
# cmd_vel_left 왼쪽바퀴, cmd_vel_right 오른쪽 바퀴
# profile 계산하는 함수 넣기
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
   q    w    e
   a    s    d
        x
w : go_1ms
a : turn
q/e : 100, 60 rpm

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

a = """
Velocity Send Success
"""
settings = termios.tcgetattr(sys.stdin)
rospy.init_node('AGV_teleop_profile')
pub=rospy.Publisher('/cmd_vel_left',Float64,queue_size=20)
pub_servo=rospy.Publisher('/cmd_servo',Float64,queue_size=10)
#pub_right=rospy.Publisher('/cmd_vel_right',Float64,queue_size=20)
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

def go_1ms():
    settings = termios.tcgetattr(sys.stdin)
    print(a)
    sec=0.0
    degree=0.0
    rate = rospy.Rate(100) # 루프를 10hz로 유지 (0.1초)
    rate_t=rospy.Rate(2) #2초 
    while(sec<=0.5):
	    key = getkey()
	    velocity=-(16.0*sec*sec*sec)+(12.0*sec*sec)
	    target_linear_vel=velocity*60/(3.14*0.065)
	    sec=sec+0.02
	    degree=degree+1
	    pub.publish(target_linear_vel) # /cmd_vel 메시지 퍼블리시
	    pub_servo.publish(degree)	# /cmd_servo 메시지 퍼블리시 
				
            rospy.loginfo("Target_vel: %s \t Degree:  %s" % (target_linear_vel,degree)) 
	    rate.sleep() #rate대로 루프 속도 유지 
	    if key == ' ' or key == 's' :
		target_linear_vel=0.01
                break
    while(sec>0.0):
            key = getkey()
            velocity=-(16.0*sec*sec*sec)+(12.0*sec*sec)
            target_linear_vel=velocity*60.0/(3.14*0.065)
            sec=sec-0.02
	    
	    pub.publish(target_linear_vel) # /cmd_vel 메시지 퍼블리시 
	    pub_servo.publish(degree)	
	    degree=degree+1		
	    rospy.loginfo("Target_vel: %s \t Degree: %s" % (target_linear_vel,degree)) 
	    rate.sleep() #rate대로 루프 속도 유지 
	    if key == ' ' or key == 's' :
		target_linear_vel=0.0
		break
    sec=0.0
    target_linear_vel=0.0
    print(msg)

def turn():
    settings = termios.tcgetattr(sys.stdin)
    print(a)
    sec=0.0
    rate = rospy.Rate(10) # 루프를 10hz로 유지 (0.1초)
    while(sec<1):
	key=getkey()
	
        target_linear_vel=100.0
        sec=sec+0.05
        pub.publish(target_linear_vel)
	rospy.loginfo("\tTarget_vel: %s \t Degree:  %s" % (target_linear_vel,degree))
	rate.sleep()
	if key == ' ' or key == 's' :
	    target_linear_vel=0.0
	    break
    sec=0.0 
    target_linear_vel=0.0
    print(msg)

def degree():
    degree=0.0
    rate = rospy.Rate(10)
    while(degree<179):
        degree=degree+1
	pub_servo.publish(degree)
	rospy.loginfo("Degree: %s" % degree) 
        rate.sleep()
    
if __name__=="__main__":

    target_linear_vel=0.0
    try:
	print(msg)
	while(1):
	    key = getkey()
	    if key == 'w' :#1ms속도 프로파일 직선주행
		try:
		    go_1ms()
                      
		except:
		    print(e)
	    elif key == 'a' :#제자리회전 
		try:
		    turn()
		except:
		    print(e)
	    elif key == 'd' :#자세보정 움직임 
		try:
                    degree()
		except:
		    print(e)
	    elif key == 'q' :
		target_linear_vel=60.0
		print("Velocity :  %s " % target_linear_vel)
	    elif key == 'e' :
		target_linear_vel=100.0
		print("Velocity :  %s " % target_linear_vel)
	    elif key == 'x' :
		target_linear_vel=-100.0
		print("Velocity :  %s " % target_linear_vel)

	    elif key == ' ' or key == 's' :
		target_linear_vel=0.0
                print("Velocity :  %s " % target_linear_vel)
	    else:
                if key == '\x03': # ctrl + c 처리
                    break
	    pub.publish(target_linear_vel) #vel 값 publish하기

    except:
	print(e)
    finally:
        # ctrl + c 를 누르면 해당 구문 실행, 터틀봇이 이동하지 않도록 속도를 0으로 설정
        target_linear_vel=0.0
        pub.publish(target_linear_vel)
	pub_servo.publish(0) #vel 값 publish하기
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
         























