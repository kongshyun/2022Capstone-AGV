#!/usr/bin/env python
#-*- coding:utf-8 -*-
# command에 따라 함수를 다르게 설정 
#target_linear_vel(선속도)를 /cmd_vel로 퍼블리시하는 node
#publish 되는 값은 target_linear_vel
import rospy
from geometry_msgs.msg import Twist
import sys, select
import tty, termios

# 최고 속도 설정
MAX_LIN_VEL = 0.5
MAX_ANG_VEL = 3.0

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

# 현재 속도 출력
def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

# 속도를 완만하게 증가시킴 
def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

# 일정 범위 clamp 함수
# checkLinearLimitVelocity/checkAngularLimitVelocity에 사용되어
# 속도가 일정 범위를 넘어가지 못하게 함.
def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

# 병진 속도 범위 체크 함수
def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)
    
    return vel

# 회전 속도 범위 체크 함수
def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)

    return vel

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('Original')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        while(1):
            # 키 입력 하나 받음(멀티입력 불가)
            # 입력에 따라 속도 증가/감소
            key = getkey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            # 스페이스바나 s 가 입력되면 속도 0으로 설정
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            else:
                if key == '\x03': # ctrl + c 처리
                    break

            if status == 20 :
                print(msg)
                status = 0

            twist = Twist()

            # input , output이 다른 경우 속도를 완만하게 증가시킴(control velocity는 나중에 나오나?)
            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub.publish(twist) # /cmd_vel 메시지 퍼블리시 
            print('hello')

    except:
        print(e)

    finally:
        # ctrl + c 를 누르면 해당 구문 실행, 터틀봇이 이동하지 않도록 속도를 0으로 설정
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
