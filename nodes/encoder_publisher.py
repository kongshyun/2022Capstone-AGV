#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import PointStamped # 엔코더 메시지

if __name__=="__main__":
    
    rospy.init_node('encoder_publisher')   
    encoder = rospy.Publisher("encoder_data", PointStamped, queue_size=50)

    pst = PointStamped()
    pst.point.x = 0
    pst.point.y = 0
    pst.point.z = 0
    rospy.loginfo("publishing fake encoder")
    try:
       while not rospy.is_shutdown():
         
           pst.header.stamp = rospy.Time.now()
           pst.point.x += 3
           pst.point.y += 2
           
           encoder.publish(pst)
           rate = rospy.Rate(50)
           rate.sleep()
    except :
       pass

    finally : 


       
       rospy.loginfo("Good bye!")
    
