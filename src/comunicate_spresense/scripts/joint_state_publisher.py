#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

pub = rospy.Publisher('joint_states', JointState, queue_size=100)
data = JointState()
data.header = Header()
data.position=[0,0,0,0]
thereshold=0.7*math.pi/180

def callback(angle, id):
  if(id == 0):
      data.header.stamp = rospy.Time.now()
      data.name =  ['base_link_to_body', 'body_to_arm0', 'arm0_to_arm1', 'arm1_to_endeffector']
      rospy.loginfo("OK")
      if abs(data.position[1]-(angle.data-math.pi/2.8))>thereshold:
        data.position[1]=angle.data-math.pi/2.8
      data.velocity = []
      data.effort = []
      pub.publish(data)
  else :
      data.header.stamp = rospy.Time.now()
      data.name =  ['base_link_to_body', 'body_to_arm0', 'arm0_to_arm1', 'arm1_to_endeffector']
      rospy.loginfo("OK")
      if abs(data.position[2]-(angle.data-data.position[1]-(40*math.pi/180)-(40*math.pi/180)))>thereshold:
        data.position[2]=angle.data-data.position[1]-(40*math.pi/180)-(40*math.pi/180)
      data.velocity = []
      data.effort = []
      pub.publish(data)


def listener():
    rospy.init_node('listener', anonymous=True)

    sub_0 = rospy.Subscriber("/sensor1", Float64, callback, callback_args=0)
    sub_1 = rospy.Subscriber("/sensor2", Float64, callback, callback_args=1)

    rospy.spin()

if __name__ == '__main__':
    listener()