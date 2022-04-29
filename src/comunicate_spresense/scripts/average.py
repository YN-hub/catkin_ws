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
thereshold=1*math.pi/180
isfirst1=True
isfirst2=True
prev1=0
prev2=0
def callback(angle, id):
  global isfirst1,isfirst2,prev1,prev2
  alpha=0.01
  if(id == 0):
      data.header.stamp = rospy.Time.now()
      data.name =  ['base_link_to_body', 'body_to_arm0', 'arm0_to_arm1', 'arm1_to_endeffector']
      if isfirst1==True:
        data.position[1]=angle.data-math.pi/2.8
        prev1=angle.data-math.pi/2.8
        rospy.loginfo("first0")
        isfirst1=False
      else:
        data.position[1]=alpha*(angle.data-math.pi/2.8)+(1-alpha)*prev1
        prev1=angle.data-math.pi/2.8
      data.velocity = []
      data.effort = []
      pub.publish(data)
  else :
      data.header.stamp = rospy.Time.now()
      data.name =  ['base_link_to_body', 'body_to_arm0', 'arm0_to_arm1', 'arm1_to_endeffector']
      if isfirst2==True:
        data.position[2]=angle.data-data.position[1]-(40*math.pi/180)-(40*math.pi/180)
        prev2=angle.data-data.position[1]-(40*math.pi/180)-(40*math.pi/180)
        isfirst2=False
        rospy.loginfo("first1")
      else:
        data.position[2]=alpha*(angle.data-data.position[1]-(40*math.pi/180)-(40*math.pi/180))+(1-alpha)*prev2
        prev2=angle.data-data.position[1]-(40*math.pi/180)-(40*math.pi/180)
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