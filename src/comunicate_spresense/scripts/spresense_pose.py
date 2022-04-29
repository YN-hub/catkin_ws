#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import paho.mqtt.client as mqtt     # MQTTのライブラリをインポート
import rospy
from rospy.core import rospyinfo
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64
import math

angle=[0,0,0]
pub = rospy.Publisher('joint_states', JointState, queue_size=100)
rospy.init_node('angle_to_joint_state')

# ブローカーに接続できたときの処理
def on_connect(client, userdata, flag, rc):
  print("Connected with result code " + str(rc))  # 接続できた旨表示
  client.subscribe("/spresens1/angle")  # subするトピックを設定 

# ブローカーが切断したときの処理
def on_disconnect(client, userdata, flag, rc):
  if  rc != 0:
    print("Unexpected disconnection.")

# メッセージが届いたときの処理
def on_message(client, userdata, msg):
  # msg.topicにトピック名が，msg.payloadに届いたデータ本体が入っている
  angle=list(map(float,(str(msg.payload))[2:-1].split(" "))) 	
  print((str(msg.payload)))
  data = JointState()
  data.header = Header()
  data.header.stamp = rospy.Time.now()
  data.name =  ['base_link_to_body', 'body_to_arm0', 'arm0_to_arm1', 'arm1_to_endeffector']
  data.position=[0,0,0,0]
  rospy.loginfo("OK")
  data.position[1]=(angle[0])*math.pi/180
  data.velocity = []
  data.effort = []
  pub.publish(data)


if __name__ == '__main__':
      # MQTTの接続設定
    client = mqtt.Client()                 # クラスのインスタンス(実体)の作成
    client.on_connect = on_connect         # 接続時のコールバック関数を登録
    client.on_disconnect = on_disconnect   # 切断時のコールバックを登録
    client.on_message = on_message         # メッセージ到着時のコールバック
    client.connect("localhost", 1883, 60)  # 接続先は自分自身

    rospy.init_node('angle_to_joint_state')
    client.loop_forever() 