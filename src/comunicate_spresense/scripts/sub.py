#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import paho.mqtt.client as mqtt     # MQTTのライブラリをインポート
import rospy
from rospy.core import rospyinfo
from std_msgs.msg import Header
from std_msgs.msg import Float64
import math

pub = rospy.Publisher('sensor1', Float64, queue_size=100)
rospy.init_node('Spresense1')

# ブローカーに接続できたときの処理
def on_connect(client, userdata, flag, rc):
  print("Connected with result code " + str(rc))  # 接続できた旨表示
  client.subscribe("/spresense1/angle")  # subするトピックを設定 

# ブローカーが切断したときの処理
def on_disconnect(client, userdata, flag, rc):
  if  rc != 0:
    print("Unexpected disconnection.")

# メッセージが届いたときの処理
def on_message(client, userdata, msg):
  # msg.topicにトピック名が，msg.payloadに届いたデータ本体が入っている
  angle=list(map(float,(str(msg.payload))[2:-1].split(" "))) 	
  print((str(msg.payload)))
  data=(angle[0])*math.pi/180
  pub.publish(data)


if __name__ == '__main__':
      # MQTTの接続設定
    client = mqtt.Client()                 # クラスのインスタンス(実体)の作成
    client.on_connect = on_connect         # 接続時のコールバック関数を登録
    client.on_disconnect = on_disconnect   # 切断時のコールバックを登録
    client.on_message = on_message         # メッセージ到着時のコールバック
    client.connect("localhost", 1883, 60)  # 接続先は自分自身

    rospy.init_node('Spresense1')
    client.loop_forever() 
