#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

import message_filters
from sklearn.metrics import accuracy_score
from std_srvs.srv import Empty
from std_msgs.msg import Float64

rospy.init_node("metric")

rospy.Service("start", Empty, start)
rospy.Service("eval", Empty, evaluate)

y = []
y_hat = []

pred = message_filters.Subscriber("y_hat", StringStamped)
real = message_filters.Subscriber("y", StringStamped)

realtimePublisher = rospy.Publisher("val", Float64)


compare_pair = [pred, real]
    
ts = message_filters.TimeSynchronizer(compare_pair, 10)
ts.registerCallback(append_pair)

def append_pair(pred, real):
    y.append(real)
    y_hat.append(pred)
    acc = accuracy_score(y, y_hat)
    realtimePublisher.publish(acc)

def start(req):
    y = []
    y_hat = []

def evaluate(req):
    acc = accuracy_score(y, y_hat)
    rospy.loginfo("acc %f"%acc)

