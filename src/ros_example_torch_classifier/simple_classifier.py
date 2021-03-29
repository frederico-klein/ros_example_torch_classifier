#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from std_srvs.srv import Trigger, TriggerResponse
from ros_example_torch_classifier.ros_classifier import RosClassifier, ClassifierRotator

class SimpleRosClassifier(RosClassifier):
    def __init__(self):
        super(SimpleRosClassifier, self).__init__()
        ##need to set training input
        ##need to set label topic
        ##need to set test topic
        ##need to set test labels

    def do_classify(self, req):
        return TriggerResponse(success=True, message= "Classification done")

    def do_predict(self, req):
        return TriggerResponse(success=True, message= "Prediction done")

if __name__ == '__main__':
    try:
        with ClassifierRotator(SimpleRosClassifier) as cr:
            rospy.spin()
    except rospy.ROSInterruptException:
            pass
