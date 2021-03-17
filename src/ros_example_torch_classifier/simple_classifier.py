#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from std_srvs.srv import Trigger, TriggerResponse

class ClassifierRotator():
    def __init__(self):
        ##each new split I need to spawn a new RosClassifier
        ## I am going to have some services here that do things:

        pass

    def __enter__(self):
        self.get_next = rospy.Service("~get_next", Trigger, self.do_one_thing)

        return self

    def __exit__(seld, *exc):
        self.get_next.shutdown("\n\texc list: {}\n, {}".format(*exc,exc[0]))

    def do_one_thing(self):
        with RosClassifier() as rc:
            rospy.spin()

    def do_something_else(self):
        pass


class RosClassifier():
    def __init__(self):
        ##need to set training input
        ##need to set label topic
        ##need to set test topic
        ##need to set test labels
        pass

    def __enter__(self):
        self.clf_do = rospy.Service("~classify", Trigger, self.do_classify)
        self.clf_predict = rospy.Service("~predict", Trigger, self.do_predict)
        return self

    def __exit__(seld, *exc):
        self.get_next.shutdown("\n\texc list: {}\n, {}".format(*exc,exc[0]))

    def do_classify(self, req):
        return TriggerResponse(success=True, message= "Classification done")

    def do_predict(self, req):
        return TriggerResponse(success=True, message= "Prediction done")

if __name__ == '__main__':
    try:
        with ClassifierRotator() as cr:
            rospy.spin()
    except rospy.ROSInterruptException:
            pass
