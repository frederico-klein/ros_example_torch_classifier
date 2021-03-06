#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import message_filters
#from std_msgs.msg import String ## we need it stamped or we cant use TimeSynchronizer
from ros_example_torch_classifier.msg import StringStamped
from ros_example_torch_classifier.utils import check_remap
from std_srvs.srv import Trigger, TriggerResponse


#def check_remap(listo):
#    for name in listo:
#        rospy.logdebug("the name that %s is resolving to: %s"%(name, rospy.resolve_name(name)))

class ClassifierRotator():
    def __init__(self, classifier = None):
        ##each new split I need to spawn a new RosClassifier
        ## I am going to have some services here that do things:
        rospy.init_node("cfr", anonymous=True, log_level=rospy.DEBUG)
        self.classifier_prototype = classifier
        self.classifier = None

    def __enter__(self):
        self.get_next = rospy.Service("~get_next", Trigger, self.do_one_thing)

        return self

    def __exit__(self, *exc):
        self.get_next.shutdown("\n\texc list: {}\n, {}".format(*exc,exc[0]))

    def do_one_thing(self, req):
        #with RosClassifier() as rc:
        #self.classifier = rc
        #    rospy.spin()
        if self.classifier is not None:
            self.classifier.stop("Next instance called.")
        self.classifier =  self.classifier_prototype()
        self.classifier.start()
        return TriggerResponse(success=True, message= "response")

    def do_something_else(self):
        pass


class RosClassifier():
    def __init__(self):
        self.train_data = []
        self.test_data = []
        ### this is awkward; ideally we would have them published together with a custom message,
        ### TODO: consider if custom messages is better here.

        ##need to set training input
        ##need to set label topic

        check_remap(["train_in", "train_label", "test_in", "test_label"])
        rospy.logdebug("the name that train_in is resolving to: %s"%rospy.resolve_name("train_in"))
        self.training_input = message_filters.Subscriber('train_in', StringStamped)
        self.training_label = message_filters.Subscriber('train_label', StringStamped)

        train_sub_pair = [self.training_input, self.training_label]
        self.train_ts = message_filters.TimeSynchronizer(train_sub_pair , 10)
        #self.ts = message_filters.ApproximateTimeSynchronizer(train_sub_pair , 10, 0.1, allow_headerless=True)
        self.train_ts.registerCallback(self.train_callback)

        ##need to set test topic
        ##need to set test labels
        self.test_input = message_filters.Subscriber('test_in', StringStamped)
        self.test_label = message_filters.Subscriber('test_label', StringStamped)

        test_sub_pair = [self.test_input, self.test_label]
        self.test_ts = message_filters.TimeSynchronizer(test_sub_pair , 10)
        #self.test_ts = message_filters.ApproximateTimeSynchronizer(test_sub_pair , 10, 0.1, allow_headerless=True)
        self.test_ts.registerCallback(self.test_callback)

    def train_callback(self, data, label):
        #maybe we just collect everything until we get a do_train
        #this will fail for big dataset strategies where we do not want to keep
        #the whole thing ever, but just
        rospy.logwarn_once("I'm receiving data.")
        self.train_data.append((data,label))
        #pass

    def test_callback(self, data, label):
        self.test_data.append((data,label))

    def __enter__(self):
        self.start()
        return self

    def start(self):
        self.clf_do = rospy.Service("~classify", Trigger, self.do_train)
        self.clf_predict = rospy.Service("~predict", Trigger, self.do_predict)

    def stop(self, reason = "No reason given."):
        self.clf_do.shutdown(reason)
        self.clf_predict.shutdown(reason)

    def __exit__(self, *exc):
        reason = "\n\texc list: {}\n, {}".format(*exc,exc[0])
        self.stop(reason = reason)

    def do_train(self, req):
        assert(self.train_data)
        rospy.logwarn("training not implemented")
        return TriggerResponse(success=True, message= "Stub classification done")

    def do_predict(self, req):
        assert(self.test_data)
        rospy.logwarn("prediction not implemented")
        return TriggerResponse(success=True, message= "Stub prediction done")

if __name__ == '__main__':
    try:
        with ClassifierRotator(RosClassifier) as cr:
            rospy.spin()
    except rospy.ROSInterruptException:
            pass
