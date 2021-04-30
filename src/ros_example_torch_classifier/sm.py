#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import Empty, EmptyResponse

##splitter is spawned from a roslaunch with parameters


#call the splitter
#to get train and test publishers
# make them respond to triggers?
# as in  a service?
# or make it slow enough so everyone has a chance to get it?
# or put a clock input?

## now I have a training/ test split, go over all classifiers in sequence

rospy.init_node("state_machine")

myrate = rospy.Rate(1)

class_top      = "/classifiers1/a_classifier_tm1/get_next"
class_do_top   = "/classifiers1/a_classifier_tm1/classify"
class_pd_top   = "/classifiers1/a_classifier_tm1/predict"
splitter_top   = "/the_splitter/get_next"
talk_train_top = "/the_splitter/train/atalk"
talk_test_top  = "/the_splitter/test/atalk"

next_split      = rospy.ServiceProxy(splitter_top   , Trigger)
next_classifier = rospy.ServiceProxy(class_top      , Trigger)
just_talk       = rospy.ServiceProxy(talk_train_top , Empty)
test_talk       = rospy.ServiceProxy(talk_test_top , Empty)

class_do        = rospy.ServiceProxy(class_do_top, Trigger)
class_pd        = rospy.ServiceProxy(class_pd_top, Trigger)

rospy.wait_for_service(splitter_top)
next_split()
rospy.wait_for_service(class_top)
next_classifier()
##do a for loop on each of the classifier groups
rospy.wait_for_service(talk_train_top) ## this is the offline version
just_talk()

##I need to check the transition as well. so at some point after just_talk I need to be in a not finished state

while ( rospy.get_param("/the_splitter/train/finished")):
    myrate.sleep()

## now we are in the unfinished state, we wait.

while (not rospy.get_param("/the_splitter/train/finished")):
    myrate.sleep()

class_do()

##TODO: this is parallel with the train publishing, so it can be optimized
rospy.wait_for_service(talk_test_top) ## this is the offline version
test_talk()

while ( rospy.get_param("/the_splitter/test/finished")):
    myrate.sleep()

## now we are in the unfinished state, we wait.

while (not rospy.get_param("/the_splitter/test/finished")):
    myrate.sleep()

class_pd()

##i have data to classify something!
