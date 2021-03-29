#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Trigger, TriggerResponse

##splitter is spawned from a roslaunch with parameters


#call the splitter
#to get train and test publishers
# make them respond to triggers?
# as in  a service?
# or make it slow enough so everyone has a chance to get it?
# or put a clock input?

## now I have a training/ test split, go over all classifiers in sequence

next_split = rospy.ServiceProxy("csv_splitter/get_next", Trigger)
next_classifier = rospy.ServiceProxy("/classifiers2/a_classifier2_tm2/get_next", Trigger)



##do a for loop on each of the classifier groups
