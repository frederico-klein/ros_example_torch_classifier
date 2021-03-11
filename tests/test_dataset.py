#!/usr/bin/env python
PKG = 'ros_example_torch_classifier'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.
import rospy, rosservice
import sys
import unittest
import inspect

#this_function_name = inspect.currentframe().f_code.co_name
#import subprocess
import numpy as np
import pandas as pd
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
from ros_example_torch_classifier.splitter import Splitter
from ros_example_torch_classifier.dataset import CsvTalker

rospy.init_node("transient_test_node", anonymous=True, log_level=rospy.DEBUG)

df_dict = {'num_legs': [2, 4, 8, 0, 6, 10],
           'num_wings': [2, 0, 0, 0, 4, 0],
           'num_specimen_seen': [10, 2, 1, 8, 3, 0],
           'do_I_like_it': [0, 1, 1, 1, 0, 0]}
df = pd.DataFrame(df_dict,
          index=['falcon', 'dog', 'spider', 'fish', 'dragonfly', 'limulus'])

class TestDataset(unittest.TestCase):
    def test_publish(self): # only functions with 'test_'-prefix will be run!
        with CsvTalker(data=df, name="test_data", loop_forever = False) as atalker:
            atalker.atalk() #cannot use the blocking version or this will never exit
            errorlist = []
            for key, value in df_dict.items():
                try:
                    rospy.wait_for_message("test_data/"+ key, String , timeout=1) ## this will work because I am using a latch. otherwise it will take too long and fail, because the dataset is too small
                except:
                    errorlist.append(key)
            self.assertListEqual(errorlist,[],"topics failed to publish: {}".format(errorlist))
    def test_asynch_publish(self): # only functions with 'test_'-prefix will be run!
        with CsvTalker(data=df, name="test_data") as atalker:
            asynchronous_talk_srv = rospy.ServiceProxy("~atalk", Empty)
            rospy.loginfo("If this message appears")
            asynchronous_talk_srv() ## maybe also blocking
            rospy.loginfo(" at the same time as this one, this method is not blocking")
            errorlist = []
            for key, value in df_dict.items():
                try:
                    rospy.wait_for_message("test_data/"+ key, String, timeout=1) ## this will work because I am using a latch. otherwise it will take too long and fail, because the dataset is too small
                except:
                    errorlist.append(key)
            self.assertListEqual(errorlist,[],"topics failed to publish: {}".format(errorlist))

    def test_publish_one(self): # only functions with 'test_'-prefix will be run!
        this_function_name = inspect.currentframe().f_code.co_name
        rospy.loginfo("starting test: {}".format(this_function_name))
        with CsvTalker(data=df, name="test_data") as atalker:
            asynchronous_single_srv = rospy.ServiceProxy("~clock", Empty)
            asynchronous_single_srv() ## maybe also blocking, but it is very short.
            errorlist = []
            for key, value in df_dict.items():
                try:
                    rospy.wait_for_message("test_data/"+ key, String, timeout=5) ## this will work because I am using a latch. otherwise it may take too long and fail, because the dataset is too small
                except:
                    errorlist.append(key)
            self.assertListEqual(errorlist,[],"topics failed to publish: {}".format(errorlist))

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_dataset', TestDataset)
