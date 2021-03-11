#!/usr/bin/env python
PKG = 'ros_example_torch_classifier'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.
import rospy
import sys
import unittest
#import subprocess
import numpy as np
import pandas as pd
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

class TestSplitting(unittest.TestCase):
    ## test 1 == 1
    def test_splitting_srv(self):
        ## I need to come up with a DataFrame- from pandas help, extended:
        with Splitter(df,3,2, seed=42) as asplitter:
            get_next_srv = rospy.ServiceProxy("~get_next", Empty)
            self.assertEqual(get_next_srv(),EmptyResponse())
            ##should have updated the traintalker and the testtalker

    def test_splitting(self):
        ## I need to come up with a DataFrame- from pandas help, extended:
        with Splitter(df,3,2, seed=42) as asplitter:
            get_next_srv = rospy.ServiceProxy("~get_next", Empty)
            get_next_srv()
            self.assertIsNotNone(asplitter.traintalker )
            self.assertIsNotNone(asplitter.testtalker )
            self.assertIsNotNone(asplitter.k )
    #
    #
    # def test_one_equals_one(self): # only functions with 'test_'-prefix will be run!
    #     self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    import rostest

    rostest.rosrun(PKG, 'test_splitting', TestSplitting)
