#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import copy
import ros_example_torch_classifier.dataset as dataset
import pandas as pd
import numpy as np
#from std_srvs.srv import Empty, EmptyResponse
from std_srvs.srv import Trigger, TriggerResponse
from sklearn.model_selection import RepeatedStratifiedKFold

class Splitter():
    def __init__(self, dataframe, n_splits, n_repeats, seed=42):
        self.rskf = RepeatedStratifiedKFold(n_splits=n_splits, n_repeats=n_repeats,
                 random_state=seed)
        self.data = dataframe
        self.len = len(self.data)
        self.traintalker = None
        self.testtalker = None
        self.k = None
        self.get_next = None

    def __enter__(self):
        self.get_next = rospy.Service("~get_next", Empty, self.tick)
        return self

    def  __exit__(self, *exc):
        self.get_next.shutdown("\n\texc list: {}\n, {}".format(*exc,exc[0]))

    def tick(self,req):
        self.k = next(self.produce_split())
        rospy.logdebug("k: %d"%self.k)
        return EmptyResponse()

    def produce_split(self):
        bogusX = np.random.rand(self.len,2)
        bogusY = np.round(np.random.rand(self.len,1))

        for k, (train, test) in enumerate(self.rskf.split(bogusX, bogusY)):
            self.traintalker = dataset.CsvTalker(name="train", data=self.data)
            self.testtalker = dataset.CsvTalker(name="test", data=self.data)
            yield k

if __name__ == '__main__':
    try:
        rospy.init_node('csv_splitter', anonymous=True)

        n_splits = rospy.get_param("~n_splits", default=5)
        n_repeats = rospy.get_param("~n_repeats", default=2)
        dataset_path = rospy.get_param("~dataset_path", default="articles.csv")
        with Splitter(pd.read_csv(dataset_path),n_splits=n_splits,n_repeats=n_repeats) as asplitter:
            rospy.spin()

    except rospy.ROSInterruptException:
        pass
