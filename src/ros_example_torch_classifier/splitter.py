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
    def __init__(self, dataframe, n_splits=2, n_repeats=2, seed=42, cv_type="RepeatedStratifiedKFold"):
        if cv_type is "RepeatedStratifiedKFold":
            self.rskf = RepeatedStratifiedKFold(n_splits=n_splits, n_repeats=n_repeats,
                 random_state=seed)
        elif cv_type is "kfolds" or cv_type is "test_train_splits":
            #TODO: not implemented: kfolds, test_train_splits
            raise Exception("Cross validation type not yet implemented: %s"%cv_type)
        else:
            raise Exception("Cross validation type not defined: %s"%cv_type)
        self.data = dataframe
        self.len = len(self.data)
        self.traintalker = None
        self.testtalker = None
        self.k = None
        self.get_next = None

    def __enter__(self):
        self.get_next = rospy.Service("~get_next", Trigger, self.tick)
        # self.get_next = rospy.Service("~get_next", Empty, self.tick)
        return self

    def  __exit__(self, *exc):
        self.get_next.shutdown("\n\texc list: {}\n, {}".format(*exc,exc[0]))

    def tick(self,req):
        self.k = next(self.produce_split())
        rospy.logdebug("k: %d"%self.k)
        return TriggerResponse(success=True, message= "Hello")
        # return EmptyResponse()

    def produce_split(self):
        bogusX = np.random.rand(self.len,2)
        bogusY = np.round(np.random.rand(self.len,1))

        for k, (train, test) in enumerate(self.rskf.split(bogusX, bogusY)):
            self.traintalker = dataset.CsvTalker(name="train", data=self.data.take([train]))
            self.testtalker = dataset.CsvTalker(name="test", data=self.data.take([test]))
            yield k

if __name__ == '__main__':
    try:
        rospy.init_node('csv_splitter', anonymous=True)

        n_splits = rospy.get_param("~n_splits", default=5)
        cv_type = rospy.get_param("~cv_type", default="RepeatedStratifiedKFold")
        n_repeats = rospy.get_param("~n_repeats", default=2)
        seed = rospy.get_param("~seed", default=22)
        use_random_splits = rospy.get_param("~use_random_splits", default=True)
        dataset_path = rospy.get_param("~dataset_path", default="articles.csv")

        if use_random_splits:
            seed = np.random.randint(0, np.iinfo(np.uint32).max)
            rospy.loginfo("Using random seed: %d"%seed)
        else:
            rospy.loginfo("Using fixed seed: %d"%seed)

        with Splitter(pd.read_csv(dataset_path),n_splits=n_splits,n_repeats=n_repeats, cv_type=cv_type, seed=42) as asplitter:
            rospy.spin()

    except rospy.ROSInterruptException:
        pass
