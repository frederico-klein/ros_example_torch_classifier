#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import copy
import pandas as pd
import numpy as np
#from std_srvs.srv import Empty, EmptyResponse
from std_srvs.srv import Trigger, TriggerResponse
from sklearn.model_selection import RepeatedStratifiedKFold
import ros_example_torch_classifier.utils as u
import ros_example_torch_classifier.dataset as dataset


u.VVV = True

class Splitter():
    def __init__(self, dataframe, n_splits=2, n_repeats=2, seed=42,
                stamped = True,
                cv_type="RepeatedStratifiedKFold"):
        if cv_type == "RepeatedStratifiedKFold":
            self.rskf = RepeatedStratifiedKFold(n_splits=n_splits, n_repeats=n_repeats,
                 random_state=seed)
        elif cv_type == "kfolds" or cv_type == "test_train_splits":
            #TODO: not implemented: kfolds, test_train_splits
            raise Exception("Cross validation type not yet implemented: %s"%cv_type)
        else:
            raise Exception("Cross validation type not defined: %s, type: %s"%(cv_type,type(cv_type)))
        self.data = dataframe
        self.len = len(self.data)
        self.traintalker = None
        self.testtalker = None
        self.k = None
        self.get_next = None
        self.stamped = stamped
# this solution will not work for very big datasets, but special care will have to be put here once we want to load specially large sets. 
# I will keep a copy of the whole set loaded only so I properly dump without needing to iterate through splits or integrate a big part of dataset.py code in here to make it consistent 
        self.alltalker = dataset.CsvTalker(name="all", loop_forever= False,
                data=self.data)
        self.alltalker.start() ## should create the dump services

    def __enter__(self):
        self.get_next = rospy.Service("~get_next", Trigger, self.tick)
        # self.get_next = rospy.Service("~get_next", Empty, self.tick)
        bogusX = np.random.rand(self.len,2)
        bogusY = np.round(np.random.rand(self.len,1))
        rospy.logdebug(f"self.len: {self.len}")
        rospy.logdebug(f"bogusX: {bogusX}")
        rospy.logdebug(f"bogusY: {bogusY}")
        self.enumerate = enumerate(self.rskf.split(bogusX, bogusY))
        return self

    def stop(self, reason = "No reason given"):
        self.get_next.shutdown(reason)

    def update(self):
        u.nlogvvv("splitter update called.")
        try:
            if self.traintalker:
                u.nlogvvv("trying to update traintalker")
                self.traintalker.update()
            else:
                u.nlogvvv("traintalker NOT DEFINED. DOING NOTHING")
                rospy.logwarn_throttle(60, "traintalker not defined yet. not updating")
            if self.testtalker:
                u.nlogvvv("trying to update testtalker")
                self.testtalker.update()
            else:
                u.nlogvvv("testtalker NOT DEFINED. DOING NOTHING")
                rospy.logwarn_throttle(60, "testtalker not defined yet. not updating")
            return True
        except rospy.ROSInterruptException:
            return False

    def __exit__(self, *exc):
        # deregistering services
        reason = "\n\texc list: {}\n, {}".format(*exc,exc[0])
        self.stop(reason = reason)

    def tick(self,req):
        succ, response = self.produce_split()

        rospy.logdebug("k: %d"%self.k)
        return TriggerResponse(success=succ, message= response)
        # return EmptyResponse()

    def produce_split(self):
        response = "OK."
        try:
            k, (train, test) = next(self.enumerate )
        except StopIteration:
            #rospy.loginfo("Done with current splits.")
            response = "Done with current splits."
            self.stop(reason = response)
            return False, response
        rospy.logdebug(f"train: {train}")
        rospy.logdebug(f"test: {train}")
        rospy.logdebug(f"k: {k}")
        if self.traintalker is not None:
            rospy.logdebug("stopping traintalker")
            self.traintalker.stop("getting next split")
        if self.testtalker is not None:
            rospy.logdebug("stopping testtalker")
            self.testtalker.stop("getting next split")

        self.k = k
        self.traintalker = dataset.CsvTalker(name="train", 
                loop_forever = False,
                data=self.data.take(train), stamped = self.stamped)
        self.traintalker.start()
        self.testtalker = dataset.CsvTalker(name="test", 
                loop_forever = False,
                data=self.data.take(test), stamped = self.stamped)
        self.testtalker.start()
        return True, response


if __name__ == '__main__':
    try:
        #rospy.init_node('csv_splitter', anonymous=True)
        rospy.init_node('csv_splitter', anonymous=True, log_level=rospy.DEBUG)

        n_splits = rospy.get_param("~n_splits", default=5)
        cv_type = rospy.get_param("~cv_type", default="RepeatedStratifiedKFold")
        n_repeats = rospy.get_param("~n_repeats", default=2)
        seed = rospy.get_param("~seed", default=22)
        use_random_splits = rospy.get_param("~use_random_splits", default=True)
        stamped = rospy.get_param("~stamped", default=True)
        dataset_path = rospy.get_param("~dataset_path", default="articles.csv")

        if use_random_splits:
            seed = np.random.randint(0, np.iinfo(np.uint32).max)
            rospy.loginfo("Using random seed: %d"%seed)
        else:
            rospy.loginfo("Using fixed seed: %d"%seed)

        myrate = rospy.Rate(1)
        with Splitter(  pd.read_csv(dataset_path),
                        n_splits=n_splits,
                        n_repeats=n_repeats,
                        cv_type=cv_type,
                        stamped=stamped,
                        seed=42) as asplitter:
            rospy.loginfo("Entering splitter loop.")
            while(True):
                u.nlogvvv("Splitter update loop.")
                if not asplitter.update():
                    break
                myrate.sleep()
            #rospy.spin()

    except rospy.ROSInterruptException:
        pass
