#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, rosservice
import torch
import pandas as pd
import threading
from std_msgs.msg import String, Header
from ros_example_torch_classifier.msg import StringStamped
from std_srvs.srv import Empty, EmptyResponse
import re

def get_service_by_name(service_name):
    """Gets service complete name by name. Useful to debug namespace issues.

    @param service_name: type of service to find
    @type  service_name: str
    @return: list of services that have service_name as a part of their name
    @rtype: [str]
    """
    srv_list = []
    for srv in rosservice.get_service_list():
        if service_name in srv:
            srv_list.append(srv)
    return srv_list

class CsvTalker():
    def __init__(self, name= "",data= "data", loop_forever = True, stamped=False):
        self.name = name
        self.data = data
        self.stamped = stamped
        # assuming a 1ist row of labels!
        self.publist = []
        self.rate = rospy.Rate(10) # 10hz
        self.loop_forever = loop_forever
        self.thread = None
        self.enumerate = self.data.iterrows()
        if self.stamped:
            rospy.logwarn("defined stamped string for publishing")
            self.message_type = StringStamped
        else:
            rospy.logdebug("defined standard string for publishing")

            self.message_type = String

    def start(self):
        self.get_next = rospy.Service("~"+self.name+"/"+"atalk", Empty, self.asynchronous_talk)
        self.get_single = rospy.Service("~"+self.name+"/"+"clock", Empty, self.asynchronous_single)
        rospy.logdebug("List of atalk services: %s"%get_service_by_name("atalk"))
        rospy.logdebug("List of clock services: %s"%get_service_by_name("clock"))
        for acol in self.data.columns:
            self.publist.append(rospy.Publisher(self.name+"/"+re.sub("[: ]","_",acol), self.message_type, queue_size=10, latch=True))

        rospy.loginfo("CsvTalker loaded OK.")

    def stop(self, reason = "No reason given."):
        # deregistering services
        self.get_next.shutdown(reason)
        self.get_single.shutdown("see reason above")
        for pubb in self.publist:
            pubb.unregister()
        rospy.loginfo("CsvTalker unloaded OK.")

    def __enter__(self):
        self.start()
        return self

    def  __exit__(self, *exc):
        # deregistering services
        reason = "\n\texc list: {}\n".format(*exc)
        self.stop(reason)

    def say_single_row(self,row):
        if self.stamped:
            h = Header()
            h.stamp = rospy.Time.now() ##making it easier for TimeSynchronizer by using exactly the same time.
            for acol, apublisher in zip(self.data.columns, self.publist ):
                my_str = row[acol]
                #rospy.loginfo(my_str)
                msg = self.message_type
                msg.header = h
                msg.data = str(my_str)
                apublisher.publish(msg)
        else:
            for acol, apublisher in zip(self.data.columns, self.publist ):
                my_str = row[acol]
                #rospy.loginfo(my_str)
                apublisher.publish(str(my_str)) ## making sure it is a string



    def asynchronous_single(self, req):
        rospy.logdebug("called asynchronous_single .")
        next(self.asingle())
        rospy.logdebug("ended asynchronous_single .")
        return EmptyResponse()

    def asingle(self):
        rospy.loginfo("starting dataset iteration .")
        for index, row in self.enumerate:
            yield self.say_single_row(row)

        rospy.loginfo("dataset finished.")

    def asynchronous_talk(self, req):
        self.atalk()
        return EmptyResponse()

    def atalk(self):
        # non-blocking version of talk
        self.thread = threading.Thread(target=self.talk)
        self.thread.start()


    def talk(self):
        while not rospy.is_shutdown():
            for index, row in self.data.iterrows():
                self.say_single_row(row)
                self.rate.sleep()
            rospy.loginfo("dataset finished.")
            if self.loop_forever:
                rospy.loginfo("restarting from beginning.")
            else:
                break

if __name__ == '__main__':
    try:
        rospy.init_node('csv_talker', anonymous=True)
        with CsvTalker(data=pd.read_csv("articles.csv"), name="data") as atalker:
            atalker.talk()
    except rospy.ROSInterruptException:
        pass
