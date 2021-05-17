#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, rosservice
import torch
import pandas as pd
import threading
from std_msgs.msg import String, Header
from ros_example_torch_classifier.msg import StringStamped
from std_srvs.srv import Empty, EmptyResponse
from ros_example_torch_classifier.srv import Dump, DumpResponse
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
        rospy.logwarn("using only 100 items from head!\n this was added for testing remove or fix for proper testing")
        self.data = data.head(100)
        self._finished = False
        self.stamped = stamped
        # assuming a 1ist row of labels!
        self.publist = []
        self.dumplist = []
        ## so this is really strange. I was hitting a rate limit here with 10hz. 
        ## TODO: now this 1hz is witchcraft. it possibly need to be a send receive model. 
        self.rate = rospy.Rate(1) # 1hz
        self.loop_forever = loop_forever
        self.thread = None
        self.enumerate = self.data.iterrows()
        if self.stamped:
            rospy.logwarn("defined stamped string for publishing")
            self.message_type = StringStamped
        else:
            rospy.logdebug("defined standard string for publishing")

            self.message_type = String

    @property
    def finished(self):
        return self._finished

    @finished.setter
    def finished(self,value):
        ##I want this to be a param
        rospy.set_param('~%s/finished'%self.name, value)
        self._finished = value

    def df(self, acol,req):
        fdata = self.data[acol].drop_duplicates() ##maybe we also want to remove NaNs here?
        rospy.loginfo(acol)
        rospy.logwarn(self.data["abstract"])
        rospy.logwarn("dump service called")
        rospy.logwarn(fdata)
        odata = " ".join(fdata)
        return DumpResponse(data=odata)

    def start(self):
        self.finished = False # will set the param
        self.get_next = rospy.Service("~"+self.name+"/"+"atalk", Empty, self.asynchronous_talk)
        self.get_single = rospy.Service("~"+self.name+"/"+"clock", Empty, self.asynchronous_single)
        rospy.logdebug("List of atalk services: %s"%get_service_by_name("atalk"))
        rospy.logdebug("List of clock services: %s"%get_service_by_name("clock"))
        for acol in self.data.columns:
            pacol = re.sub("[: ]","_",acol)
            ## for this architecture missing messages is extremely bad, so I will set up a very long queue. it should be fine to do even larger memory wise, since we have text.
            self.publist.append(rospy.Publisher(self.name+"/"+ pacol, self.message_type, queue_size=20, latch=True))
            df = lambda x: self.df(acol,x)
            self.dumplist.append(rospy.Service("~"+self.name+"/"+pacol+"/"+"dump", Dump, df))

        rospy.loginfo("CsvTalker loaded OK.")

    def stop(self, reason = "No reason given."):
        # deregistering services
        self.get_next.shutdown(reason)
        self.get_single.shutdown(reason)
        for pubb in self.publist:
            pubb.unregister()
        for dumps in self.dumplist:
            dumps.shutdown(reason)
        rospy.loginfo("CsvTalker unloaded OK.")

    def __enter__(self):
        self.start()
        return self

    def  __exit__(self, *exc):
        # deregistering services
        reason = "\n\texc list: {}\n".format(*exc)
        self.stop(reason)

    def say_single_row(self,row):
        rospy.logdebug("======")
        rospy.logdebug("called say single row {}".format(row))
        rospy.logdebug("======")
        if self.stamped:
            rospy.logdebug("stamped publishing")
            h = Header()
            h.stamp = rospy.Time.now() ##making it easier for TimeSynchronizer by using exactly the same time.
            for acol, apublisher in zip(self.data.columns, self.publist ):
                if apublisher.get_num_connections() >0: ##avoids publishing if no one is listening
                    my_str = row[acol]
                    #rospy.loginfo(my_str)
                    msg = self.message_type()
                    msg.header = h
                    msg.data = str(my_str)
                    apublisher.publish(msg)
                    ## too verbose
                    #rospy.logdebug("should have published topic %s"%acol)
                    rospy.logdebug("should have published message %s"%(str(msg)))
 
        else:
            rospy.logdebug("unstamped version")
            for acol, apublisher in zip(self.data.columns, self.publist ):
                my_str = row[acol]
                #rospy.loginfo(my_str)

                apublisher.publish(str(my_str)) ## making sure it is a string
                rospy.logdebug("should have published topic %s"%acol)


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
        rospy.loginfo("atalk: starting threaded version of talk")
        self.thread = threading.Thread(target=self.talk)
        self.thread.start()


    def talk(self):
        rospy.loginfo("talk called.")
        while not rospy.is_shutdown():
            for index, row in self.data.iterrows():
                rospy.logdebug("calling say single row")
                self.say_single_row(row)
                rospy.logdebug("sleeping")
                self.rate.sleep()
            rospy.loginfo("dataset finished.")
            self.finished = True
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
