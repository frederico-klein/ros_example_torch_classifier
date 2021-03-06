#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosservice
import rospy

VVV = False

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

def read_params_with(name):
    sstr = name +"0"
    stops = []
    while (True):
        try:
            stops.append(rospy.get_param(sstr))
            sstr = name+ str(int(sstr.split(name)[-1])+1)
            nlogvvv(sstr)
        except KeyError:
            nlogdebug("No more parametro "+name)
            break
    nloginfo(stops)
    return stops


# this should probably be done with custom loggers.
def nlog(logstr, level="info"):
    if level == "debug":
        level_func = rospy.logdebug
    elif level == "info":
        level_func = rospy.loginfo
    elif level == "warn" or level == "warning":
        level_func = rospy.logwarn
    elif level == "error" or "err":
        level_func = rospy.logerr
    elif level == "fatal":
        level_func = rospy.logfatal
    level_func("node:<<{}>> {}".format(rospy.get_name(), logstr))
 
def check_remap(listo, level="debug"):
    for name in listo:
        nlog("the name that %s is resolving to: %s"%(name, rospy.resolve_name(name)))

def nlogdebug(logstr):
    nlog(logstr, level="debug")
def nloginfo(logstr):   
    nlog(logstr, level="info")
def nlogwarn(logstr):
    nlog(logstr, level="warn")
def nlogerr(logstr):
    nlog(logstr, level="err")
def nlogfatal(logstr):
    nlog(logstr, level="fatal")

def nlogvvv(logstr):
    if VVV:
        nlog("VVV: "+ logstr, level="debug")
    else:
        pass ## nothing will be logged if VVV not set for VVV messages.

def nlogvvvW(logstr):
    if VVV:
        nlog("VVV: "+ logstr, level="warning")
    else:
        pass ## nothing will be logged if VVV not set for VVV messages.


def strip_get_topics():
    pt = []
    for tpt in rospy.get_published_topics():
        pt.append(tpt[0])
    return set(pt)

def wait_for_topics(topic_list, rate=2, max_tries = 10000):
    ##we want every topic to exist, I reckon
    #also since this is no longer inside a huge instruction maybe we could make due without the python weird syntax. but it works, so no need to change it
    tries = 0
    myrate = rospy.Rate(rate)
    ts = set(topic_list)
    while(tries<max_tries):
        tries+=1
        gpts = strip_get_topics() 
        outset = ts - gpts 
        nlogdebug("set difference from topic_list and get_published_topics:\n\t{}".format(outset))
        if outset:
            nloginfo("Waiting for topics to be published:\n\t{}".format(outset))
            myrate.sleep()
        break
    else:
        nlogerr("max_tries ({}) reached".format(max_tries))

