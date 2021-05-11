#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy

def check_remap(listo, level="debug"):
    if level == "debug":
        level_func = rospy.logdebug
    if level == "info":
        level_func = rospy.loginfo
    if level == "warn" or level == "warning":
        level_func = rospy.logwarn
    if level == "error" or "err":
        level_func = rospy.logerr
    if level == "fatal":
        level_fatal = rospy.logfatal

    for name in listo:
        level_func("the name that %s is resolving to: %s"%(name, rospy.resolve_name(name)))


