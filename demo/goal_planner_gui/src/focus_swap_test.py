#!/usr/bin/env python

import sys
import rospy
from planner_control_msgs.srv import focus_swap

if __name__ == "__main__":
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "ok.png"
    print("Simple test that replaces the focus swap icon with image: >%s<" % filename)
    print("sending >%s< to server" % filename)
    rospy.wait_for_service('focus_swap_signal')    
    try:
        control_service = rospy.ServiceProxy('focus_swap_signal', focus_swap)
        response = control_service(filename)
        if response.success:
            print("Signal %s sent successfully" % filename)
        else:
            print("Signal %s sent - server could not process it" % filename)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
