#!/usr/bin/env python

import os
import rospy, rospkg, roslaunch
import subprocess

ROS_EXPERIMENT_LOG_FOLDER = 'experiment_log/'

if __name__ == "__main__":
    logfile = os.path.join(os.path.dirname(ROS_EXPERIMENT_LOG_FOLDER), "performance.log")
    for worldobj in [10,20]:
        for repetitions in range(2):
            logf = open(logfile, "a+")
            logf.write("\nNextExperiment, %d" % worldobj)
            logf.close()
            subprocess.call(["roslaunch","aaai18_experiments", "performance_scenario.launch", "myargs:=%d"%worldobj])            
            print ("starting next experiment")