import os
import time
import rospy
from datetime import datetime
from planner_control_msgs.srv import menu_control

ROS_EXPERIMENT_LOG_FOLDER = 'experiment_log/'



class ExperimentLogger():
    def __init__(self, auto = False):
        print "Creating Experiment Loggers with auto = %s" %auto        
        self.navigation_logfilename, self.action_logfilename, self.performance_logfilename = self.create_logfiles()         
        self.last_time = None
        self.autoperform = auto
    
    def create_logfiles(self):
         directory = os.path.dirname(ROS_EXPERIMENT_LOG_FOLDER)
         if not os.path.exists(directory):
             os.makedirs(directory)
         current_time_string = time.strftime("%y_%m_%d_%H_%M_%S", time.localtime())
         navigationfilepath = os.path.join(directory, current_time_string + "_nav.log")
         actionfilepath = os.path.join(directory, current_time_string + "_act.log")
         performancefilepath = os.path.join(directory, "performance.log")
         print ("performance file = %s" % os.path.abspath(performancefilepath))         
         return (navigationfilepath, actionfilepath, performancefilepath)

    def log_action_start(self, name, arguments):
        text = "%s starting %s - %s\n" % (time.strftime("%H:%M:%S ", time.localtime()), name, arguments)
        logf = open(self.action_logfilename, "a+")
        logf.write(text)
        logf.close()

    def log_action_end(self, name, success):
        text = "%s finished %s - %s\n" % (time.strftime("%H:%M:%S ", time.localtime()), name, success)
        logf = open(self.action_logfilename, "a+")
        logf.write(text)
        logf.close()

    def log_navigation(self, index, arguments):
        text = "%s selected #%s: %s\n" % (time.strftime("%H:%M:%S ", time.localtime()), index, arguments)
        logf = open(self.navigation_logfilename, "a+")
        logf.write(text)
        logf.close()

    def log_performance(self, logstring, execute=False):
        if self.autoperform:        
            print("Log performance debug string : %s"% logstring)
            if self.last_time:
                difference = (datetime.now() - self.last_time).total_seconds()
                self.last_time = None
                print ("time difference to now = %fs" % difference)
                logf = open(self.performance_logfilename, "a+")
#                logf.write(", %s:, %s" % (logstring, difference))
                logf.write(", %s" % (difference))
                logf.close()
                if execute:
                    print("Performance experiment is executing a teleop select command")
                    rospy.wait_for_service('menu_control_signal')    
                    try:
                        control_service = rospy.ServiceProxy('menu_control_signal', menu_control)                    
                        response = control_service(3)
                        print ("sent keysignal success? %s " % response.success)                    
                    except rospy.ServiceException, e:
                        print("Service call failed: %s"%e)
                else:
                    print("do not execute because execute = False")
            else:
#                logf = open(self.performance_logfilename, "a+")
#                logf.write(", %s->" % (logstring))
#                logf.close()
                print ("restart time counter %s" % time.strftime("%H:%M:%S ", time.localtime()))
                self.last_time = datetime.now()                
        print("logging done")