#!/usr/bin/env python

import sys
import rospy   
import actionlib
import random
from database_msgs.srv import set_bool_attribute, set_string_attribute
from database_msgs.msg import object
from robot_interface_msgs.msg import *
from random import randint

supported_actions = {moveToAction, dropObjectAction, graspObjectAction, drinkSubjectAction, pourLiquidAction, openBottleAction, arrangeFlowerAction, pickFlowerAction, giveObjectAction}
SUCCESS_PROBABILITY = 1.0 # Probability that actions succeed
PROGRESS_FREQUENCY = 2 # progress frequency in Hz
PROGRESS_AMOUNT = 20 # percent progress per step 


class FakeActionServer(object):
    _feedback = None
    _result = None    
    action_dict = {drinkSubjectAction: "neurobots_demo/drink_subject",
                   dropObjectAction: "neurobots_demo/drop_object",
                   graspObjectAction: "neurobots_demo/grasp_object",
                   moveToAction: "neurobots_demo/move_to",
                   pourLiquidAction: "neurobots_demo/pour_liquid",
                   arrangeFlowerAction: "neurobots_demo/arrange_flower",
                   pickFlowerAction: "neurobots_demo/pick_flower",
                   openBottleAction: "neurobots_demo/open_bottle",
                   giveObjectAction: "neurobots_demo/give_object"}
    feedback_dict = {moveToAction: moveToFeedback,
                     dropObjectAction: dropObjectFeedback,
                     graspObjectAction: graspObjectFeedback,
                     drinkSubjectAction: drinkSubjectFeedback,
                     pourLiquidAction: pourLiquidFeedback,
                     arrangeFlowerAction: arrangeFlowerFeedback,
                     pickFlowerAction: pickFlowerFeedback,
                     openBottleAction: openBottleFeedback,
                     giveObjectAction: giveObjectFeedback}
    result_dict = {moveToAction: moveToResult,
                   dropObjectAction: dropObjectResult,
                   graspObjectAction: graspObjectResult,
                   drinkSubjectAction: drinkSubjectResult,
                   pourLiquidAction: pourLiquidResult,
                   arrangeFlowerAction: arrangeFlowerResult,
                   pickFlowerAction: pickFlowerResult,
                   openBottleAction: openBottleResult,
                   giveObjectAction: giveObjectResult}
    
    def __init__(self, action):
        self.action = action        
        self._feedback = self.feedback_dict[action]()
        self._result = self.result_dict[action]()
        self._action_name = self.action_dict[action]
        self._as = actionlib.SimpleActionServer(self._action_name, action, execute_cb=self.execute, auto_start=False)
        self._as.start()
    
    def execute(self, goal):
        print "executing action server for ", str(self.action)
        progress = 0
        r = rospy.Rate(PROGRESS_FREQUENCY)
        success = True
        success_rand = random.random()
        fail_at = 100
        if success_rand > SUCCESS_PROBABILITY:
#            print "rand= ", success_rand, " factor= ", success_rand/(1-SUCCESS_PROBABILITY)
            fail_at = int(100 * success_rand/(1-SUCCESS_PROBABILITY))
        print "Action will fail at ", fail_at
         
         
        while progress < 100:
            progress += PROGRESS_AMOUNT            
            if progress > fail_at:
                print "Aborted action because of simulated error"
                self._as.set_aborted(self._result, "SimulatedError")
                self.result_database_triggers(False)
                return
            if self._as.is_preempt_requested():                 
                 self._as.set_aborted(self._result, "Aborted")
                 print "Aborted action because of preemtion request"
                 return
            r.sleep()
            self._feedback.state = "Progress of action: %d%%" % progress
            self._as.publish_feedback(self._feedback)                        
            print "made progress %d%%" % progress        
        self.result_database_triggers(True)                
        self._as.set_succeeded(self._result)
           
    
    def result_database_triggers(self, success):
        print "Collecting database triggers..."
        object_strings_to_alter = set()
        object_bools_to_alter = set()
        goal = self._as.current_goal.get_goal()
        
        if self.action == moveToAction:
            self._result.arrived = success
            if success:
                if (goal.action_name=="approach"):
                    object_strings_to_alter.add((goal.robot, "at", goal.target_object))
                else:
                    #move to action, set new room of "robot" to "target"
                    object_strings_to_alter.add((goal.robot, "in", goal.target_object))
                    object_strings_to_alter.add((goal.robot, "at", "nowhere"))
                self._result.arrived = True
        elif self.action == dropObjectAction:
            self._result.placed = success
            if success:
                object_strings_to_alter.add((goal.current_object, "position", goal.target_location))
                object_bools_to_alter.add((goal.robot, "arm-empty", True))            
        elif self.action == graspObjectAction:
            self._result.grasped = success
            if success:
                object_strings_to_alter.add((goal.target_object, "position", goal.robot))
                object_bools_to_alter.add((goal.robot, "arm-empty", False))            
        elif self.action == drinkSubjectAction:
            self._result.arrived = success
            if success:
                object_strings_to_alter.add((goal.glass, "contains", "empty"))                
        elif self.action == pourLiquidAction:
            self._result.poured = success
            if success:
                object_strings_to_alter.add((goal.target_object, "contains", goal.swapped_content))
                object_strings_to_alter.add((goal.current_vessel, "contains", "empty"))
            
        elif self.action == arrangeFlowerAction:
            self._result.arranged = success
            if success:
                object_strings_to_alter.add((goal.flower, "position", goal.vase))
                object_strings_to_alter.add((goal.vase, "contains", goal.flower))
                object_bools_to_alter.add((goal.robot, "arm-empty", True))
            
        elif self.action == pickFlowerAction:
            self._result.picked = success
            if success:
                object_strings_to_alter.add((goal.flower, "position", goal.robot))
                object_strings_to_alter.add((goal.vase, "contains", "empty"))
                object_bools_to_alter.add((goal.robot, "arm-empty", False))                       
        elif self.action == openBottleAction:
            self._result.opened = success
            if success:
                object_bools_to_alter.add((goal.target_object, "is-open", True))         
        elif self.action == giveObjectAction:
            self._result.given = success
            if success:
                object_strings_to_alter.add((goal.target_object, "position", goal.human))
                object_bools_to_alter.add((goal.robot, "arm-empty", True))             
        else:
            print "Action not supported"
            assert False  
        print "altering %s strings and %s bools" % (len(object_strings_to_alter),len(object_bools_to_alter)) 
        for object in object_strings_to_alter:
            print "Altering object %s, attribute %s -> %s" % object
            rospy.wait_for_service('neurobots_database/debug/set_string')
            alter_service = rospy.ServiceProxy('neurobots_database/debug/set_string', set_string_attribute)
            response = alter_service(object[0], object[1], object[2], True)
        for object in object_bools_to_alter:
            print "Altering object %s, attribute %s -> %s" % object
            rospy.wait_for_service('neurobots_database/debug/set_bool')            
            alter_service = rospy.ServiceProxy('neurobots_database/debug/set_bool', set_bool_attribute)
            response = alter_service(object[0], object[1], object[2], True)

#                 rosobj = getobj_service(object[0])
#             except rospy.ServiceException, e:
#                 print "Object getter call failed: %s"%e
#             print rosobj
#             attribute_string = rosobj.object.attribute_values[rosobj.object.attribute_names.index(object[1])] if object[1] in rosobj.object.attribute_names else "NOT FOUND"
#             print "attribute vals to alter: ", attribute_string
#             att_list = attribute_string.split()
#             if isinstance(object[2], bool):
#                 att_list[-1] = int(object[2])
#             else:
#                 att_list[-1] = object[2]
#                 att_list[-2] = len(object[2])
#             attribute_string = " ".join(str(e) for e in att_list)
#             print "altered attribute      : ", attribute_string
#             rosobj.object.attribute_values[rosobj.object.attribute_names.index(object[1])] = attribute_string
# #                print "writing back to database"
#             rospy.wait_for_service('neurobots_database/add_object')
#             add_service = rospy.ServiceProxy('neurobots_database/add_object', add_object)
#             response = add_service(rosobj.object, object[0], True)
                         

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        try:
            SUCCESS_PROBABILITY=float(sys.argv[1])
        except ValueError:
            print "1st parameter given to fake executor was not a float, using default success probability of %s instead" % SUCCESS_PROBABILITY

    if len(sys.argv) >= 3:
        try:
            PROGRESS_FREQUENCY = int(sys.argv[2])
        except ValueError:
            print "2nd parameter given to fake executor was not an int, using default progress frequency of %s Hz instead" % SUCCESS_PROBABILITY           
        
    if len(sys.argv) >= 4:
        try:
            PROGRESS_AMOUNT = int(sys.argv[3])
        except ValueError:
            print "3rd parameter given to fake executor was not an int, using default progress amount of %s%% instead" % SUCCESS_PROBABILITY                    
              
    print "Starting fake action executor with %s success probability, increasing progress by %s%% every %s Hz" % (SUCCESS_PROBABILITY,PROGRESS_AMOUNT,PROGRESS_FREQUENCY)    
    print "Fake action executor processes the following actions: "
    for action in supported_actions:
        print str(action)
    try:
        rospy.init_node('fake_action_executor')
        servers = []
        for action in supported_actions:
            servers.append(FakeActionServer(action))
        rospy.spin()        
    except rospy.ROSInterruptException:
        print "fake action executor node interrupted before completion"
