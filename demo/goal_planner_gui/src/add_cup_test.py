#!/usr/bin/env python

import sys
import rospy
from database_msgs.srv import add_object, get_object, remove_object
from database_msgs.msg import object

if __name__ == "__main__":
    ADD_MODE = True
    objname = "newcup"
    if len(sys.argv) > 1:
        if sys.argv[1] in ["-d","-r","--remove","--delete","-rm"]:
            print "Deleting instead"
            ADD_MODE = False
            if len(sys.argv) > 2:
                objname = sys.argv[2]
        else:
            objname = sys.argv[1]
    
#     Comment in the following block to see how to fetch an object    
#     print "Waiting for service"    
#     rospy.wait_for_service('neurobots_database/get_object')
#     try:
#         getobj_service = rospy.ServiceProxy('neurobots_database/get_object', get_object)
#         g2_obj = getobj_service('g1')
#     except rospy.ServiceException, e:
#         print "Object getter call failed: %s"%e
#     print "found glass g1:"
#     print g2_obj
#     
    
    print("Simple test that adds/deletes an empty cup object to the database; name: >%s<" % objname)    
    if ADD_MODE:        
        rospy.wait_for_service('neurobots_database/add_object')
    else:
        rospy.wait_for_service('neurobots_database/remove_object')
        
    try:
        if ADD_MODE:
            print("ADDING object >%s< to database " % objname)
            control_service = rospy.ServiceProxy('neurobots_database/add_object', add_object)
            object_message = object()
            object_message.attribute_names = ['contains', 'position', 'is-open', 'perceptible', 'locatable', 'type', 'name']
            object_message.attribute_values = ['22 serialization::archive 10 0 0 1 0 0 0 3 5 empty', '22 serialization::archive 10 0 0 1 0 0 0 3 5 shelf', '22 serialization::archive 10 0 0 1 0 0 0 2 1', '22 serialization::archive 10 0 0 1 0 0 0 2 0', '22 serialization::archive 10 0 0 1 0 0 0 2 0', '22 serialization::archive 10 0 0 1 0 0 0 3 3 cup', '22 serialization::archive 10 0 0 1 0 0 0 3 %d %s'% (len(objname), objname)]       
            print "sending message: ", object_message
            response = control_service(object_message, objname, True)
        else:
            print("REMOVING object >%s< to database " % objname)
            control_service = rospy.ServiceProxy('neurobots_database/remove_object', remove_object)
            response = control_service(objname)
        if response.success:
            print("Signal %s sent successfully" % objname)
        else:
            print("Signal %s sent - server could not process it" % objname)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e