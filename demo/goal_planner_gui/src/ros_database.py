import rospy
import cStringIO

from pddllib import pddl
import actionlib
from PyQt5 import QtCore, QtGui

import ros_image_provider

from robot_interface_msgs.msg import *
from planner_control_msgs.srv import *
from database_msgs.srv import *
from neurobots_world import *

from cplan import plans
# from docutils.nodes import Element
from cgi import logfile

import logging
logger = logging.getLogger('root')
logger2 = logging.getLogger('highlighted')

ROS_ACTION_TIMEOUT = 10  # time in seconds to wait for ROS action server


def boost_variant_extract(bvvec):  # very hacky abuse of the raw data: if the message containing a boost vector has more than 12 elements, it is a vector. In a vector, entries have 2 elements with information for boost, and one element with the actual name of the element in the vector, so we just use every 3rd element, starting from position 11, where the vector begins
    split_list = bvvec.split()
    if len(split_list) > 12:
#        print "vector chopped list = ", split_list[11::3]
        return split_list[11::3]
#    print split_list[-1]
    return split_list[-1]  # the last element is either a string or a number {0,1} which corresponds to the truth value of a predicate. As python uses 1 and 0 as True and False, this hacky extraction works for us

def create_dict_from_object(ros_object):  # creates a dictonary from an object containting its attributes
    # e.g. {name : omnirob, location: kitchen}
#     print("ROSOBJECT = ",ros_object)
    attribute_list = [boost_variant_extract(element) for element in ros_object.attribute_values]
    object_dict = dict(zip(ros_object.attribute_names, attribute_list))
#     print "created object dictionary."
#     for key in object_dict:
#         print "%s --> %s" % (key, object_dict[key])
    return object_dict

def dict_changes(dict_origin, dict_update):
    result = dict()
    for key in dict_update:
        if key in ["perceptible", "locatable"]:
            continue
        unmasked = dict_update[key]
        try:
            if isinstance(unmasked, basestring):
                unmasked = int(unmasked)  # convert '1' in 1 if possible
        except ValueError:
            pass
#         logger.debug("origin = %s, unmask= %s, result = %s" % (dict_origin[key], unmasked, (dict_origin[key] != unmasked)))
        if dict_origin.get(key) != unmasked:
            result[key] = unmasked
    return result

class RosDatabaseHandler():
    def __init__(self, widget, args, neurodict):  # The widget is used to display action feedback in the GUI statusbar
        self.signal_dict = {1:"up", 2:"down", 3:"select", 4:"abort", 5:"abort/down-contextsensitive"}
        self.widget = widget
        self.references = args.references
        self.callback_done = False
        self.current_action = None
        self.action_clients = {}
        self.event_queue = []
        self.world = NeurobotsWorld(neurodict)
        self.ros_image_provider = ros_image_provider.RosImageProvider(args)

#         self.action_dict = {drinkSubjectAction: "neurobots_demo/drink_subject",
#                dropObjectAction: "neurobots_demo/drop_object",
#                graspObjectAction: "neurobots_demo/grasp_object",
#                moveToAction: "neurobots_demo/move_to",
#                pourLiquidAction: "neurobots_demo/pour_liquid",
#                arrangeFlowerAction: "neurobots_demo/arrange_flower",
#                pickFlowerAction: "neurobots_demo/pick_flower",
#                openBottleAction: "neurobots_demo/open_bottle",
#                giveObjectAction: "neurobots_demo/give_object"}
#         self.result_bool_dict = {moveToAction: moveToResult.arrived,
#                dropObjectAction: dropObjectResult.placed,
#                graspObjectAction: graspObjectResult.grasped,
#                drinkSubjectAction: drinkSubjectResult.arrived,
#                pourLiquidAction: pourLiquidResult.poured,
#                arrangeFlowerAction: arrangeFlowerResult.arranged,
#                pickFlowerAction: pickFlowerResult.picked,
#                openBottleAction: openBottleResult.opened,
#                giveObjectAction: giveObjectResult.given}
        self.action_clients = {
                drinkSubjectAction: actionlib.SimpleActionClient("neurobots_demo/drink_subject", drinkSubjectAction),
                dropObjectAction: actionlib.SimpleActionClient("neurobots_demo/drop_object", dropObjectAction),
                graspObjectAction: actionlib.SimpleActionClient("neurobots_demo/grasp_object", graspObjectAction),
                moveToAction: actionlib.SimpleActionClient("neurobots_demo/move_to", moveToAction),
                pourLiquidAction: actionlib.SimpleActionClient("neurobots_demo/pour_liquid", pourLiquidAction),
                arrangeFlowerAction: actionlib.SimpleActionClient("neurobots_demo/arrange_flower", arrangeFlowerAction),
                pickFlowerAction: actionlib.SimpleActionClient("neurobots_demo/pick_flower", pickFlowerAction),
                openBottleAction: actionlib.SimpleActionClient("neurobots_demo/open_bottle", openBottleAction),
                giveObjectAction: actionlib.SimpleActionClient("neurobots_demo/give_object", giveObjectAction)
            }
        
        for k, v in self.action_clients.iteritems():
            try:
                if not v.wait_for_server(rospy.rostime.Duration(ROS_ACTION_TIMEOUT)):
                    raise Exception("Server Exceeded Timeout %d" % ROS_ACTION_TIMEOUT)
                logger.debug("Action server found %s", k.__name__)
            except Exception as e:
                logger.error("Timeout finding action server for %s, topic = %s", k.__name__, v.action_client.ns)
                logger.critical(e)
                exit(20)
#                 self.event_queue.append((self.widget.back, []))

        self.outstanding_changes = set()  # once the GUI executes an action, we store the effects in this queue and wait for the Neurobots database to trigger these changes
        # changes are a triple of strings (object, changed_attribute, new_value)
        self.widget.add_image_provider(self.ros_image_provider)
        #self.widget.experiment_logger.log_performance("ROS Database Handler created")

    def __del__(self):
        self.action_clients = None

    def feedbackcaller(self, message):
#         if not self.current_action_client:  # action was aborted
#             return
        logger.debug("Enqueing Feedback: %s" % message.state)
        self.event_queue.append((self.widget.display_status_text, ["Action Status: %s" % message.state]))

    def donecaller(self, status, result):
        #logger2.debug(type(result))
        success = True
        if ("False" in str(result)):  # HACK: do not want to go over all the members arrived/poured/opened/placed...
            self.outstanding_changes = set()
            success = False
        self.widget.current_action_succeeded = success
        self.widget.experiment_logger.log_action_end(self.current_action, result)
#         self.current_action = None
#         self.current_action_client = None
        self.callback_done = True
        self.action_done_database_updated()

    def action_done_database_updated(self):  # this is called twice, once by the "donecaller" wich is triggered from the successful execution of the ROS-action on the robot, and once from the database, after an expected world change is processed
        if self.callback_done and (not self.outstanding_changes):
            self.event_queue.append((self.widget.action_executed, []))
            self.current_action = None
        else:
            logger.debug("Action is not properly executed yet. Reason:")
            if self.outstanding_changes:
                logger.debug("outstanding changes:")
                logger.debug(self.outstanding_changes)
            else:
                logger.debug("action did not call the ROS succeeded message yet")
    def add_change(self, object, argument, value):
        worldobj = self.world.get(object).attributes
        logger.debug("add change to %s", worldobj)
        logger.debug("current value of %s is %s -> %s" % (argument, worldobj.get(argument), value))
        if self.world.get(object).attributes.get(argument) != value:
            self.outstanding_changes.add((object, argument, value))
        else:
            logger.debug("no change needed, value already correct")

    def execute_on_robot(self, action, args):
        self.current_action = action.name
        arglist = [str(arg) for arg in args]
        logger2.debug("Executing action %s with args %s" % (self.current_action, arglist))
        self.widget.experiment_logger.log_action_start(self.current_action, arglist)
        rosaction = None
        rosgoal = None
        if self.current_action in ["approach", "go", "move"]:
            rosaction = moveToAction
            name = "move_to" if (self.current_action in ["go", "move"]) else "approach"
            if name == "move_to":
                self.add_change(str(args[0]), "in", str(args[2]))
                if self.current_action == "move":
                    self.add_change(str(args[0]), "at", str(args[1]))
                else:
                    self.add_change(str(args[0]), "at", "nowhere")
                rosgoal = moveToGoal(robot=str(args[0]), target_object=str(args[2]), action_name=name)
            else:
                # self.add_change(str(args[0]), "at", "nowhere")
                logger.debug(len(self.outstanding_changes))
                rosgoal = moveToGoal(robot=str(args[0]), target_object=str(args[2]), action_name=name)
                self.add_change(str(args[0]), "at", str(args[2]))
        elif self.current_action == "grasp":
            rosaction = graspObjectAction
            rosgoal = graspObjectGoal(robot=str(args[0]), target_object=str(args[1]))
            self.add_change(str(args[1]), "position", str(args[0]))
            self.add_change(str(args[0]), "arm-empty", False)
        elif self.current_action == "pick":
            rosaction = pickFlowerAction
            rosgoal = pickFlowerGoal(robot=str(args[0]), flower=str(args[1]), vase=str(args[2]))
            self.add_change(str(args[1]), "position", str(args[0]))
            self.add_change(str(args[2]), "contains", "empty")
            self.add_change(str(args[0]), "arm-empty", False)
        elif self.current_action == "open":
#            print "Opening of bottles is not implemented yet"
            rosaction = openBottleAction
            rosgoal = openBottleGoal(fixing_robot=str(args[0]), turning_robot=str(args[0]), target_object=str(args[1]))
            self.add_change(str(args[1]), "is-open", True)
        elif self.current_action == "drop":
            rosaction = dropObjectAction
            rosgoal = dropObjectGoal(robot=str(args[0]), current_object=str(args[1]), target_location=str(args[2]))
            self.add_change(str(args[1]), "position", str(args[2]))
            self.add_change(str(args[0]), "arm-empty", True)
        elif self.current_action == "arrange":
            rosaction = arrangeFlowerAction
            rosgoal = arrangeFlowerGoal(robot=str(args[0]), flower=str(args[1]), vase=str(args[2]))
            self.add_change(str(args[1]), "position", str(args[2]))
            self.add_change(str(args[0]), "arm-empty", True)
            self.add_change(str(args[2]), "contains", str(args[1]))
        elif self.current_action == "give":
            rosaction = giveObjectAction
            rosgoal = giveObjectGoal(robot=str(args[0]), target_object=str(args[1]), human=str(args[2]))
            self.add_change(str(args[0]), "arm-empty", True)
            self.add_change(str(args[1]), "position", str(args[2]))
        elif self.current_action == "drink":
            rosaction = drinkSubjectAction
            rosgoal = drinkSubjectGoal(robot=str(args[0]), glass=str(args[1]), target_subject=str(args[2]))
            self.add_change(str(args[1]), "contains", "empty")
        elif self.current_action == "pour":
            rosaction = pourLiquidAction
            rosgoal = pourLiquidGoal(robot=str(args[0]), current_vessel=str(args[1]), target_object=str(args[2]), swapped_content=str(args[4]))
            self.add_change(str(args[1]), "contains", "empty")
            self.add_change(str(args[2]), "contains", str(args[4]))
        else:
            logger.error("There is no such action with self.current_action %s" % self.current_action)
            assert False
            
        assert rosaction
        assert rosgoal
        logger.debug("Executing %s with the following goal:\n%s" % (rosaction.__name__, rosgoal))
        self.callback_done = False
        
        try:
            self.action_clients[rosaction].send_goal(rosgoal, done_cb=self.donecaller, feedback_cb=self.feedbackcaller)
            self.current_action_client = self.action_clients[rosaction]
        except Exception as e:
            self.event_queue.append((self.widget.back, []))
            
#         self.current_action_client = actionlib.SimpleActionClient(self.action_dict[rosaction], rosaction)
#         self.callback_done = False
#         try:
#             if not self.current_action_client.wait_for_server(rospy.rostime.Duration(ROS_ACTION_TIMEOUT)):
#                 raise Exception("Server Exceeded Timeout %d" % ROS_ACTION_TIMEOUT)
#             logger.debug("Action server found")
#             self.current_action_client.send_goal(rosgoal, done_cb=self.donecaller, feedback_cb=self.feedbackcaller)
#             # send goal asap, the GUI is blocked as long as the execution handling has not terminated
#         except Exception as e:
#             logger.error("Timeout finding action server for %s, topic = %s" % (rosaction.__name__, self.action_dict[rosaction]))
#             logger.error(e)
#             self.event_queue.append((self.widget.back, []))
        logger.debug("execution send to robot via ROS")

    def abort_current_action(self):
        logger.debug("Cancelling action")
        self.outstanding_changes = set()
        if self.current_action_client:
            self.current_action_client.cancel_all_goals()

    def ros_menu_control(self, control_request):
        logger.debug("Control Signal received: >%s< (%s)" % (control_request.direction, self.signal_dict[control_request.direction]))
        pl_ui = self.widget.view.rootObject().findChild(QtCore.QObject, "planner_ui")
        go_ui = self.widget.view.rootObject().findChild(QtCore.QObject, "goal_ui")
        active_ui = go_ui
        if self.widget.mode == 1:  # MENU_MODE_PLAN
            active_ui = pl_ui

        if control_request.direction == 1:
            self.event_queue.append((active_ui.teleop_up, []))
#            active_ui.teleop_up() # funktionniert leider nicht, warum auch immer
#            QtCore.QMetaObject.invokeMethod(active_ui, "teleop_up")
        elif control_request.direction == 2:
            self.event_queue.append((active_ui.teleop_down, []))
#            QtCore.QMetaObject.invokeMethod(active_ui, "teleop_down")
        elif control_request.direction == 3:
            self.event_queue.append((active_ui.teleop_select, []))
#            QtCore.QMetaObject.invokeMethod(active_ui, "teleop_select")
        elif control_request.direction == 4:
            self.event_queue.append((active_ui.teleop_back, []))
#            QtCore.QMetaObject.invokeMethod(active_ui, "teleop_back")
        elif control_request.direction == 5:
            if planning_mode:
                self.event_queue.append((active_ui.teleop_back, []))
#                QtCore.QMetaObject.invokeMethod(pl_ui, "teleop_back")
            else:
                self.event_queue.append((active_ui.teleop_down, []))
#                QtCore.QMetaObject.invokeMethod(go_ui, "teleop_down")
        else:
            return menu_controlResponse(False)
        return menu_controlResponse(True)

    def ros_get_menu_navigation(self, request):
        logger.debug("#############\nThe current mode is " + self.widget.mode)
        planningmode = (self.widget.mode == 1)  # 1 = MENU_MODE_PLAN
        logger.debug("PlanningMode" if planningmode else "MenuMode")
#        for key in self.widget.__dict__:
#            print "%s -> %s" % (key, self.widget.__dict__[key])
#        print "The length of the list is", len(self.widget.elems)
        currentindex = self.widget.current_menu_index
#        print "current index is ", currentindex
        if planningmode:
            logger.debug ("item %d / %d" % (currentindex, len(self.widget.current_plan)))
            logger.debug ("planitemstatus = " + self.widget.current_plan[currentindex])
            for key in self.widget.current_plan[currentindex].__dict__:
                logger.debug("%s -> %s" % (key, self.widget.current_plan[currentindex].__dict__[key]))

        right = True
        if planningmode:
            up = False
            down = False
            left = currentindex >= len(self.widget.current_plan) - 1
            if left and self.widget.current_plan[currentindex].status == plans.ActionStatusEnum.EXECUTABLE:
                right = False
        else :
            up = True  # currentindex > 0
            down = True  # currentindex + 1 < len(self.widget.elems)
            left = len(self.widget.menu_stack) > 0
        logger.debug("up: %s, down: %s, left: %s, right: %s" % (up, down, left, right))
        # return dummy value for now
        return get_menu_navigation_directionsResponse(up, down, left, right)

    def ros_focus_swap(self, focus_image_message):
        logger.debug("New Focus Image received: %s" % focus_image_message.filename)
        focus_image_filename = "images/" + focus_image_message.filename
        focus_item = self.widget.view.rootObject().findChild(QtCore.QObject, "focuspoint")
        logger.debug("setting new image to " + focus_image_filename)
        self.event_queue.append((focus_item.set_image, [focus_image_filename]))
#        QtCore.QMetaObject.invokeMethod(focus_item, "set_image", QtCore.Qt.DirectConnection, QtCore.Q_ARG(QtCore.QVariant, focus_image_filename))
        return focus_swapResponse(True)  # TODO: check whether the image path exists and return False if not

    def database_change_update(self, change):
        irrelevant_attributes = ["pose", "perceptible", "locatable"]
        logger.debug("The following changes appeared in the world")
        changes_were_expected = False
        for element in change.changed_objects:
            logger.debug("%s - fetching updates..." % element)
            try:
                srv = rospy.ServiceProxy('neurobots_database/get_object', get_object)
                object_response = srv(element)
                if object_response.found:
                    obj_dict = create_dict_from_object(object_response.object)
                    logger.debug("found the following new informations on %s:" % element)
                    obj_name = str(obj_dict["name"])
                    assert str(element) == obj_name
                    logger.debug("object dict = %s", obj_dict)
                    if self.world.contains(obj_name):
                        logger.debug("current state of object:")
                        logger.debug(self.world.get(obj_dict["name"]))
                        change_dict = dict_changes(self.world.get(obj_dict["name"]).attributes, obj_dict)
                        logger.debug("change dict = %s", change_dict)
                        changes = set((obj_name, key, change_dict[key]) for key in change_dict.keys() if key not in irrelevant_attributes)
                        logger.debug("The following changes appeared: %s", changes)
                        logger.debug("Expected changes for %s: %s" % (obj_name, self.outstanding_changes))
                        self.world.update(changes)
                        if len(changes - self.outstanding_changes) > 0:
                            logger.debug("There were unexpected world changes -> go back to main %s", (changes - self.outstanding_changes))
                            changes_were_expected = False
                            continue
                        else:
                            self.outstanding_changes = self.outstanding_changes - changes
                            if not self.outstanding_changes:
                                logger.debug("all outstanding changes performed, calling action_done_database_updated()")
                                self.action_done_database_updated()
                            else:
                                logger.debug("still waiting for %s", self.outstanding_changes)
                            logger.debug("change was expected")
                            changes_were_expected = True
                            continue
                        # check for expected changes
                    else:
                        logger.debug("object was not present before -> it was added to the database")
                        changes_were_expected = False
                else:
                    logger.debug("Object %s changed but not present in database" % element)
                    logger.debug("If the object was removed, it should have been there before:")
                    logger.debug(self.world.get(str(element)))
                    changes_were_expected = False
                    # # we could remove the object from the world, but as the world is reset anyways to build the context we abstain
            except rospy.ServiceException, e:
                logger.error("Service call failed: %s" % e)
        if changes_were_expected:
            logger.debug("all world changes were expected -> return without update")
            return
        logger.debug("Resetting the goal context now")
        (newproblem, newobjects) = get_problem_from_database(self.widget.context.problem.domain)
        self.world = NeurobotsWorld(newobjects)
        import reference_list
        newrefs = reference_list.ReferenceList(newproblem, self.references)
        newrefs.create_atomic_partitions()
        newrefs.create_extended_partitions()
        newrefs.create_optimistic_partitions()
        logger.debug("refreshing context")
        self.widget.refresh_context(newproblem, newrefs)
        logger.debug("currently the widget has the following elements %s", self.widget.elems)
        logger.debug("currently the plan is %s", self.widget.current_plan)
        logger.debug("currently the goal is %s", self.widget.current_goal)
        logger.debug("jumping back to root menu")
        self.event_queue.append((self.widget.handle_world_update, []))
        self.widget.ignore_move_events = False
#        QtCore.QMetaObject.invokeMethod(self.widget.view.rootObject(), "handle_world_update")



#         self.widget.init_menu()
#         print "show goal ui"
#         self.widget.show_plan_ui()
#         self.widget.show_goal_ui()
# #        print "reloading UI"
# #        self.widget.reload_ui()
#         print "elems are now", self.widget.elems


def get_problem_from_database(domain):
    has_rooms = False
    if domain.name in ["neurobots-demo", "neurobots-iros2", "neurobots-simgen", "neurobots-small"]:
        has_rooms = True
        logger.debug("Domain has rooms %s" % domain.name)
    else:
        logger.debug("Domain has no rooms %s" % domain.name)
    logger.debug("Trying to fetch problem file from Neurobots Database")
    rospy.wait_for_service('neurobots_database/get_world')
    try:
        srv = rospy.ServiceProxy('neurobots_database/get_world', get_world)
        world_response = srv()
        world = world_response.world
        logger.debug("fetched world " + world.domain + " with " + str(len(world.objects)) + " objects")
        # for objn in world.objects:
            # print "obj: ",objn
    except rospy.ServiceException, e:
        logger.error("Service call failed: %s" % e)

    problem_objects = set()
    init_strings = set()
    neurobots_objects = []
    logger.debug("Domain types are: %s", map(str, domain.types))
    
    def determine_subertypes(type):
        alltypes = set()
        t = type
        alltypes.add(t)
        tobj = domain.types.get(t, None)
        if tobj:
            supertypes = tobj.get_supertypes()
            for s in tobj.get_supertypes():
                alltypes.add(str(s))
#         print "all supertypes of %s are %s" % (type, alltypes) 
        return alltypes
    
    for wobj in world.objects:
        obj_dict = create_dict_from_object(wobj)
        logger.debug(obj_dict)
        stripped_dict = dict()
        assert "name" in obj_dict, "object has no 'name' %s" % obj_dict
        name = obj_dict["name"]
        stripped_dict["name"] = obj_dict["name"]
        assert "type" in obj_dict, "object %s has no 'type'" % name
        type = obj_dict["type"]
        stripped_dict["type"] = obj_dict["type"]
        logger.debug("DEBUG: detectet object %s of type %s " % (name, type))
        supertypes = determine_subertypes(type)
        # logger.debug("supertypes of type %s: %s", type, supertypes)
        
        if (type != "poses"):
            problem_objects.add(pddl.TypedObject(name, domain.types[type]))
            if any(True for x in supertypes if x in ["transportable", "vessel"]):
                assert "position" in obj_dict, "object %s has no 'position'" % name
                init_strings.add("(= (position %s) %s)" % (name, obj_dict["position"]))
                stripped_dict["position"] = obj_dict["position"]
                if "vessel" in supertypes:
                    if not "contains" in obj_dict:
                        logger.debug("Uh, object named " + name + " is a vessel with no specified content")
                    init_strings.add("(= (contains %s) %s)" % (name, obj_dict.get("contains", "empty")))
                    stripped_dict["contains"] = obj_dict.get("contains", "empty")
                    if "is-open" in obj_dict and obj_dict["is-open"]:
                        init_strings.add("(is-open %s)" % name)
                        stripped_dict["is-open"] = True
                    if type == "glass":
                        assert "shaped" in obj_dict, "object %s has no 'shape'" % name
                        init_strings.add("(= (shaped %s) %s)" % (name, obj_dict["shaped"]))
                        stripped_dict["shaped"] = obj_dict["shaped"]
                if  any(True for x in supertypes if x in ["flower", "vase", "cup"]):
                    if not "colored" in obj_dict:
                        logger.debug("Uh, object named " + name + " is a flower, vase or cup with no specified color")
                    else:
                        init_strings.add("(= (colored %s) %s)" % (name, obj_dict.get("colored", "uncolored")))
                        stripped_dict["colored"] = obj_dict["colored"]
            elif any(True for x in supertypes if x in ["robot", "furniture", "human"]):
                if has_rooms:
                    if "in" in obj_dict:
                        loc = obj_dict["in"]
                        init_strings.add("(= (in %s) %s)" % (name, loc))
                        stripped_dict["in"] = obj_dict["in"]
                    else:
                        assert len(obj_dict) == 4, "Base %s is not located anywhere (no 'in' attribute found) and does not appear to be a type definition (dict: %s)" % (name, obj_dict)
                if "aligned" in obj_dict:
                    aligned = obj_dict["aligned"]
                    init_strings.add("(= (aligned %s) %s)" % (name, aligned))
                    stripped_dict["in"] = aligned
                if type == "robot":
                    if "mobile" in obj_dict and int(obj_dict["mobile"]):
                        init_strings.add("(mobile %s)" % name)
                        stripped_dict["mobile"] = True
                    else:
                        stripped_dict["mobile"] = False
                    if "arm-empty" in obj_dict and int(obj_dict["arm-empty"]):
#                        print "ARM-EMPTY = ", obj_dict["arm-empty"]
                        init_strings.add("(arm-empty %s)" % name)
                        stripped_dict["arm-empty"] = True
                    else:
                        stripped_dict["arm-empty"] = False
                    stripped_dict["at"] = obj_dict.get("at", "nowhere")
                    init_strings.add("(= (at %s) %s)" % (name, obj_dict.get("at", "nowhere")))
            elif type == "room":
                logger.debug("Connected predicates for room %s" % name)
                rooms = obj_dict["connected"]
                if isinstance(rooms, str):
                    rooms = [rooms]  # avoid string to be interpreted as list
                stripped_dict["connected"] = rooms
                for room in rooms:
                    logger.debug("adding connection (connected %s %s)" % (name, room))
                    init_strings.add("(connected %s %s)" % (name, room))
            neurobots_objects.append(stripped_dict)
    pddlproblem = pddl.Problem("neurobots-test-problem", problem_objects, [], None, domain)
    logger.debug("Problem created, neurobots world dict = %s", map(str, neurobots_objects))
    logger.debug("Problem created, problem_objects = %s", map(str, [str(n) for n in problem_objects]))
    logger.debug("Problem created, adding initial strings")
    for init_string in init_strings:
        logger.debug("Trying to process %s", init_string)
        if "= (" in init_string and ") 0" in init_string:  # # HACK! someties objects are detected by the camera falsely, so an object could be in the database without position. The attribute position is then present, but the position is False "(= (position object) 0)" instead. In this case we ignore the initial fact
            logger.warn("Warning: init_string ignored: %s", init_string)
            continue
        elem_parser = pddl.Parser(cStringIO.StringIO(init_string))
        init_elem = pddl.problem.InitElement.parse(iter(elem_parser.root), pddlproblem)
        logger.debug("  -- Init element: %s", init_elem)
        pddlproblem.init.append(init_elem)
    return (pddlproblem, neurobots_objects)
