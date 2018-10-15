#!/usr/bin/env python

# A goal consists of multiple arguments, which are PartitionEntries. 
# Other ref: contains a list of excluding References

import sys, time
import argparse
import rospy
import rospkg

from planner_control_msgs.srv import *
from database_msgs.msg import database_change

import experiment_log

import logging, constants
from pprint import pprint
from inspect import getmembers

from cookielib import logger

#parse argument here because we need them for logging
ap = argparse.ArgumentParser(description="A PDDL goal generation tool")
ap.add_argument('domain', help="planning domain")
ap.add_argument('references', help="file that contains information on what is referable")
ap.add_argument('problem', nargs='?', default='', help="problem description as a pddl problem")
ap.add_argument('--serialized-partitions', nargs=1, default=None, type=str, help="the filename of the serialized partition (without extension)")
ap.add_argument('-d', '--debug', action="store_true")
ap.add_argument('-r', '--refs', action="store_true", help="print possible references for types and exit")
ap.add_argument('-t', '--dump-tree', action="store_true", help="generate and print all goals and exit")
ap.add_argument('-i', '--interactive', action="store_true", help="interactive debug mode")
ap.add_argument('-T', '--type', type=str, help="use <type> for any operations that operate on types")
ap.add_argument('-e', '--evaluate', action="store_true", help="evaluate on the goals of the pddl problem")
ap.add_argument('-x', '--executable', action="store_true", help="only executable actions")
ap.add_argument('-o', '--objects', action="store_true", help="print references to each object in the problem and exit")
ap.add_argument('-g', '--gui', action="store_true", help="show GUI")
ap.add_argument('-m', '--mute', action="store_true", help="whether to mute or not the console output")
ap.add_argument('--autoexecute', action="store_true", help="autoexecute next action in planning mode")
ap.add_argument('--auto_perf_ex', action="store_true", help="auto select menu entries to test performance")
ap.add_argument('-ros', '--ros', nargs='*', default=False)
ap.add_argument('-it', '--image-topic', nargs='*', default=None, type=str, help="start execution with a specified" +
                                                                                " list of image topics to subscribe.")
ap.add_argument('-cit', '--cimage-topic', nargs='*', default=None, type=str,
                help="start execution with a specified list of compressed image topics to subscribe.")
#%ap.add_argument('-l', '--log', nargs='*', default=None, type=str)

args = ap.parse_args()

def setupLogger():
    global logger, logger2
    logging._handlerList = []
    #remove potential ros handlers (if we don't remove them, no default handler for stdout is available!)
    for hdlr in logging.root.handlers:
        logging.root.removeHandler(hdlr)
    logger = logging.getLogger('root')
    logger_handler = logging.StreamHandler()  # Handler for the logger
    logger.handlers = []
    logger.addHandler(logger_handler)
    logger_handler.setFormatter(logging.Formatter("\033[92m[%(levelname)s, %(filename)s:%(lineno)s]\033[0m %(message)s"))
    logger.setLevel(logging.DEBUG if args.debug else logging.WARNING)
    logger.disabled = False
    logger.propagate = False
        
    if args.mute:
        logger.setLevel(logging.CRITICAL)
        
    logger2 = logging.getLogger('highlighted')
    logger_handler = logging.StreamHandler()  # Handler for the logger
    logger2.handlers = []
    logger2.addHandler(logger_handler)
    logger_handler.setFormatter(logging.Formatter('\033[91m[%(levelname)s, %(filename)s:%(lineno)s]\033[0m %(message)s'))
    logger2.setLevel(logging.DEBUG if args.debug else logging.WARNING)
    logger2.disabled = False
    logger2.propagate = False
    
#logging
#import logging
#LEVEL = logging.DEBUG if args.debug else logging.WARNING
#FORMAT = '%(message)s'
#logging.basicConfig(format=FORMAT, level=LEVEL)

from pddllib import pddl
import pddllib.extensions

pddl.register_extension_package(pddllib.extensions)

import goals, reference_list, references, partitions, evaluation

rospack = rospkg.RosPack()
config_path = rospack.get_path('goal_planner_gui')
config_path += '/src/'

setupLogger()

if args.mute:
    logger.setLevel(logging.CRITICAL)

logger.info("Loading domain %s" % args.domain)
dom = pddl.load_domain(args.domain)

def create_world_hash_file(file, neurobots_dict):
    import os.path
    file = open(file, 'w')
    h = hash(str(neurobots_dict))
    logger2.debug("Write hash of world to disk: %s", h)
    file.write(str(h))
    file.close()
    
def check_world_hash(file, neurobots_dict):
    import os.path
    logger2.debug("world hash file: %s!", file)
    if os.path.isfile(file):
        h = hash(str(neurobots_dict))
        file = open(file, 'r')
        filehash = int(file.read())
        logger2.debug("Found hash of world in file: %s, current hash: %s", filehash, h)
        return h == filehash
    else:
        logger2.debug("Hashes not equal or file not available")
        return False

def create_goal_context():
    #print "The ROS argument is:", args.ros
    neurobots_dict = None
    if args.ros != False: # != False makes sure that processing --ros without parameters (an empty list) ends up in the second branch
        logger.info("Fetching problem from ROS database")
        import ros_database
        (prob,neurobots_dict) = ros_database.get_problem_from_database(dom)
    else:
        logger.info("Loading problem %s" % args.problem)
        prob = pddl.load_problem(args.problem, dom)
    logger.debug("REF: " + args.references)
    refs = reference_list.ReferenceList(prob, args.references)
    
    serialization_path = "/tmp/reference_list" if not args.serialized_partitions else (args.serialized_partitions[0])
    file_serialization = serialization_path + ".ser"
    file_serialization_hash = serialization_path + "_hash.ser"
    
    world_hash_ok = check_world_hash(file_serialization_hash, neurobots_dict)
    constants.WORLD_HASH_OK = world_hash_ok
    constants.SERIALIZATION_PATH = serialization_path
    
    if not world_hash_ok:
        create_world_hash_file(file_serialization_hash, neurobots_dict)
    
    if not constants.SERIALIZE_PARTITIONS or (constants.SERIALIZE_PARTITIONS and not refs.deserialize(file_serialization)):
        logger.debug("\033[93m atomic_partitions\033[0m")
        refs.create_atomic_partitions()
        logger.debug("\033[93m extended_partitions\033[0m")
        refs.create_extended_partitions()
        logger.debug("\033[93m optimistic_partitions\033[0m")
        refs.create_optimistic_partitions()
        if constants.SERIALIZE_PARTITIONS:
            refs.serialize(file_serialization)
            
    if args.refs:
        logger.debug("Refs...")
        logger.debug("refs: " + args.refs)
        if args.type is not None:
            types = [dom.types[args.type]]
            logger.debug("types:" + types)
        else:
            types = dom.types.itervalues()

        for t in types:
            logger.debug("\n")
            logger.debug(t + ":")
            for p in refs.type_partitions[t]:
                logger.debug( "     %.2f|%.2f:  %s" % (p.information(), p.information(potential=True), p))

            logger.debug( "   total I: ~%.2f" % refs.type_information[t])

        # for t in types:
        #     groups = refs.filter_groups(refs.groups, typ=t)
        #     if not groups:
        #         continue
        #     print t, ":"
        #     best = refs.get_best_groups(t)
        #     print "   ", refs.group_gain(best, typ=t), map(lambda x: map(str, x), best)
        #     print "\n"
        #     for g in refs.get_all_groups(t):
        #         I = refs.group_gain([g], typ=t)
        #         print "      ", I , map(str, g)
        exit(0)
    return (goals.GoalContext(prob, refs),neurobots_dict)

def print_goal(g, index):
    reachable = g.get_reachable_goals()
    num_reachable = g.get_num_reachable()
    num_reached = g.get_num_reached()
    total_relaxed_dist = sum(g.context.rpg.get_ff_distance(f) for f in reachable)
    avg_cost = float(total_relaxed_dist+1) / (num_reachable+1)
    score = num_reachable / avg_cost**0.5
    comp = g.ref_count()
    status = ""
    if g.is_unique():
        status = "  "
    elif g.is_empty():
        status = "! "
    else:
        status = "->"

    return "%2d: %s %s  (%d/%d/%.2f/%.2f/%d)" % (index, status, str(g), num_reachable, num_reached, avg_cost, score, comp)


def dump_tree(goal, depth=0):
    if goal.is_unique() or goal.is_empty():
        return
    next_goals = goal.next()
    for i, g in enumerate(next_goals):
        logger.debug( "  " * depth, print_goal(g, i+1))
        dump_tree(g, depth+1)


def interactive_debug(gcontext):
    def parse(s, h, max_arg=None):
        if len(s) < len(h) or not s.startswith(h):
            return None
        if max_arg is None and s == h:
            return True, None
        try:
            i = int(s[len(h):])
            if max_arg is None or i <= max_arg:
                return True, i
        except ValueError, IndexOutOfRange:
            pass
        return None

    typ = None
    current_partition = None
    next_partitions = None
    done = False
    do_print = True
    while not done:
        if current_partition is None:
            tstr = None
            if args.type is not None:
                tstr = args.type
                args.type = None
            else:
                logger.info( "enter type:")
                tstr = raw_input().strip()

            if tstr == "q":
                done = True
            try:
                typ = dom.types[tstr]
                current_partition = partitions.Partition.get_initial_partition(typ, gcontext.refs)
                next_partitions = current_partition.children[0].all_next()
                do_print = True
                # current_groups = list(refs.get_all_groups(typ))
            except KeyError:
                logger.error( "type not found")
                continue

        else:
            if do_print:
                logger.info( "-----------------------------------------------")
                for i, p in enumerate(next_partitions):
                    if p.information(potential=True) <= 0.0000001:
                        logger.debug( "%2d: !%s" % (i, p.dump_stats()))
                    else:
                        logger.debug( "%2d: %s" % (i, p.dump_stats()))
                do_print = False

            c = raw_input().strip()

            if c == "p":
                do_print = True
            if parse(c, "o", len(next_partitions)-1):
                _, i = parse(c, "o")
                p = next_partitions[i]
                for c in p.children:
                    logger.debug(c)
                    logger.debug("         %s", map(str, c.get_matches()))

            elif c == "q":
                done = True
            elif c == "":
                if current_partition:
                    if current_partition.parent:
                        current_partition = current_partition.parent.partition
                        next_partitions = sum((c.all_next() for c in  current_partition.children), [])
                        do_print = True
                    else:
                        current_partition = None
            elif parse(c, "b"):
                _, depth = parse(c, "b")
                if depth is None:
                    depth = 3 # default
                if current_partition:
                    next_partitions = [c.search_best(maxdepth=depth) for c in  current_partition.children]
                    next_partitions = [p for p in next_partitions if p is not None]
                    do_print = True
            elif c == "r":
                next_partitions = sum((c.all_next() for c in  current_partition.children), [])
                do_print = True
            else:
                try:
                    i = int(c)
                    current_partition = next_partitions[i]
                    next_partitions = sum((c.all_next() for c in  current_partition.children), [])
                    do_print = True
                except ValueError:
                    logger.error("no such entry")


    exit (0)


def interactive(gcontext, initial):
    logger.debug( "--------------")
    current_goal = None
    all_goals = []

    stack = []
    done=False
    while not done:
        # next_goals = current_goal.next()
        logger.debug( "current: " + current_goal)
        if current_goal is None:
            next_goals = initial
        else:
            next_goals = [g for g in current_goal.next_flattened() if not g.is_empty()]
        # next_goals = [g for g in current_goal.next() if not g.is_empty()]
        logger.debug("%d goals selected" % len(all_goals))
        for i, g in enumerate(next_goals):
            logger.debug(print_goal(g, i+1))
            # print map(str, g.all_partitions())

        c = raw_input().strip()
        if c == "d":
            for i, g in enumerate(next_goals):
                logger.debug( map(str, g.all_partitions()))
                logger.debug( g.dump_goal_stats())
        elif c == "q":
            exit(0)
        else:
            try:
                i = int(c)
                if i > 0 and i <= len(next_goals):
                    stack.append(current_goal)
                    current_goal = next_goals[i-1]
                    if current_goal.is_unique():
                        logger.debug( "selected")
                        all_goals.append(current_goal)
                        current_goal = None
                        stack = []
                else:
                    logger.warn( "invalid choice")
            except ValueError, e:
                if stack:
                    logger.debug("go back")
                    current_goal = stack.pop()


def print_obj_refs(gcontext):
    olist = sorted(gcontext.problem.objects, key = lambda o: o.name)
    for o in olist:
        p = partitions.Partition.get_initial_partition(o.type, gcontext.refs)
        ref_entry = p.get_child_for_object(o)
        final_entry = None
        while ref_entry is not None and ref_entry != final_entry:
            final_entry = ref_entry
            ref_entry = ref_entry.expand_single(o)
        logger.debug(o + final_entry.text())
        logger.debug( final_entry.description())


planner_widget = None
app = None

def shutdown_hook():
    app.exit()

if args.gui and  __name__ == "__main__":
    # GUI
    from PyQt5.QtWidgets import QApplication, QWidget
    from PyQt5.QtCore import pyqtRemoveInputHook
    import ui
    import ros_database
    rospy.init_node('goal_planner_gui')
    #reset logger, because init_node will replace the logger object
    setupLogger()
    pyqtRemoveInputHook()    
    experimentlogger = experiment_log.ExperimentLogger(args.auto_perf_ex)
    logger.info("Created logger with auto = %s"%args.auto_perf_ex)
    experimentlogger.log_performance("App started")
    app = QApplication(sys.argv)    
    (gcontext,neurodict) = create_goal_context()    
    
    planner_widget = ui.Selector(gcontext, config_path, experimentlogger)
    roshandler = ros_database.RosDatabaseHandler(planner_widget, args, neurodict)
    
    planner_widget.ros_handler = roshandler
    planner_widget.experiment_logger = experimentlogger
    
    if args.autoexecute:
        logger.info( "Starting gui in autoexecute mode")
        planner_widget.AUTOEXECUTE = True
#    if args.log:
#        print "logging proband actions to %s" %args.log
        #TODO: implement this
    control_srv = rospy.Service('menu_control_signal', menu_control, roshandler.ros_menu_control)
    swap_srv = rospy.Service('focus_swap_signal', focus_swap, roshandler.ros_focus_swap)
    menu_navigation_srv = rospy.Service('get_menu_navigation_signal', get_menu_navigation_directions, roshandler.ros_get_menu_navigation)
    rospy.Subscriber('neurobots_database/database_change', database_change, roshandler.database_change_update)
    rospy.on_shutdown(shutdown_hook)
    experimentlogger.log_performance("Created Goal Context",True)
    sys.exit(app.exec_())


elif __name__ == "__main__":
    gcontext = goals.GoalContext(prob, refs)

    if args.dump_tree:
        current_goal = gcontext.new_goal()
        t0 = time.time()
        dump_tree(current_goal)
        logger.debug( "time for all goals: " + (time.time()-t0))
    elif args.evaluate:
        evaluation.evaluate(gcontext, args)
    elif args.interactive:
        interactive_debug(gcontext)
    elif args.objects:
        print_obj_refs(gcontext)
    else:
        if args.executable:
            init_g = goals.ExecActionGoal.initial_goals(gcontext)
        else:
            init_g = goals.FunctionGoal.initial_goals(gcontext) + goals.ActionGoal.initial_goals(gcontext)
        interactive(gcontext, init_g)
