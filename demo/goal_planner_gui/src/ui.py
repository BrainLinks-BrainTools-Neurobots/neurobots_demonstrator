import sys
from OpenGL import GL

from PyQt5.QtCore import pyqtProperty, pyqtSlot, Qt, QObject, QUrl, QVariant, QMetaObject, Q_ARG, QTimer, QThread
from PyQt5.QtGui import QKeySequence, QIcon
from PyQt5.QtWidgets import QApplication, QWidget, QShortcut, QVBoxLayout
from PyQt5.QtQuick import QQuickView
from PyQt5.QtQml import qmlRegisterType, QQmlContext

from cplan import planner, task, plans

from pddllib import pddl

import goals, partitions, constants
import planning
import ros_database
from threading import Thread

#import log

import traceback

import logging

logger = logging.getLogger('root')
logger2 = logging.getLogger('highlighted')

ui_constants = {
    'ActionStatus' : {
        'EXECUTABLE' : plans.ActionStatusEnum.EXECUTABLE,
        'IN_PROGRESS' : plans.ActionStatusEnum.IN_PROGRESS,
        'EXECUTED' : plans.ActionStatusEnum.EXECUTED,
        'UNSUCCESSFUL' : plans.ActionStatusEnum.UNSUCCESSFUL,
        'FAILED' : plans.ActionStatusEnum.FAILED }
}

GOAL_MENU = 1
ACTION_MENU = 2

class GoalMenuItem(QObject):
    def __init__(self, goal, current_index):
        super(GoalMenuItem, self).__init__()
        self.goal = goal
        self.arg = []

        if goal is not None:
            if isinstance(goal, goals.AlternativeActionGoal) or isinstance(goal, goals.AlternativeFunctionGoal):
                self.arg.append(ImageTextItem("Get other attribute", None))
            else:
                if goal.initial:
                    refs = goal.context.refs
                    if isinstance(goal, goals.FunctionGoal):
                        self.arg = [ImageTextItem(refs.get_name(goal.function), refs.get_image(goal.function))]
                    elif isinstance(self.goal, goals.ActionGoal):
                        self.arg = [ImageTextItem(refs.get_name(goal.action), refs.get_image(goal.action.name))]
                    else:
                        logger.debug("\n !!!!!! new %s !!!!!!!\n" % type(goal))
    
                else:
                    arg = goal.get_arg(current_index)
                    self.arg = [ImageTextItem(name, image) for name, image in arg.description()]
    
                if goal.is_unique() and current_index < len(goal.all_partitions()) - 1:
                    self.goal = goal.next()[0]
                    for arg in self.goal.all_partitions()[current_index + 1:]:
                        self.arg += [ImageTextItem(name, image) for name, image in arg.description()]

#             #logger.debug("button args: %s", map(str, self.arg))
#             if isinstance(goal, goals.AlternativeActionGoal) or isinstance(goal, goals.AlternativeFunctionGoal):
#                 #if isinstance(goal, goals.AlternativeActionGoal):
#                 #    print "Action:", goal.action
#                 #if isinstance(goal, goals.AlternativeFunctionGoal):
#                 #    print "Function:", goal.function, goal.context.refs.get_name(goal.function)
#                 #print "GoalMenuItem args: ", [a.text for a in self.arg]
#                 #print type(goal)        print "quoted ImageTextItem (%s,%s) for image" % (self._text, self._image)
# 
#                 #print "Goal forbid partition: ", goal.forbid_partiiton
#                 if len(self.arg) == 1:
#                     #self.arg.append(ImageTextItem("dummy", None))
#                     self.arg.append(ImageTextItem(self.arg[0]._text + " don't care", None))
#                 else :
#                     #print "arg: ", map(str, self.arg)
#                     text = self.arg[-1]
#                     #print "text: ", text._text, type(text)
#     
#                     if '=' in text._text:
#                         #print "replace: ", text._text
#                         if len(text._text) > 1:
#                             text._text = text._text.split('=')[0].strip() + " don't care"
#                         else:
#                             text._text = " don't care"
#                     else:
#                         #print "else text"
#                         text._text += " don't care" # +=
#                 #if len(self.arg[1]._text) > 1:
#                 #    self.arg[1]._text = self.arg[1]._text.split('=')[0]+" don't care"
#                 #self.arg[-1]._text = "don't care"

    @pyqtProperty('bool', constant=True)
    def initial(self):
        return self.goal.initial if self.goal is not None else False

    @pyqtProperty('QString', constant=True)
    def action(self):
        return ""

    @pyqtProperty('QVariant', constant=True)
    def argument(self):
        return QVariant(self.arg)


    @pyqtProperty('bool', constant=True)
    def more(self):
        if self.goal is None or self.goal.is_unique():
            return False
        return True

    @pyqtProperty('bool', constant=True)
    def back(self):
        return False

class MiscMenuItem(GoalMenuItem):
    def __init__(self, text, image, action, more):
        super(MiscMenuItem, self).__init__(None, 0)
        self.goal = None
        self.arg = [ImageTextItem(text, image)]
        self._action = action
        self._more = more

    @pyqtProperty('bool', constant=True)
    def initial(self):
        return True

    @pyqtProperty('QString', constant=True)
    def action(self):
        return self._action

    @pyqtProperty('bool', constant=True)
    def more(self):
        return self._more

class BackMenuItem(GoalMenuItem):
    def __init__(self):
        super(BackMenuItem, self).__init__(None, 0)
        self.goal = None
        self.arg = [ImageTextItem("back", "images/back.png")]
        
    @pyqtProperty('bool', constant=True)
    def initial(self):
        return False

    @pyqtProperty('QString', constant=True)
    def action(self):
        return "back"

    @pyqtProperty('bool', constant=True)
    def back(self):
        return True

class ChooseMenuItem(GoalMenuItem):
    def __init__(self):
        super(ChooseMenuItem, self).__init__(None, 0)
        self.goal = None
        self.arg = [ImageTextItem("choose", "images/face_smile.bmp")]

    @pyqtProperty('bool', constant=True)
    def initial(self):
        return False

    @pyqtProperty('QString', constant=True)
    def action(self):
        return "choose"

    @pyqtProperty('bool', constant=True)
    def back(self):
        return True

class ImageTextItem(QObject):
    def __init__(self, text, image ):
        super(ImageTextItem, self).__init__()
        self._text = text
        self._image = image
        
    @pyqtProperty('QString', constant=True)
    def text(self):
        return self._text

    @pyqtProperty('QString', constant=True)
    def image(self):
        return self._image

class RefItem(QObject):
    def __init__(self):
        super(RefItem, self).__init__()
        self.name_item = None
        self.args = None

    @pyqtProperty('QVariant', constant=True)
    def name(self):
        return self.name_item

    @pyqtProperty('QVariantList', constant=True)
    def arguments(self):
#         print "returning QVariant cast of %s" % [a for a in self.args]  
        return [QVariant(a) for a in self.args]


class CurrentGoalItem(RefItem):
    def __init__(self, goal):
        super(RefItem, self).__init__()
        self.goal = goal
        
        refs = goal.context.refs
        if isinstance(goal, goals.FunctionGoal):
            self.name_item = ImageTextItem(refs.get_name(goal.function), refs.get_image(goal.function))
        elif isinstance(self.goal, goals.ActionGoal):
#             print "Goal is action goals Refs class: %s" % refs.__class__            
#             print "goal.action: %s" % goal.action
#             print "goal.action class: %s" % goal.action.__class__
#             print "name: %s" % refs.get_name(goal.action)
#             print "image: %s" % refs.get_image(goal.action)
#             print "cdict: %s" %refs.__dict__
            self.name_item = ImageTextItem(refs.get_name(goal.action.name), refs.get_image(goal.action.name))

        self.current = goal.arg_index(goal.get_current_arg())

#         logger2.debug("Current Goal has current index : %d", self.current) 
        args = goal.args # [0:current]  
        self.args = [[ImageTextItem(name, image) for name, image in arg.description()] for arg in args]

    @pyqtProperty(int)        
    def current_param(self):
        #print "get current goal parameter index : ", self.current
        return self.current

class PlanItem(RefItem):
    def __init__(self, action, gcontext):
        super(PlanItem, self).__init__()
        self.action = action
        self.context = gcontext
        self.compute_args()

        self.name_item = ImageTextItem(gcontext.refs.get_name(action.action.name), gcontext.refs.get_image(action.action.name))

    def compute_args(self):
        self.args = []
        relevant_args = set(pddl.visitors.visit(self.action.action.effect, pddl.visitors.collect_free_vars, []))
#         logger2.debug(map(str, relevant_args))
        for o, a in zip(self.action.arguments, self.action.action.args):
#             logger2.debug("%s, %s", o, a)
            if a not in relevant_args:
                continue
            p = partitions.Partition.get_initial_partition(o.type, self.context.refs)
            ref_entry = p.get_child_for_object(o)
            final_entry = None
            check_information = False
            while ref_entry is not None and ref_entry != final_entry:
                final_entry = ref_entry
                ref_entry = ref_entry.expand_single(o, check_information)
                check_information = True
                
#                 logger2.debug("(%s) %s --- (%s) %s", type(final_entry).__name__, final_entry, type(ref_entry).__name__, ref_entry)
#                 if (ref_entry != None and final_entry != None):
#                     logger2.debug("%s, %s", type(ref_entry.ref).__name__, type(final_entry.ref).__name__)
#                     logger2.debug("%s, %s", ref_entry.ref.__hash__(), final_entry.ref.__hash__())
#                     logger2.debug("%s, %s", ref_entry.partition.__hash__(), final_entry.partition.__hash__())
#                     logger2.debug("%s, %s", hash(ref_entry), hash(final_entry))
#                     logger2.debug("%s, %s", ref_entry.__hash__(), final_entry.__hash__())
#                 else:
#                     logger2.debug("None")
#                 logger2.debug(ref_entry != final_entry)
                
#             logger2.debug(final_entry)
            self.args.append([ImageTextItem(name, image) for name, image in final_entry.description()])

    @pyqtProperty('QString', constant=True)
    def action_status (self):
        return self.action.status


qmlRegisterType(GoalMenuItem, 'Goals', 1, 0, 'GoalMenuItem')
qmlRegisterType(ImageTextItem, 'Goals', 1, 0, 'ImageTextItem')
qmlRegisterType(RefItem, 'Goals', 1, 0, 'RefItem')
qmlRegisterType(CurrentGoalItem, 'Goals', 1, 0, 'CurrentGoalItem')
qmlRegisterType(PlanItem, 'Goals', 1, 0, 'PlanItem')

MENU_MODE_GOAL = 0
MENU_MODE_PLAN = 1

rootMenu = [MiscMenuItem("Plan for Goal", "images/plan.png", "goals", True),
            MiscMenuItem("Execute Action", "images/step.png", "actions", True),
            MiscMenuItem("Quit", "images/quit.png", "quit", False)]

class Selector(QWidget):
    def __init__(self, goal_context, config_path, experimentlogger):
        self.path = config_path
        super(Selector, self).__init__()
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.ros_poller)
        self.setup_ui()

        self.AUTOEXECUTE = False
        self.ignore_move_events = False

        self.context = goal_context
        self.planner = None
        self.current_plan = None
        self.current_action_succeeded = False
        self.current_action_index = 0
        self.current_menu_index = 0
        self.task = None

        self.planner = planning.Planner(goal_context.init, self.path)
        self.ros_handler = None
        self.experiment_logger = experimentlogger

        self.init_menu()
        self.show_goal_ui()

    def ros_poller(self):
#         print "checking for ROS triggered changes in ROS event queue"
        if self.ros_handler and self.ros_handler.event_queue:
#            print "Processing events from ROS event queue. There are %d events" % len(self.ros_handler.event_queue)
            event = self.ros_handler.event_queue.pop(0)
#            print "popped ", str(event[0])
            if "teleop" not in str(event[0]):
                event[0](*event[1]) # event is a pair (method,[params]) the asteriks unpacks parameters
            else:
                logger.debug("HEY, don't teleoperate while actions are running: %s", str(event[0]))
#            print "executed ", str(event[0])

    def setup_ui(self):
        self.view = QQuickView()
        self.view.setResizeMode(QQuickView.SizeRootObjectToView)
        container = QWidget.createWindowContainer(self.view)
        container.setFocusPolicy(Qt.StrongFocus)
        layout = QVBoxLayout()
        layout.addWidget(container)
        self.setLayout(layout)

        ctx = self.view.rootContext()
        ctx.setContextProperty("selector", self)
        ctx.setContextProperty("C", ui_constants)

        self.set_menu([])
        self.set_plan([])
        self.set_current(None)
        self.view.setSource(QUrl(self.path+'ui.qml'))

        self.resize(800, 800)
        self.move(300, 300)
        self.setWindowTitle('Goal Planner GUI')
        self.setWindowIcon(QIcon('images/icon.png'))

        reload = QShortcut(QKeySequence("Ctrl+R"), self)
        reload.activated.connect(self.reload_ui)

        quit = QShortcut(QKeySequence("Ctrl+C"), self)
        quit.activated.connect(self.quit)
        quit = QShortcut(QKeySequence("Esc"), self)
        quit.activated.connect(self.quit)

        self.ros_timer.start(10)
        self.showMaximized();

    def test_populate(self):
        l = ["give", "grasp", "drop", "open", "close"]
        self.set_menu(l)

    def set_current(self, goal):
        ctx = self.view.rootContext()
        ctx.setContextProperty("currentGoal", QVariant(goal))
        self.current = goal # do not free the list while it's in the model

    def set_menu(self, elems):
        ctx = self.view.rootContext()
        ctx.setContextProperty("selectionsModel", QVariant(elems))
        self.elems = elems # do not free the list while it's in the model

    def set_plan(self, elems):
        ctx = self.view.rootContext()
        ctx.setContextProperty("planModel", QVariant(elems))
        self.plans = elems # do not free the list while it's in the model

    def activate_loading(self):
        compute_image_focus = self.view.rootObject().findChild(QObject, "compute_image_focus")
        compute_image_focus.turnOn()
        
    def deactivate_loading(self):
        compute_image_focus = self.view.rootObject().findChild(QObject, "compute_image_focus")
        compute_image_focus.turnOff()

    @pyqtSlot()
    def goal_menu(self):
        class create_goal_menu(QThread):
            def __init__(self, instance):
                QThread.__init__(self)
                self.instance = instance
                
            def __del__(self):
                self.wait()
                
            def run(self):
                if constants.SHOW_FUNCTION_GOALS:
                    self.instance.next_goals = goals.FunctionGoal.initial_goals(self.instance.context) + goals.ActionGoal.initial_goals(self.instance.context)
                else:
                    self.instance.next_goals = goals.ActionGoal.initial_goals(self.instance.context)
        #         logger2.debug("Next goals: %s", map(str, self.next_goals))
                self.instance.next_goals = [g for g in self.instance.next_goals if self.instance.filter_goal(g)]
        """
        Shows the selection of options
        """
        self.experiment_logger.log_performance("Goal Menu shows next goals ...")
        self.create_goal_menu_thread = create_goal_menu(self)
        self.create_goal_menu_thread.finished.connect(self.create_goal_menu_done)
        self.create_goal_menu_thread.start()
        self.activate_loading()

    def create_goal_menu_done(self):
        items = [GoalMenuItem(g, -1) for g in self.next_goals]
        items.append(BackMenuItem())
        self.current_goal = GOAL_MENU
        self.build_header(None)
        self.set_menu(items)
        self.experiment_logger.log_performance("Goal Menu shows next goals done", True)
        self.create_goal_menu_thread = None
        self.deactivate_loading()
        
    @pyqtSlot()
    def action_menu(self):
        self.next_goals = goals.ExecActionGoal.initial_goals(self.context)

        items = [GoalMenuItem(g, -1) for g in self.next_goals]
        items.append(BackMenuItem())

        self.current_goal = ACTION_MENU
        self.build_header(None)
        self.set_menu(items)

    @pyqtSlot(int)
    def select_goal(self, index):
#         logger2.debug("select goal")
#         image = self.view.rootObject().findChild(QObject, "compute_image")
#         image.turnOn()
#        self.experiment_logger.log_performance("user chose goal #%s" %index)

        #traceback.print_stack()
        if self.next_goals:
            self.display_goal(self.next_goals[index])
        #self.experiment_logger.log_performance("goal #%s displayed" %index)
#         image.turnOff()


    @pyqtSlot(int)
    def select_action(self, index):
        if self.ignore_move_events:
            logger.debug("aborting action execution because an action is already in progress")
            return
        #print self.current_action_index, index
        if not (self.current_action_index == index or self.current_action_index + 1 == index):
            logger.debug("Execute previous actions first!!!")
            return
        if self.current_plan and index < len(self.current_plan):
            if self.current_plan[index].status == plans.ActionStatusEnum.EXECUTED:
                logger.debug("The action was already executed")
                return

            plan_node = self.current_plan[index]

            plan_node.status = plans.ActionStatusEnum.IN_PROGRESS
            self.display_status_text("Action is in progress")
            self.ignore_move_events=True
            self.ros_handler.execute_on_robot(plan_node.action, plan_node.arguments)

            items = [PlanItem(pnode, self.context) for pnode in self.current_plan]
            self.set_plan(items)
            planView = self.view.rootObject().findChild(QObject, "planView")
            planView.setProperty("currentIndex", index)
            self.current_action_index = index
            self.current_action_succeeded = False
            logger.debug("action selection of %s done, action executed" %index)
#            self.show_plan_ui()

    def action_executed(self):
        success = self.current_action_succeeded
        planView = self.view.rootObject().findChild(QObject, "planView")
        index = self.current_action_index
        logger.debug("current action index %d was executed. success = %s"% (index, success))
        if success:
            plan_node = self.current_plan[index]
            if not self.planner.execute_action(plan_node.action, plan_node.arguments):
                plan_node.status = plans.ActionStatusEnum.FAILED
                logger.fatal("\033[91mWarning warning warning, Planner found action to be infeasible... aborting\033[0m")
                #assert False
            self.current_plan[index].status = plans.ActionStatusEnum.EXECUTED
            #logger.debug("will refresh context now")
            #self.refresh_context(self.planner.problem)
        else:
            self.current_plan[index].status = plans.ActionStatusEnum.UNSUCCESSFUL
        self.ignore_move_events=False
        logger.debug("select next action in plan GUI")
        planView = self.view.rootObject().findChild(QObject, "planView")
        items = [PlanItem(pnode, self.context) for pnode in self.current_plan]
        self.set_plan(items)
        if success and index + 1 < len(self.current_plan):
            planView.setProperty("currentIndex", index + 1) # select next plan step
            if self.AUTOEXECUTE:
                logger.debug("Auto-executing next")
                pui = self.view.rootObject().findChild(QObject, "planner_ui")
                pui.teleop_select()
            else:
                logger.debug("autoexecute ist turned off")
                self.refresh_context(self.planner.problem)
        else:
            planView.setProperty("currentIndex", index)
#        self.show_plan_ui()

    def handle_world_update(self):
        world = self.view.rootObject().findChild(QObject, "world_image")
        world.turn_world()
        self.init_menu()
        self.show_goal_ui()
        self.planner.set_state(self.context.init)
        # print("World change happened, world turning image displayed. Now I check the plan")
        # print("goal= ",self.goal)
        # self.plan(goal)


    @pyqtSlot()
    def back(self):
        if self.ignore_move_events and self.current_plan[self.current_action_index].status == plans.ActionStatusEnum.IN_PROGRESS:
            #action is currently running -> abort it
            self.current_plan[self.current_action_index].status = plans.ActionStatusEnum.UNSUCCESSFUL
            planView = self.view.rootObject().findChild(QObject, "planView")
            items = [PlanItem(pnode, self.context) for pnode in self.current_plan]
            self.set_plan(items)
            planView.setProperty("currentIndex", self.current_action_index)
            self.ros_handler.abort_current_action();
            self.ignore_move_events = False
            return
        if self.menu_stack:
            g = self.menu_stack.pop()
            self.current_goal = None
            if g == GOAL_MENU:
                self.goal_menu()
            elif g == ACTION_MENU:
                self.action_menu()
            else:
                self.display_goal(g)
        else:
            self.init_menu()
            self.show_goal_ui()
    @pyqtSlot()
    def choose(self):
        logger.debug("choose pressed")
        # self.menu_stack.pop()
        goal = self.current_goal
        if goal.is_unique():
            if isinstance(goal, goals.ExecActionGoal):
                self.execute_action_goal(goal)
            else:
                self.plan(goal)
            return

        if self.current_goal is not None:
            self.menu_stack.append(self.current_goal)
        current_index = goal.arg_index(goal.get_current_arg())
        logger.debug("index = %d", current_index)
        logger.debug("\033[93m next from current goal----------\n----------\n--------\n-------")
        self.next_goals = [g for g in self.current_goal.next_all() if not g.is_empty() and self.filter_goal(g)]
        #self.next_goals = [g for g in self.current_goal.next() if not g.is_empty() and self.filter_goal(g)]
        logger.debug("\033[0m next_goals: %s", map(str, self.next_goals))
        items = [GoalMenuItem(g, current_index) for g in self.next_goals]

        if self.menu_stack:
            items.append(ChooseMenuItem())
            items.append(BackMenuItem())

        self.build_header(goal)
        self.set_menu(items)

    def reload_ui(self):
#        print "reload_ui clear cache"
        self.view.engine().clearComponentCache()
#        print "reload_ui set menu"
        self.set_menu([])
#        print "reload_ui set plan"
        self.set_plan([])
#        print "reload_ui set current"
        self.set_current(None)

        if self.mode == MENU_MODE_GOAL:
            logger.debug("reload_ui show goal")
            self.show_goal_ui()
        else:
            logger.debug("reload_ui show plan")
            self.show_plan_ui()
        #print "reload_ui updatself.menu_stack.append(self.current_goal)e"
        self.repaint()

    @pyqtSlot()
    def quit(self):
	#pass
        # self.hide()
        sys.exit(0)

#   @pyqtSlot('QString')
    def display_status_text(self, displaytext):
#        print "about to display in statusbar >%s<" % displaytext
        bar = self.view.rootObject().findChild(QObject, "statusbar")
#        QMetaObject.invokeMethod(bar, "display", Qt.DirectConnection, Q_ARG(QVariant, displaytext))
        #bar.display("%s" % displaytext)
        bar.display("")

    def action_help(self, action):
        help = ""
        if action == "go":
            help = "go [robot] [room], i.e., move the robot into a room"
        elif action == "approach":
            help = "approach [robot] [target base], i.e., move the robot to a target (furniture, flowerbed, humans) in the current room"
        elif action == "drop":
            help = "drop [robot] [object] [target], i.e., drop an object on a target (furniture, flowerbed, humans)"
        elif action == "arrange":
            help = "arrange [robot] [flower] [vase], i.e., arrange a flower in a vase"
        elif action == "open":
            help = "open [robot] [bottle], i.e., lets the robot open a bottle"
        elif action == "drink":
            help = "drink [human] [vessel] [content], i.e., the robot gives a drink in a vessel with the specified content to the human"
        elif action == "pour":
            help = "pour [from vessel] [to vessel] [content], i.e., the robot pours a liquid (3. parameter) from the first vessel into the second one"
        elif action == "give":
            help = "give [robot] [object] [human], i.e., the robot brings an object to the human"
        elif action == "grasp":
            help = "grasp [robot] [object], i.e., the robot grasps an object"
        elif action == "pick":
            help = "pick [robot] [flower] [vase], i.e., the robot picks a flower from a vase"
        else:
            help = "Unknown action " + action
        return help

    @pyqtSlot(int)
    def display_status(self, index):
        text = "Unknown Element with Index "+str(index)
#         print "Displaying ", text
#         print "Current goal -> ", self.current_goal
#         print "Current menustack -> ", self.menu_stack
#         print "current -> ", self.current
#         print "Mode -> ", self.mode
        self.current_menu_index =  index
        if self.mode == MENU_MODE_GOAL:
            if self.current_goal:
#                 print "ui.py/display_status debug dump of current goal:"
#                 print self.current_goal.__class__
#                 if type(self.current_goal) is not int:
#                     print "active = %s" % self.current_goal.arg_index 
#                     print "goal args= "
#                     for arg in self.current_goal.args:
#                         print str(arg)
                if (index < len(self.next_goals)):
                    next_goal = self.next_goals[index]
                    text = str(next_goal)
                    if isinstance(next_goal, goals.ActionGoal):
                        action = next_goal.action.name
                        help = self.action_help(action)
                        text = "Current: " + text + "\nHelp: " + help
                elif(index == len(self.next_goals)):
                    text = "BACK"
            else:
                text = rootMenu[index].arg[0].text
        else:
            if index < len(self.current_plan):
                text = str(self.current_plan[index])
            else:
                text = "Cannot display status of step %s in a plan of length %s" % (index, len(self.current_plan))
                
#         if self.current_goal and isinstance(self.current_goal, goals.ActionGoal):
#             action = self.current_goal.action.name
#             help = self.action_help(action)
#             text = "Current: " + text + "\nHelp: " + help
        self.display_status_text(text)
        self.experiment_logger.log_navigation(index, text)

    def init_menu(self):
#         logger2.debug("initializing menu")
#         image = self.view.rootObject().findChild(QObject, "compute_image")
#         image.turnOn()
        self.mode = MENU_MODE_GOAL
        self.menu_stack = []
        self.current_goal = None
        self.build_header(None)

        ctx = self.view.rootContext()
        # root = self.view.rootObject().findChild(QObject, "rootMenu")
        logger.debug("set context property to root menu %s", rootMenu)
#         image.turnOff()
        ctx.setContextProperty("selectionsModel", rootMenu)
        self.set_menu(rootMenu)

    def show_goal_ui(self):
        self.mode = MENU_MODE_GOAL
        gui = self.view.rootObject().findChild(QObject, "goal_ui")
        pui = self.view.rootObject().findChild(QObject, "planner_ui")
        pui.setProperty("visible", False)
        gui.setProperty("visible", True)
        gui.setProperty("focus", True)

    def show_plan_ui(self):
        logger.debug("show plan ui in mode %s", self.mode)
        self.mode = MENU_MODE_PLAN
        gui = self.view.rootObject().findChild(QObject, "goal_ui")
        pui = self.view.rootObject().findChild(QObject, "planner_ui")
        gui.setProperty("visible", False)
        pui.setProperty("visible", True)
        pui.setProperty("focus", True)
        self.display_status(0)

    def add_image_provider(self, image_provider):
        self.image_provider = image_provider
        self.view.rootContext().engine().addImageProvider(
            'ros_image_provider', self.image_provider
        )
        image_provider.connect_gui(self)

    # def change_focuspoint_color_srv(self):
        # widget = self.view.rootObject().findChild(QObject, 'crosshair')

    # def switch_to_video_view(self):
    #     self.view.rootObject().switch_to_video_view()

    # def switch_to_main_view(self):
    #     self.view.rootObject().switch_to_main_view()

    def build_header(self, goal):
        if goal is None or goal.__class__ == goals.GoalSpec:
            self.set_current(None)
            return

        self.set_current(CurrentGoalItem(goal))

    def display_goal(self, goal):   
        class display_goal_menu(QThread):
            def __init__(self, instance, goal):
                QThread.__init__(self)
                self.instance = instance
                self.goal = goal
                
            def __del__(self):
                self.wait()
                
            def run(self):
                if self.goal.is_unique():
                    if isinstance(self.goal, goals.ExecActionGoal):
                        self.instance.execute_action_goal(self.goal)
                    else:
                        self.instance.plan(self.goal, False)
                        self.instance.show_plan_flag = True
                    return
        
                self.instance.current_index = self.goal.arg_index(self.goal.get_current_arg())
                self.instance.next_goals = [g for g in self.instance.current_goal.next_flattened() if not g.is_empty() and self.instance.filter_goal(g)]

        logger.debug("will display goal %s", str(goal))
        self.experiment_logger.log_performance("ui.show_goal_ui display goal")

#         #CalledGoals.Instance().call_goal(goal)
#         if goal.is_unique():
#             if isinstance(goal, goals.ExecActionGoal):
#                 self.execute_action_goal(goal)
#             else:
#                 self.plan(goal)
#             return
# 
#         if self.current_goal is not None:
#             self.menu_stack.append(self.current_goal)
# 
#         self.current_goal = goal
#         self.current_index = goal.arg_index(goal.get_current_arg())
        #tmp_goals = [(-CalledGoals.Instance().get_calls(g)*100+i, g) for i, g in enumerate(self.current_goal.next_flattened()) if not g.is_empty() and self.filter_goal(g)]
        #tmp_goals.sort()
        #self.next_goals = [g for _, g in tmp_goals]
        if self.current_goal is not None:
            self.menu_stack.append(self.current_goal)
        
        self.current_goal = goal
        self.show_plan_flag = False
        
        self.display_goal_menu_thread = display_goal_menu(self, goal)
        self.display_goal_menu_thread.finished.connect(self.display_goal_menu_done)
        self.display_goal_menu_thread.start()
        self.activate_loading()
        
#         self.next_goals = [g for g in self.current_goal.next_flattened() if not g.is_empty() and self.filter_goal(g)]

    def display_goal_menu_done(self):
        if self.show_plan_flag:
            self.plan_update_gui()
            if self.experiment_logger.autoperform:
                self.quit()
        
        items = [GoalMenuItem(g, self.current_index) for g in self.next_goals]
        if self.menu_stack:
            #items.append(ChooseMenuItem())
            items.append(BackMenuItem())

        self.build_header(self.current_goal)
        self.set_menu(items)
        self.experiment_logger.log_performance("ui.display_goal_menu_done", True)
        self.display_goal_menu_thread = None
        self.deactivate_loading()

    def execute_action_goal(self, action_goal):
        action = action_goal.action
        problem = self.context.problem

        #instantiate an action that matches the selected action and execute it
        valid_args = action_goal.get_matches()
        action_args = []
        for aarg in action_goal.action.args:
            if aarg in action_goal.used_arguments:
                i = action_goal.used_arguments.index(aarg)
                objects = set(tup[i] for tup in valid_args)
                # print self.args[i], map(str, objects)
            else:
                objects = list(problem.get_all_objects(aarg.type))
            action_args.append(objects)

        inst_function = action.get_inst_func(self.context.init)
        args = None
        for mapping in action.smart_instantiate(inst_function, action.args, action_args, problem):
            args = [mapping[a] for a in action.args]
            break

        self.planner.execute_action(action, args)
        #self.refresh_context(self.planner.problem)

        self.menu_stack = []
        self.action_menu()

    def refresh_context(self, problem, refs=None):
        if refs is None: # use old references instead
            refs = self.context.refs
        self.context = goals.GoalContext(problem, refs)

    def plan_update_gui(self):
        items = [PlanItem(pnode, self.context) for pnode in self.current_plan]
        self.set_plan(items)
        self.show_plan_ui()

    def plan(self, goal, update_gui=True):
        self.menu_stack = []
        self.current_goal = None
        self.current_action_index = 0

        assert self.planner is not None
        self.planner.find_plan(goal)
        self.current_plan = self.planner.get_plan()

        if update_gui:
            self.plan_update_gui()

    def filter_goal(self, goal):
        if goal.is_universal():
            return False
        return True
