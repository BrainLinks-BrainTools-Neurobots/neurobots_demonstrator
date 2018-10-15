from os.path import abspath, dirname

from configobj import ConfigObj

from pddllib import pddl
import cplan

import goals

import logging
logger = logging.getLogger('root')

#planner config
CONFIG_FN = "config.ini"

class Planner(object):
    def __init__(self, state, config_path):
        config = ConfigObj(config_path+CONFIG_FN, file_error=True, list_values=False, interpolation="ConfigParser")
        config['src_path'] = abspath(dirname(__file__))
        config = cplan.utils.config_to_struct(config)        
        self.planner = getattr(cplan.planner, config.base_planner.planner)(config)
        self.state = state
        self.problem = state.problem
        self.plan = None

    def set_state(self, state):
        self.state = state
        self.problem = state.problem
        

    def execute_action(self, action, args):
        logger.debug("execute action (%s %s)" % (action.name, " ".join(map(str, args))))
        
        new_state = None

        with action.instantiate(dict(zip(action.args, args)), self.problem):
            if not self.state.is_executable(action):
                return False
            
            new_state = self.state.copy()
            new_state.apply_effect(action.effect)
            
        init_facts = [f.to_init() for f in new_state.iterfacts()]

        new_prob = pddl.Problem(self.problem.name, self.problem.objects, init_facts, self.problem.goal, self.problem.domain, self.problem.optimization, self.problem.opt_func)

        self.problem = new_prob
        self.state = new_state

        return True
        
        
    def find_plan(self, goal):
        problem = self.problem.copy()
        pddlgoal = goal.pddl_goal()
        problem.goal = pddlgoal.copy(new_scope=problem)
        logger.debug(problem.goal.pddl_str())

        task = cplan.task.Task(problem, 0)
        # task.deadline = 220
        task.set_planner(self.planner)
        task.problem_to_state()
        task.replan()

        logger.debug(task.plan)
        self.plan = task.plan

    def get_plan(self):
        if self.plan is None:
            return []
        
        sorted_plan = self.plan.topological_sort()
        return [a for a in sorted_plan if a not in (self.plan.init_node, self.plan.goal_node)]
