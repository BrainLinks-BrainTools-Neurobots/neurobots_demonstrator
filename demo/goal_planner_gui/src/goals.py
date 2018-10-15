import math
import logging

from pddllib import pddl
from pddllib.pddl import visitors
from pddllib.state import state
from itertools import chain, product
from collections import defaultdict

import constants, partitions, relaxed_exploration
from utils import memoized
from references import *

import traceback

import copy
import pdb

import logging
logger = logging.getLogger('root')
logger2 = logging.getLogger('highlighted')

class GoalSpec(object):
    def __init__(self, context):
        self.context = context #GoalContext
        self.children = None
        self.value = None
        self.fix_score = None

    def next(self):
        """Return the children of the current goal"""
        if self.children is None:
            # print [str(g) for g in self._next()]
            sorted_results = [(-self.context.get_goal_score(g), str(g), g) for g in self._next()]
            sorted_results.sort()
            self.children = [g for _, _, g in sorted_results]
        # print "Retun next: ", map(str, self.children)
            # print "Return Goal: ", map(str,self.children)
        return self.children

    def next_all(self):
        """Return all children of the current goal"""
        logger.debug("Get all Children")
        sorted_results = [(-self.context.get_goal_score(g), str(g), g) for g in self._allnext()]
        sorted_results.sort()
        allchildren = [g for _, _, g in sorted_results]
        # print "Retun next: ", map(str, self.children)
            # print "Return Goal: ", map(str,self.children)
        return allchildren

    def next_flattened(self):
        """Return the children of the current goal, flattening the menu
        hierarchy if possible. Flattening will only happen as long as
        there are <= MAX_ENTRIES in the menu.

        """
        logger.debug("\033[93m get children of current Goal %s\033[0m", self)
        # traceback.print_stack()
        children = self.next()
        #logger.debug("children: %d", len(children))
        return children
    
        def sort_key(g):
            return (-self.context.get_goal_score(g), str(g))

        children = self.next()
        done = False
        ignored = set()
        while len(children) < constants.MAX_ENTRIES and not done:
            # promote high-value subgoals
            gcount = sum(g.get_num_reachable() for g in children)
            avg_gcount = float(gcount) / len(children)

            replace = None
            new_goals = None
            done = True
            for g in children:
                if g.get_num_reachable() > avg_gcount:
                    # try to extract subgoals
                    for g2 in g.next():
                        if g2 not in ignored and g2.get_num_reachable() > avg_gcount:
                            new_goals = g.extract_child(g2)
                            if new_goals is not None:
                                # print "flatten!", g2
                                break
                            ignored.add(g2)
                    if new_goals:
                        replace = g
                        break
            if new_goals:
                done = False
                children = [g for g in children if g != replace] + new_goals
                children.sort(key=sort_key)
        return children

    def is_unique(self):
        return len(self.next()) == 1

    def is_empty(self):
        return len(self.next()) == 0

    def get_arg(self, index):
        return None

    def arg_index(self, arg):
        return -1

    def get_completed_args(self):
        """ Return the argument entries that have been completely specified (or are quantified)"""
        return []

    def get_current_arg(self):
        """ Return the argument entry that is currently being refined """
        return None

    def _next(self):
        """Internal method that returns this goal's children. Supposed to be
           overridden by child classes

        """
        raise NotImplementedError
    def _allnext(self):
        """Internal method that returns this goal's children. Supposed to be
           overridden by child classes

        """
        raise NotImplementedError
    def _nextalternative(self):
        """Internal method that returns this goal's alternative children. Supposed to be
           overridden by child classes

        """
        raise NotImplementedError

    def is_reached(self):
        return self.get_num_reachable() <= self.get_num_reached()
    
    def is_reached(self, num_reachable):
        return num_reachable <= self.get_num_reached()

    def is_reachable(self):
        return self.get_num_reachable() > 0
    
    def is_reachable(self, num_reachable):
        return num_reachable > 0

    def ref_count(self):
        """ Return the number of references required to describe this goal"""
        return sum(pe.ref_count() for pe in self.all_partitions())

    def is_quantified(self):
        """ Return True if any reference of this goal is quantified """
        return any(pe.is_quantified() for pe in chain(self.args, [self.value]))

    def is_existential(self):
        """ Return True if any reference of this goal is quantified """
        return any(isinstance(pe, partitions.ExistentialPartitionEntry) for pe in chain(self.args, [self.value]))

    def is_universal(self):
        """ Return True if any reference of this goal is quantified """
        return any(isinstance(pe, partitions.UniversalPartitionEntry) for pe in chain(self.args, [self.value]))

    def check_quantified_goal_count(self, goals, args, goal_by_object_pos):
        """Compute the number of goals that match the current reference.
        TODO: describe arguments

        """
        objects = [set(pe.get_matches()) if pe is not None else set() for pe in args]
#         l = logger2#logging.getLogger("goal.reachability")
#         l.debug(map(lambda x: map(str, x), objects))
#         l.debug(type(goal_by_object_pos))
#         l.debug(self)
        def check(i, facts):
            if i >= len(args):
#                 l.debug("  "*i + "done %d %d", i, len(facts))
                return len(facts)

            if isinstance(args[i], partitions.ExistentialPartitionEntry):
#                 l.debug("  "*i + "E %d %s %d", i, args[i], len(facts))
                max_count = -1
                for o in objects[i]:
                    goal_by_o_pos = goal_by_object_pos[(o, i)]
#                     l.debug("  "*i + "%d %s %d %s", i, o, len(goal_by_o_pos),  # map(str,goal_by_object_pos[(o,i)])
#                             "-omit-")
                    count = check(i + 1, set(g for g in goal_by_o_pos if g in facts))
                    if count > max_count:
                        max_count = count
#                 l.debug (" "*i + "max:" + str(max_count))
                return max_count
            elif isinstance(args[i], partitions.UniversalPartitionEntry):
#                 l.debug("  "*i + "U %d %s %d", i, args[i], len(facts))
                min_count = len(facts) + 1
                for o in objects[i]:
                    count = check(i + 1, set(g for g in goal_by_object_pos[(o, i)] if g in facts))
                    if count < min_count:
                        min_count = count
#                 l.debug (" "*i + "min:" + str(min_count))
                return min_count

            elif args[i] is None:
#                 l.debug("  "*i + "- %d none %d", i, len(facts))
                return check(i + 1, facts)

            else:
#                 l.debug("  "*i + "- %d %s %d", i, args[i], len(facts))
                return check(i + 1, facts)

        # print map(str, args)
        c = check(0, set(goals))
#         l.debug("%d", c)
        # print "count: ", c
        return c


    def __str__(self):
        return "--"

    def is_equal(self, other):
        return str(self) == str(other)
        if self.children is None or other.children is None:
            return False
        for child in self.children:
            eqchild = False
            for ochild in other.children:
                if child == ochild:
                    eqchild = True
                    break
            if eqchild == False:
                return False
        return True


class FunctionGoal(GoalSpec):
    """Represents a goal to set the value of some function."""

    def __init__(self, context, function, args, value, initial=False):
        GoalSpec.__init__(self, context)
        self.function = function #pddllib.pddl.functions.Function
        self.args = args #list
        self.value = value #partitions.PartitionEntry
        self.initial = initial #partitions.PartitionEntry

        self.children = None
        self.cached_arg_matches = None
        
        if isinstance(self.value, partitions.ExistentialPartitionEntry):
            self.fix_score = -300000
        elif any(isinstance(r, OtherReference) for r in self.value.get_references()):# and all(not isinstance(r, FeatureReference) or isinstance(r, FunctionReference) for r in self.value.get_references()):
            self.fix_score = -200000

            
    @staticmethod
    def initial_goals(context):
        # print "\033[92m functions: ", map(str, context.refs.functions), "\033[0m"
        goals = [FunctionGoal.goal_from_function(context, f) for f in context.refs.functions]
        return [g for g in goals if not g.is_empty()]

    @staticmethod
    def goal_from_function(context, function):
        """Create a goal to set the value of the given function."""
        arg_partitions = []
        for a in function.args:
            # print "function args: ", a, " type ", type(a)
            p = partitions.Partition.get_initial_partition(a.type, context.refs).expand_unique()
            arg_partitions.append(p.children[0])
        # print "arg_partition", map(str,arg_partitions)

        val_partition = partitions.Partition.get_initial_partition(function.type, context.refs).expand_unique()

        return FunctionGoal(context, function, arg_partitions, val_partition.children[0], initial=True)



    def get_goal_from_args(self, args):
        return state.Fact(state.StateVariable(self.function, args[:-1]), args[-1])


    def get_reachable_objects(self, arg):
        """Return the set of objects that are potential values for the
        argument "arg"

        """
        if self.function not in self.context.mutable_functions:
            return set()

        if arg == self.value:
            index = len(self.args)
        else:
            index = self.args.index(arg)
        result = set()

        for args in self.get_matches():
#             logger.debug("matches: %s", map(str, args))
            f = self.get_goal_from_args(args)
            if self.context.rpg.is_relaxed_reachable(f):
                result.add(args[index])
        return result

    def get_reachable_goals(self):
        if self.function not in self.context.mutable_functions:
            return []
        potential_goals = (self.get_goal_from_args(a) for a in self.get_matches())
        return [f for f in potential_goals if self.context.rpg.is_relaxed_reachable(f) or f in self.context.init]

    def pddl_goal(self):
        """Return matching goals that are reachable but not yet reached as a PDDL goal formula"""

        # TODO: Quantified goals
        pddlgoals = [f.to_condition() for f in self.get_reachable_goals() if f not in self.context.init]
        return pddl.conditions.Disjunction(pddlgoals)


    def get_arg(self, index):
        if index < len(self.args):
            if isinstance(self.args[index], partitions.AlternativePartitionEntry):
                self.args[index] = self.args[index].forbid_partiiton.expand_parent
            return self.args[index]
        elif index == len(self.args):
            return self.value
        assert False

    def arg_index(self, arg):
        if arg == self.value:
            return len(self.args)
        return self.args.index(arg)

    def get_completed_args(self):
        for arg in self.args:
            if arg.is_leaf():
                yield arg
            else:
                return
        if self.value.is_leaf():
            yield self.value

    def get_current_arg(self):
        for arg in self.args:
            if not arg.is_leaf():
                return arg
        if not self.value.is_leaf():
            return self.value
        return None

    def get_future_args(self):
        found_current = False
        for arg in self.args:
            if found_current:
                yield arg
            elif not arg.is_leaf():
                found_current = True
        if found_current:
            yield self.value

    def allow_universal_expansion(self, partition):
        return (partition != self.value)

    def _next(self):
        """Overridden method that returns this goal's children. """
        logger.debug("expand: %s", self)

        prefix = list(self.get_completed_args())
        expanded_arg = self.get_current_arg()
        if not expanded_arg:
            # either we're done or no goal exists
            if self.is_reachable() and not self.is_reached():
                return [self]
            return []

        expansions = expanded_arg.expand(allow_universal=self.allow_universal_expansion(expanded_arg))
        suffix = list(self.get_future_args())

        result = []
        for entry in expansions:
            if entry.is_empty():
                continue

            # print "set reachable:", entry, id(entry), map(str,self.get_reachable_objects(expanded_arg))
            entry.set_reachable(self.get_reachable_objects(expanded_arg))

            new_args = prefix + [entry] + [e.clone() for e in suffix]
            
            
            if isinstance(entry, partitions.AlternativePartitionEntry):
                g = self.get_alternative_child(new_args, entry.forbid_partiiton)
                g.fix_score = -999999
            else:
                g = self.get_child(new_args)

#                 logger2.debug(map(str, entry.get_references()))

                if isinstance(entry, partitions.ExistentialPartitionEntry):
                    g.fix_score = -300000
                elif any(isinstance(r, OtherReference) for r in entry.get_references()) and all(not isinstance(r, RelationReference) for r in entry.get_references()):
                    g.fix_score = -200000
            
            if g.is_reachable() and not g.is_reached():
                result.append(g)

        #This can cause endless recursions (e.g., choosing "in" in menu) 
#         if len(result) == 1:
#             return result[0].next()
        return result

    def extract_child(self, goal):
        """Return this goal minus one of its children. Return None if this is
        not possible"""

        # find the first argument where the child differs from the parent
        prefix = []
        suffix = []
        entry = child_entry = None
        for pe, pe2 in chain(zip(self.args, goal.args), [(self.value, goal.value)]):
            if pe != pe2:
                entry = pe
                child_entry = pe2
            elif entry is None:
                prefix.append(pe)
            else:
                suffix.append(pe)

        # child is equal to parent or child is a quantified reference
        # in both cases, child matches all entries of parent, so there
        # is no remaining goal
        if entry is None or child_entry.is_quantified():
            return None

        new_partition = child_entry.partition.split_partition(child_entry)
        if not new_partition:
            assert False, "should not happen"
            return None

        result = []
        for entry in new_partition.children:
            entry.set_reachable(entry.reachable_objects)
            all_partitions = prefix + [entry] + suffix
            result.append(self.get_child(all_partitions[:-1], all_partitions[-1]))
        return result


    def get_child(self, new_args):
        """Return a child of this goal with the new arguments and value"""

        args = new_args[:-1]
        value = new_args[-1]

        g = FunctionGoal(self.context, self.function, args, value)

        # assert self.cached_arg_matches

        # propagate matching goals to child
        g.cached_arg_matches = []
        for arg_tuple in self.get_matches():
            matches = True
            for obj, arg, new_arg in zip(arg_tuple, chain(self.args, [self.value]), new_args):
                if arg != new_arg and not new_arg.ref.matches(obj):
                    matches = False
                    break
            if matches:
                g.cached_arg_matches.append(arg_tuple)
        return g
    def get_alternative_child(self, new_args, forbid_partiiton):
        """Return a child of this goal with the new arguments and value"""
        logger.debug("\nBuild new functiongoal")
        args = new_args[:-1]
        value = new_args[-1]
        logger.debug("args: %s", map(str, args))
        logger.debug("value: %s", value)

        g = AlternativeFunctionGoal(self.context, self.function, args, value, forbid_partiiton)

        # assert self.cached_arg_matches

        # propagate matching goals to child
        g.cached_arg_matches = []
        for arg_tuple in self.get_matches():
            matches = True
            for obj, arg, new_arg in zip(arg_tuple, chain(self.args, [self.value]), new_args):
                if arg != new_arg and not new_arg.ref.matches(obj):
                    matches = False
                    break
            if matches:
                g.cached_arg_matches.append(arg_tuple)
        return g

    def get_matches(self):
        if self.cached_arg_matches is not None:
            return self.cached_arg_matches

        self.cached_arg_matches = []
        arg_list = [pe.get_matches() for pe in self.args] + [self.value.get_matches()]

        if any(len(l) == 0 for l in arg_list):
            return []
        self.cached_arg_matches = list(product(*arg_list))
        # for args in product(*arg_list):
        #     # svar = state.StateVariable(self.function, args[:-1])
        #     self.cached_arg_matches.append(args)

        return self.cached_arg_matches


    def dump_goal_stats(self):
        goals = [self.get_goal_from_args(a) for a in self.get_matches()]
        reachable = [f for f in goals if self.context.rpg.is_relaxed_reachable(f)]
        reached = [f for f in goals if f in self.context.init]
        reachable_c = self.get_num_reachable()
        reached_c = self.get_num_reached()

        # print self
        # print "reachable: %s (%d/%d)" % (map(str, reachable), len(reachable), reachable_c)
        # print "reached: %s (%d/%d)" % (map(str, reached), len(reached), reached_c)


    @memoized
    def get_num_reachable(self):
        if self.function not in self.context.mutable_functions:
            return 0
        args = self.args + [self.value]
        reachable = self.get_reachable_goals()
        objects = [set(pe.get_matches()) for pe in args]
        object_pos = defaultdict(list)
        for f in reachable:
            fargs = list(f.all_args())
            for i, objs in enumerate(objects):
                if fargs[i] in objs:
                    object_pos[(fargs[i], i)].append(f)
        c = self.check_quantified_goal_count(reachable, args, object_pos)
        # print c
        return c

    @memoized
    def get_num_reached(self):
        args = self.args + [self.value]
        potential_goals = (self.get_goal_from_args(a) for a in self.get_matches())
        reached = [f for f in potential_goals if f in self.context.init]
        objects = [set(pe.get_matches()) for pe in args]
        object_pos = defaultdict(list)
        for f in reached:
            fargs = list(f.all_args())
            for i, objs in enumerate(objects):
                if fargs[i] in objs:
                    object_pos[(fargs[i], i)].append(f)
        c = self.check_quantified_goal_count(reached, args, object_pos)
        # print c
        return c

    def all_partitions(self):
        return self.args + [self.value]

    def __str__(self):
        argstr = [a.text() for a in self.args]
        valstr = self.value.text()
        # if isinstance(self, AlternativeFunctionGoal):
        #    print "Alternative valstr"

        # print "valstr: ", valstr
        if valstr in ("true", "false"):
            if valstr == "true":
                verb = "set"
            else:
                verb = "unset"
            return "%s %s of %s" % (verb, self.context.refs.get_name(self.function), ", ".join(argstr))
        return "set %s of %s to %s" % (self.context.refs.get_name(self.function), ", ".join(argstr), valstr)


class ActionGoal(GoalSpec):
    def __init__(self, context, action, effects, args, used_arguments, initial=False):
        GoalSpec.__init__(self, context)
        self.action = action #pddllib.pddl.actions.Action
        self.args = args
        self.current_arg_index = 0
        
        self.used_arguments = used_arguments

        self.effects = effects
        self.initial = initial

        self.cached_arg_matches = None
        self.cached_reachable_goals = None
        self.cached_num_reachable = None
        self.cached_num_reached = None
        self.children = None
        
#         logger2.debug(type(self.action))
#         logger2.debug([type(item) for item in self.args])
#         logger2.debug(map(str,  self.args))
#         logger2.debug([type(item) for item in self.used_arguments])
#         logger2.debug([type(item) for item in self.effects])
#         logger2.debug(type(self.initial))
# 
#         logger2.debug("init action goal: action: %s\n args: %s\n used_arguments: %s\n effects: %s\n initial: %s", map(str, self.action), 
#                      map(str, self.args),
#                      map(str, self.used_arguments),
#                      map(str, self.effects),
#                      self.initial)

    def set_current_arg_index(self, index):
        self.current_arg_index = index

    @staticmethod
    def initial_goals(context):
        logger.debug("\033[92m Init ActionGoals\033[0m")
        # print "context.refs.actions: ", map(str, context.refs.actions)
        # print "type actions :", [type(a) for a in context.refs.actions]
        goals = sum((ActionGoal.goals_from_action(context, a) for a in context.refs.actions), [])
        return [g for g in goals if not g.is_empty()]

    @classmethod
    def goals_from_action(cls, context, action):
        result = []
        effects = visitors.visit(action.effect, visitors.collect_effects, [])
#         logger2.debug("\033[92m action: %s\033[0m", map(str, effects))
        # print "\033[92m action: ", map(str, action), "\033[0m"
        args = set()
        for e in effects:
            args |= set(e.visit(visitors.collect_free_vars))
            # print "goals from action: ",  map(str, args)
        arg_partitions = []
        used_args = []

        for a in action.args:
            if a in args:
                p = partitions.Partition.get_initial_partition(a.type, context.refs).expand_unique()
                arg_partitions.append(p.children[0])
                # print "children: ", p.children[0].text()
                # print "arg_partition: ", map(str, arg_partitions)
                used_args.append(a)

        goal = cls(context, action, effects, arg_partitions, used_args, initial=True)
        return [goal]

        # for e in effects:
        #     args = set(e.visit(visitors.collect_free_vars))
        #     arg_partitions = []
        #     used_args = []
        #     for a in action.args:
        #         if a in args:
        #             p = partitions.Partition.get_initial_partition(a.type, context.refs).expand_unique()
        #             arg_partitions.append(p.children[0])
        #             used_args.append(a)
        #     result.append(cls(context, action, [e], arg_partitions, used_args, initial=True))
        # return result

    def get_reachable_objects(self, arg):
        """Return the set of objects that are potential values for the
        argument "arg"

        """
        # print "\n get reachable objects"
        result = set()
        index = self.args.index(arg)
        # print "index: ", index
        
        for args in self.get_matches():
#             if self.action.name == "grasp":
#                 logger.debug("matches for action %s: %s, effects: %s", self.action.name, map(str, args), map(str, self.get_effects_from_arg(args)))
            if all(self.context.rpg.is_relaxed_reachable(f) for f in self.get_effects_from_arg(args)):
#                 logger.debug("add: %s, %s, %d", self.action.name, map(str, args), index)
                result.add(args[index])
        return result


    def all_partitions(self):
        return [p for p in self.args]

    def get_arg(self, index):
        args = [a for a in self.args]
        # for i, arg in enumerate(args):
        #    if isinstance(arg, partitions.AlternativePartitionEntry):
        #        args[i] = arg.forbid_partiiton.expand_parent
        if isinstance(args[index], partitions.AlternativePartitionEntry):
            return args[index].forbid_partiiton.expand_parent
        return args[index]

    def arg_index(self, arg):
        args = [a for a in self.args]
        # print "arg_index: "
        # print map(str, self.args)
        return args.index(arg)

    def get_completed_args(self):
        for arg in self.args:
            if arg.is_leaf():
                yield arg
            else:
                return

    def get_current_arg(self):
        for arg in self.args:
#             logger2.debug("arg: %s", arg)
            if not arg.is_leaf():
#                 logger2.debug("found leaf at index: %d, %s, %s", self.arg_index(arg), arg, type(arg))
#                 logger2.debug("parent: %s", map(str, arg.partition.parent.get_references()) if arg.partition.parent else None)
#                 if arg.partition.parent:
#                     logger2.debug("diff: %s", map(str, arg.get_reference_diff(arg.partition.parent)))
                return arg
        return None

    def get_future_args(self):
        found_current = False
        for arg in self.args:
            if found_current:
                yield arg
            elif not arg.is_leaf():
                found_current = True


    def get_argument_indices(self, literal):
        """ Given an effect, return a list of of indices which correspond to the position
            of the literal's arguments in the global argument list"""

        _, fargs, _, _, value = state.StateVariable.svar_args_from_literal(literal)
#         logger2.debug(literal)
#         logger2.debug(value)
#         logger2.debug(map(str, self.used_arguments))
#        is_const = isinstance(value, pddl.ConstantTerm)
#         for a in chain(fargs, [value]):
#             if isinstance(value, pddl.ConstantTerm):
#                 logger2.debug("%s, %s, %s, %d", str(a.object), type(a.object), a.object.get_type(), any(a.object.get_type() == f.get_type() for f in self.used_arguments))
            
#         if str(literal) == "SimpleEffect: (assign FunctionTerm: contains(VariableTerm: ?o1 - drinkingvessel) Term: empty)":
#             exit(24)
#         logger2.debug([self.used_arguments.index(a.object) if a.object in self.used_arguments else -1 for a in chain(fargs, [value])])
#         result = []
#         for a in chain(fargs, [value]):
#             if a.object in self.used_arguments:
#                 result.append(self.used_arguments.index(a.object))
#             elif is_const:
#                 for k, f in enumerate(self.used_arguments, 0):
#                     if a.object.get_type() == f.get_type():
#                         result.append(k)
#                 result.append(-1)
#                 
#         return result
        #print "used:"
        #for u in self.used_arguments:
        #    print "--", u
        #for a in chain(fargs, [value]):
        #    print "---", a.object
        #    if a.object in self.used_arguments:
        #        print "---", self.used_arguments.index(a.object)    
        #    else:
        #        print "---", -1
        return [self.used_arguments.index(a.object) if a.object in self.used_arguments else -1 for a in chain(fargs, [value])]


    def get_arguments_from_effects(self, goal_tup):
        """ Given a list of goals (as facts), return a tuple of pddl objects that cause these effects
        """

        result = [None] * len(self.args)
        for f, e in zip(goal_tup, self.effects):
            indices = self.get_argument_indices(e)
            objects = chain(f.svar.args, [f.value])
            for i, obj in zip(indices, objects):
                assert result[i] is None or result[i] == obj
                result[i] = obj
        return tuple(result)

    def get_partitions_for_effect(self, effect):
        """ Return the references that are relevant for the given effect """
        # action_args = list(self.action.args)
        # mapping = {action_args[self.argument_index[i]] : i for i in xrange(len(self.args))}

        return [self.args[i] for i in self.get_argument_indices(effect)]

    def allow_universal_expansion(self, partition):
        return all(partition != self.get_partitions_for_effect(e)[-1] for e in self.effects)

    def _next(self):
        """Overridden method that returns this goal's children. """
        logger.debug("\033[92m Overridden expand: %s\033[0m", self)
        # traceback.print_stack()
        prefix = list(self.get_completed_args())
        expanded_arg = self.get_current_arg()
        logger.debug("expanded_arg: %s type: %s", expanded_arg, type(expanded_arg))
        if not expanded_arg:
            # either we're done or no goal exists
            num_reachable = self.get_num_reachable()
            if self.is_reachable(num_reachable) and not self.is_reached(num_reachable):
                return [self]
            return []

        expansions = expanded_arg.expand(allow_universal=self.allow_universal_expansion(expanded_arg))
        suffix = list(self.get_future_args())
#         logger2.debug("expansion: %s", map(str, suffix))
        result = []
        reachable_objects = self.get_reachable_objects(expanded_arg)
        len_expansions = len(expansions)
        
        for entry in expansions:
            if entry.is_empty():
                continue
            # print "Entry:", entry
            # print "matches: ", map(str, entry.get_matches())

            # print "entry: ", entry
            # print "set reachable:", entry, id(entry), map(str, self.get_reachable_objects(expanded_arg))
            entry.set_reachable(reachable_objects)
            # print "matches reachable: ", map(str, entry.get_matches())
            if len_expansions > 1 and entry.is_quantified() and entry.is_unique():
#                logger2.debug("Skipped Entry: %s", entry)
                continue
           

            new_args = prefix + [entry] + [e.clone() if e is not None else None for e in suffix]
            # print "New_args: prefix: ",  map(str, prefix), " + entry: ", str(entry), " + suffix: ", map(str, suffix)
            # print "For expansion: ", str(entry), " type ", type(entry)
            # print "New_args: prefix: ",  map(str, prefix), " + entry: ", str(entry), " + suffix: ", map(str, suffix)
            # print "New args: ", map(str, new_args)
            if isinstance(entry, partitions.AlternativePartitionEntry):
                # print "Alternative Goal:", map(str, new_args)

                # new_args = [[entry][0]]
                # print "self.args " ,map(str, self.args) ," vs ", map(str, new_args)
                g = self.get_alternative_child(new_args, entry.forbid_partiiton)
                g.fix_score = -999999
            else:
                # print "type new_args: ", [type(a) for a in new_args]

                g = self.get_child(new_args)
                # print "New Goal: ", str(g)
                # print "args:", map(str,g.args[0].get_matches())
                
#                 logger2.debug("%s", entry)
                if isinstance(entry, partitions.ExistentialPartitionEntry):
                    g.fix_score = -300000
                elif any(isinstance(r, OtherReference) for r in entry.get_references()) and all(not isinstance(r, RelationReference) for r in entry.get_references()):
                    g.fix_score = -200000

            # print "with args: ", map(str,new_args)
            # print "Is reachable: ",g.is_reachable()
            # pdb.set_trace()
            num_reachable = g.get_num_reachable()
            is_reachable = g.is_reachable(num_reachable)
            is_reached = g.is_reached(num_reachable) if is_reachable else False
            if is_reachable and not is_reached:
                # print "Add goal: ", str(g)
                # print "new Args: ", str(self.get_child(new_args))
#                 logger2.debug("Can reach: %s", g)
                result.append(g)
#             else:
#                 logger2.debug("Can't reach: %s, entry: %s, is_reachable: %d, is_reached: %d", g, entry, is_reachable, is_reached)
        
        logger.debug("new Goals: %s", map(str, result))
        if len(result) == 1:
            return result[0].next()
        return result

    def _allnext(self):
        """Overridden method that returns all this goal's children. """
        logger.debug("\033[92m Overridden all expand: %s \033[0m", self)
        # traceback.print_stack()
        prefix = list(self.get_completed_args())
        expanded_arg = self.get_current_arg()
        # print "expanded_arg: ", expanded_arg, " type: ",type(expanded_arg)
        if not expanded_arg:
            # either we're done or no goal exists
            if self.is_reachable() and not self.is_reached():
                return [self]
            return []

        expansions = expanded_arg.expandall(allow_universal=self.allow_universal_expansion(expanded_arg))
        logger.debug("expansion:  %s", map(str, expansions))
        logger.debug("\n--------------------------------------------------------------------------\n")
        suffix = list(self.get_future_args())

        result = []
        for entry in expansions:
            if len(entry) == 0:
                continue
            logger.debug("entr in best.children)y: %s %s", entry, type(entry))
            # print "set reachable:", entry, id(entry), map(str, self.get_reachable_objects(expanded_arg))
            entry.set_reachable(self.get_reachable_objects(expanded_arg))

            if entry.is_quantified() and entry.is_unique() and len(expansions) > 1:
                continue

            new_args = prefix + [entry] + [e.clone() if e is not None else None for e in suffix]
            # print "New_args: prefix: ",  map(str, prefix), " + entry: ", str(entry), " + suffix: ", map(str, suffix)
            # print "For expansion: ", str(entry), " type ", type(entry)
            # print "New_args: prefix: ",  map(str, prefix), " + entry: ", str(entry), " + suffix: ", map(str, suffix)
            # print "New args: ", map(str, new_args)
            if isinstance(entry, partitions.AlternativePartitionEntry):
                logger.debug("Alternative Goal: %s", map(str, new_args))
                input("Alternative")
                # new_args = [[entry][0]]
                # print "self.args " ,map(str, self.args) ," vs ", map(str, new_args)
                g = self.get_alternative_child(new_args, entry.forbid_partiiton)
                g.fix_score = -999999
            else:
                # print "type new_args: ", [type(a) for a in new_args]
                g = self.get_child(new_args)
            # print "G: ", str(g)
            # print "with args: ", map(str,new_args)
            # print "Is reachable: ",g.is_reachable()
            if g.is_reachable() and not g.is_reached():
                # print "Add goal: ", str(g)
                # print "new Args: ", str(self.get_child(new_args))
                result.append(g)
            else:
                logger.debug("Can't reach: %s", g)
        logger.debug("new Goals:  %s", map (str, result))
        if len(result) == 1:
            return result[0].next()
        return result

    def get_child(self, new_args):
        # print "\nNew  Child"
        # print "action: ", map(str, self.action)
        # print "effects: ", map(str, self.effects)
        # print "new_args: ", map(str, new_args)
        # if len(new_args)>1:
        #    print "matches from [",new_args[0],"]: ", map(str, new_args[0].get_matches())
        if isinstance(self, AlternativeActionGoal):
            return ActionGoal(self.context, self.action, self.effects, new_args, self.used_arguments)
        if isinstance(self, AlternativeFunctionGoal):
            return FunctionGoal(self.context, self.action, self.effects, new_args, self.used_arguments)
        return self.__class__(self.context, self.action, self.effects, new_args, self.used_arguments)


    # def get_child(self, new_args):
    #    return self.__class__(self.context, self.action, self.effects, new_args, self.used_arguments)


    # def get_child(self, new_args):
    #    return self.__class__(self.context, self.action, self.effects, new_args, self.used_arguments)

    def get_alternative_child(self, new_args, forbid_partiitons):
        # new_args = new_args[:1] + new_args[-2:]
        # print "\nNew Alternative Child"
        logger.debug("action:  %s", map(str, self.action))
        logger.debug("effects:  %s", map(str, self.effects))
        logger.debug("new_args:  %s", map(str, new_args))
        return AlternativeActionGoal(self.context, self.action, self.effects, new_args, self.used_arguments, forbid_partiitons)

    def extract_child(self, goal):
        prefix = []
        suffix = []
        entry = child_entry = None
        for pe, pe2 in chain(zip(self.args, goal.args)):
            if pe != pe2:
                entry = pe
                child_entry = pe2
            elif entry is None:
                prefix.append(pe)
            else:
                suffix.append(pe)

        if entry is None:
            return None

        new_partition = child_entry.partition.split_partition(child_entry)
        if not new_partition:
            return None

        result = []
        for pentry in new_partition.children:
            all_partitions = prefix + [pentry] + suffix
            result.append(self.get_child(prefix + [pentry] + suffix))
        return result

    def get_matches(self):
        if self.cached_arg_matches is not None:
            return self.cached_arg_matches

        self.cached_arg_matches = []
        # print "self.args: ", map(type, self.args)
        arg_list = [a.get_matches() if a is not None else [None] for a in self.args]
        # print "arg_list matches: ", [map(str, a) for a in arg_list]
        # zerolencount = len([a for a in arg_list if len(a)>0])
        if any(len(l) == 0 for l in arg_list):
        # if zerolencount == 0:
            # print "len 0: "
            # print [map(str, a) for a in arg_list]
            return []

        self.cached_arg_matches = list(product(*arg_list))
        return self.cached_arg_matches

    def get_effects_from_arg(self, args):
        """ given an argument list of pddl objects, return the goals that would result from
            instantiating the action with these arguments"""
 
        def replace_variables(effect, new_arguments):
            function, args, _, _, value, negated = pddl.utils.get_literal_elements(effect)
            assert not negated
            replacement_args = [new if new is not None else old.object  for old, new in zip(args, new_arguments[:-1])]
#             logger2.debug("%s", map(str, new_arguments))
            replacement_value = new_arguments[-1] if new_arguments[-1] is not None else value.object
            return state.Fact(state.StateVariable(function, replacement_args), replacement_value)
 
        goal = []
        for e in self.effects:
            indices = self.get_argument_indices(e)
            fact = replace_variables(e, [args[i] if i >= 0 else None for i in indices])
            goal.append(fact)
#         logger2.debug(map(str, goal))
        return tuple(goal)

    def get_reachable_goals(self):
        if not self.cached_reachable_goals:
            potential_goals = (self.get_effects_from_arg(a) for a in self.get_matches())
#             logger2.debug(type(potential_goals))
#             logger2.debug(map(str, potential_goals))
#             potential_goals = (self.get_effects_from_arg(a) for a in self.get_matches())
            self.cached_reachable_goals = [g for g in potential_goals if all(self.context.rpg.is_relaxed_reachable(f) for f in g)]
        return self.cached_reachable_goals


    def pddl_goal(self):
        """Return matching goals that are reachable but not yet reached as a PDDL goal formula"""

        # TODO: Quantified goals
        pddlgoals = []
        for goal in self.get_reachable_goals():
            if any(f not in self.context.init for f in goal):
                if len(goal) == 1:
                    pddlgoals.append(goal[0].to_condition())
                else:
                    pddlgoals.append(pddl.conditions.Conjunction([f.to_condition() for f in goal]))

        return pddl.conditions.Disjunction(pddlgoals)

    @memoized
    def get_num_reachable(self):
        if not self.cached_num_reachable:
            reachable = self.get_reachable_goals()
            objects = [set(pe.get_matches()) if pe is not None else set() for pe in self.args]
            object_pos = defaultdict(list)
    
    #         logger2.debug(map(lambda x: map(str, x), reachable))
    #         logger2.debug(map(lambda x: map(str, x), objects))
    
            # print map(lambda x: map(str, x), reachable)
            # print map(lambda x: map(str, x), objects)
            # print self, map(str, self.args)
            
            for g in reachable:
                for f, e in zip(g, self.effects):
                    ei = self.get_argument_indices(e)
#                     added = False
                    # print f, map(str, f.all_args())
                    for i, a in enumerate(f.all_args()):
                        arg_i = ei[i]
                        # print a, arg_i, i, map(str,objects[arg_i])
                        if a in objects[arg_i]:
                            # print "*"
                            object_pos[(a, arg_i)].append(g)
#                             added = True
    #                 if added:  # FIXME: is this correct?
    #                     break
    #         logger2.debug([k.name + " - " + str(v) for k,v in object_pos])
            # print "call: ", map(str,reachable), map(str,self.args), map(str,object_pos)
            self.cached_num_reachable = self.check_quantified_goal_count(reachable, self.args, object_pos)
        # print c
        return self.cached_num_reachable

    @memoized
    def get_num_reached(self):
        if not self.cached_num_reached:
            potential_goals = (self.get_effects_from_arg(a) for a in self.get_matches())
            reached = [g for g in potential_goals if all(f in self.context.init for f in g)]
    
            objects = [set(pe.get_matches()) if pe is not None else set() for pe in self.args]
            object_pos = defaultdict(list)
    
    #         l = logging.getLogger("goal.reachability")
    # 
    #         l.debug(str(map(lambda x: map(str, x), reached)))
    #         l.debug(str(map(lambda x: map(str, x), objects)))
    
            for g in reached:
                for f, e in zip(g, self.effects):
                    ei = self.get_argument_indices(e)
    #                 l.debug("%s %s", f, ei)
                    added = False
                    for i, a in enumerate(f.all_args()):
                        arg_i = ei[i]
    #                     l.debug("(%s,%d): %s", a, arg_i, map(str, objects[arg_i]))
                        if a in objects[arg_i]:
    #                         l.debug("(%s,%d): %s", a, arg_i, map(lambda x: map(str, x), reached))
                            object_pos[(a, arg_i)].append(g)
                            added = True
                    if added:  # FIXME: is this correct?
                        break
            self.cached_num_reached = self.check_quantified_goal_count(reached, self.args, object_pos)
        return self.cached_num_reached

    def __str__(self):
        argstr = [a.text() for a in self.args if a is not None]
        return "%s %s" % (self.context.refs.get_name(self.action.name), ", ".join(argstr))

class AlternativeActionGoal(ActionGoal):
    def __init__(self, context, action, effects, args, used_arguments, forbid_partiiton, initial=False):
        super(self.__class__, self).__init__(context, action, effects, args, used_arguments, initial)
        self.forbid_partiiton = forbid_partiiton

class AlternativeFunctionGoal(FunctionGoal):
    def __init__(self, context, function, args, value, forbid_partiiton, initial=False):
        super(self.__class__, self).__init__(context, function, args, value, initial)
        self.forbid_partiiton = forbid_partiiton

class ExecActionGoal(ActionGoal):
    """ Goal type that only allows selection of currently executable actions """

    def __init__(self, context, action, effects, args, used_arguments, initial=False):
        ActionGoal.__init__(self, context, action, effects, args, used_arguments, initial)
        self.preconditions = visitors.visit(action.precondition, visitors.collect_conditions, [])

    @staticmethod
    def initial_goals(context):
        goals = sum((ExecActionGoal.goals_from_action(context, a) for a in context.refs.actions), [])
        return [g for g in goals if not g.is_empty()]

    def allow_universal_expansion(self, partition):
        # universal quantifiers make no sense for concrete actions
        return False

    def filter_preconditions(self, args):
        # build list of action arguments that match the current (partial) goal
        action_args = []
        for aarg in self.action.args:
            if aarg in self.used_arguments:
                i = self.used_arguments.index(aarg)
                objects = set(tup[i] for tup in args)
                # print self.args[i], map(str, objects)
            else:
                objects = list(self.context.problem.get_all_objects(aarg.type))

            action_args.append(objects)

        result = [set() for i in xrange(len(self.args))]
        inst_function = self.action.get_inst_func(self.context.init)
        for mapping in self.action.smart_instantiate(inst_function, self.action.args, action_args, self.context.problem):
            for i, aarg in enumerate(self.used_arguments):
                # print i, aarg, mapping[aarg]
                result[i].add(mapping[aarg])

        # print map(lambda x: map(str,x), result)
        return result

    def get_reachable_objects(self, arg):
        """Return the set of objects that are potential values for the
        partition entry "arg"

        """
        result = set()
        index = self.args.index(arg)

        for m in self.get_matches():
             # print map(str, m)
             assert all (arg.matches(o) for o, arg in zip(m, self.args))

        filtered_matches = self.filter_preconditions(self.get_matches())
        return filtered_matches[index]


class GoalContext(object):
    def __init__(self, problem, refs):
        self.problem = problem
        self.refs = refs #ReferenceList
        self.mutable_functions = set()

        self.init = state.State.from_problem(problem)
        self.rpg = relaxed_exploration.Plangraph(problem)
        self.objects = set(o for o in chain(self.problem.objects, self.problem.domain.constants) if o not in (pddl.UNKNOWN,))
        self.calculate_reachability()

    def calculate_reachability(self):
        for a in self.problem.domain.actions:
            for eff in visitors.visit(a.effect, visitors.collect_effects, []):
                self.mutable_functions.add(pddl.utils.get_function(eff))

    def get_goal_score(self, goal):
#         if isinstance(self.refs[-1], OtherReference):
#             goal.fix_score = -299999
        if goal.fix_score:
            return goal.fix_score
        if goal.get_num_reachable() == 0:
            return 0
            
        reachable = goal.get_reachable_goals()
        num_reachable = goal.get_num_reachable()
        num_reached = goal.get_num_reached()
        total_relaxed_dist = sum(self.rpg.get_ff_distance(f) for f in reachable)
        avg_cost = (float(total_relaxed_dist) + 1) / num_reachable
        # print num_reachable, avg_cost
        return (num_reachable - num_reached) / avg_cost ** 0.5
