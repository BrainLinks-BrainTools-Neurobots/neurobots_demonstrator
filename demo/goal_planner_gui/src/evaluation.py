import sys, time
from os import path

from collections import defaultdict
from itertools import product

from pddllib import pddl
from pddllib.state import state
from pddllib.pddl import visitors

import goals, references


def get_goals(prob):
    cond = visitors.visit(prob.goal, visitors.collect_conditions, [])
    init = state.State.from_problem(prob)
    for c in cond:
        f = state.Fact.from_literal(c)
        if f not in init:
            yield f

def simple_goal_match(goals, context):
    goal_per_function = defaultdict(set)
    for g in goals:
        goal_per_function[g.svar.function].add(g)

    def get_args(f):
        args = []
        for i, t in enumerate([a.type for a in f.args] + [f.type]):
            if t == pddl.t_boolean:
                args.append([pddl.TRUE, pddl.FALSE])
            elif t not in context.refs.types:
                args.append(list(context.problem.get_all_objects(t)))
            else:
                args.append(list(set(list(g.all_args())[i] for g in goal_per_function[f])))
        return args

    all_goals = []
    functions = set(goal_per_function.iterkeys())
    for f in functions:
        all_args = get_args(f)
            
        for args in product(*all_args):
            val = args[-1]
            args = args[:-1]
            fact = state.Fact(state.StateVariable(f, args), val)
            if fact not in context.init and context.rpg.is_relaxed_reachable(fact):
                all_goals.append(fact)
            
    return all_goals
        

def evaluate(context, args):
    t0 = time.time()
    explorer = Explorer(context)
    goals = set(get_goals(context.problem))
    print map(str, goals)
    baseline_goals = simple_goal_match(goals, context)
    found = explorer.explore(goals)
    print "found: %d of %d:" % (len(found), len(goals))
    for pg, g in explorer.found_at.iteritems():
        print "   (%d - %d - %d)" % (len(g.get_relevant()), explorer.found_at_depth[pg], g.ref_count()), pg,":",g
    all_selected = set()
    for g in explorer.found_at.itervalues():
        all_selected |= g.get_relevant()
        
    total_depth = sum(explorer.found_at_depth.itervalues())
    total_refs = sum(g.ref_count() for g in  explorer.found_at.itervalues())
    max_depth = max(explorer.found_at_depth.itervalues())
    max_refs = max(g.ref_count() for g in  explorer.found_at.itervalues())
    avg_depth = float(total_depth)/len(explorer.found_at_depth)
    avg_refs = float(total_refs)/len(explorer.found_at_depth)
    print "had to select %d additional goals (or %.1f as many)" % (len(all_selected) - len(goals), float(len(all_selected))/len(goals))
    print "had to search to max depth %d, avarage %.1f" % (max_depth, avg_depth)
    print "maximum number of references %d, avarage %.1f" % (max_refs, avg_refs)
    print "baseline #goals:", len(baseline_goals)
    t1 = time.time() - t0
    output_result(explorer, args, goals, t1)
    # print map(str, baseline_goals)

def output_result(explorer, args, goals, time):
    problem = explorer.context.problem
    config = explorer.context.refs.config
    pname = path.basename(args.problem)
    fname = path.join("eval", "%s-%s-%s.result" % (problem.domain.name, pname, config))

    all_selected = set()
    for g in explorer.found_at.itervalues():
        all_selected |= g.get_relevant()
        
    d = {}
    d['domain'] = problem.domain.name
    d['problem'] = problem.name
    d['config'] = config
    d['time'] = time
    d['pddlgoal'] = [str(pg) for pg in goals]
    d['allcount'] = len(all_selected)
    d['found'] = [str(pg) for pg in explorer.found_at.itervalues()]
    details = []
    for pg, g in explorer.found_at.iteritems():
        e = {}
        baseline = simple_goal_match([pg], explorer.context)
        e['pddlgoal'] = str(pg)
        e['goal'] = str(g)
        e['allcount'] = len(g.get_relevant())
        e['depth'] = explorer.found_at_depth[pg]
        e['refs'] = g.ref_count()
        e['baseline'] = len(baseline)
        details.append(e)

    d['details'] = details
    total_baseline = simple_goal_match(goals, explorer.context)
    d['baseline'] = len(total_baseline)

    f = open(fname, mode='w')
    print >>f,repr(d)
    print repr(d)
    f.close()
                      

class Explorer(object):
    def __init__(self, context):
        self.context = context

    def explore(self, pddl_goals):
        functions = set(g.svar.function for g in pddl_goals)
        # goals = self.context.new_goal().next()
        current_goals = [goals.FunctionGoal.goal_from_function(self.context, f) for f in functions]
        self.found_at = {}
        self.found_at_depth = {}
        self.found_exactly = set()

        found = set()
        for g in current_goals:
            found.update(self.explore_tree(g, pddl_goals))
        return found


    def explore_tree(self, goal, pddl_goals, depth = 1):
        if goal.is_empty():
            return []

        matches = set(goal.get_relevant())
        if not matches & pddl_goals:
            return []

        if goal.is_unique():
            found = pddl_goals & matches
            for g in found:
                if g not in self.found_at or len(self.found_at[g].get_relevant()) > len(matches):
                    self.found_at[g] = goal
                    self.found_at_depth[g] = depth
            print "|","  " * depth, "match:", goal, map(str,matches)
            if len(found) == len(matches):
                self.found_exactly |= found
            return found

        found = set()
        next_goals = goal.next_flattened()
        deferred = []
        for g in next_goals:
            print "|","  " * depth, g
            if g.is_quantified():
                deferred.append(g)
            else:
                found.update(self.explore_tree(g, pddl_goals, depth+1))
                if found:
                    pddl_goals = pddl_goals.difference(self.found_exactly)

        for g in deferred:
            found.update(self.explore_tree(g, pddl_goals, depth+1))
            if found:
                pddl_goals = pddl_goals.difference(self.found_exactly)
                    
        return found


