import time
import heapq

from pddllib import pddl
from pddllib.pddl import conditions, visitors
from pddllib.state import state
from itertools import chain, product
from collections import defaultdict

import cloudpickle

import logging, constants
logger = logging.getLogger('root')
logger2 = logging.getLogger('highlighted')

class UnaryOp(object):
    def __init__(self, action, args, conditions, effect, costs):
        self.action = action
        self.args = args
        self.conditions = conditions
        self.effect = effect
        self.op_costs = costs
        self.reset()

    def __str__(self):
        return "%s %s: cost %d, %d remaining" % (self.action.name, " ".join(a.name for a in self.args), self.h_add_costs, self.num_conditions)
        
    def reset(self):
        self.num_conditions = len(self.conditions)
        self.h_add_costs = self.op_costs

    def set_conditions(self, new_init):
        self.max_conditions = len([c for c in self.conditions if c not in new_init])
        self.reset()

    @staticmethod
    def from_action(action, args):
        result = []
        
        cond = visitors.visit(action.precondition, visitors.collect_conditions, [])
        conditions = [state.Fact.from_literal(c) for c in cond if pddl.utils.get_function(c) != pddl.builtin.equals ]
        costs = action.get_total_cost()
        costs = costs.value if costs is not None else 1
        for eff in visitors.visit(action.effect, visitors.collect_effects, []):
            e = state.Fact.from_literal(eff)
            result.append(UnaryOp(action, args, conditions, e, costs))
        return result

class Proposition(object):
    def __init__(self, fact, children):
        self.fact = fact
        self.children = children
        self.reset()

    def __str__(self):
        return "%d: %s" % (self.costs, self.fact)

    def reset(self):
        self.costs = -1
        self.has_fired = False
        self.reached_by = None

    def fire(self):
        for op in self.children:
            # print op
            op.h_add_costs += self.costs
            op.num_conditions -= 1
            assert op.num_conditions >= 0
            if op.num_conditions == 0:
                yield op
        self.has_fired = True
        

def get_inst_func(action, reachable, init, static):
    from pddllib.pddl.functions import FunctionTerm, FunctionVariableTerm, VariableTerm
    from pddllib.pddl import builtin

    def args_visitor(term, results):
        if isinstance(term, FunctionTerm):
            return sum(results, [])
        return [term]

    cond_by_arg = defaultdict(set)
    free_args = {}

    def subcond_visitor(cond, result):
        for arg in cond.free():
            cond_by_arg[arg].add(cond)
        if isinstance(cond, conditions.LiteralCondition):
            free_args[cond] = cond.free()

    condition = action.precondition

    visitors.visit(condition, subcond_visitor)

    prev_mapping = {}
    checked = set()

    def inst_func(mapping, args):
        next_candidates = []
        if checked:
            for k,v in prev_mapping.iteritems():
                if mapping.get(k, None) != v:
                    checked.difference_update(cond_by_arg[k])
        prev_mapping.update(mapping)

        def instantianteAndCheck(cond, combinations, func):
            for c in combinations:
                with cond.instantiate(dict(zip(cond.args, c)), init.problem):
                    result = func()
                yield result

        #print [a.name for a in mapping.iterkeys()]
        forced = []
        def check(cond):
            def is_instantiated_function(term):
                return type(term) == FunctionTerm or (type(term) == FunctionVariableTerm and term.is_instantiated())
            if not cond or cond in checked:
                return True
            if isinstance(cond, conditions.LiteralCondition):
                if cond.predicate == builtin.equals and is_instantiated_function(cond.args[0]) and isinstance(cond.args[1], VariableTerm):
                    v = cond.args[-1]
                    if all(a.is_instantiated() for a in cond.args[0].args if isinstance(a, VariableTerm)) and  isinstance(v, VariableTerm) and not v.is_instantiated():
                        f = pddl.utils.get_function(cond)
                        if f in static:
                            # propagate only static relations
                            svar = state.StateVariable.from_literal(cond, init)
                            forced.append((v.object, init[svar], cond))
                if all(a.is_instantiated() for a in free_args[cond]):
                    if (pddl.utils.get_function(cond) == pddl.builtin.equals):
                        #logger2.debug("c1: %s", cond.args[0].get_instance())
                        #logger2.debug("c2: %s", cond.args[1].get_instance())
                        if (cond.args[0].get_instance() == cond.args[1].get_instance()) ^ cond.negated:
                            #logger2.debug(str(cond))
                            # print cond.pddl_str(), "passed"
                            checked.add(cond)
                            return True
                        # print cond.pddl_str(), "failed", cond.args[0], cond.args[1]
                        return False
                    #FIXME: this will break nested functions
                    fact = state.Fact.from_literal(cond, init)
                    #FIXME: does not support axioms at this point
                    # exst = init.get_extended_state([fact.svar])
                    # print "checking ",fact
                    if isinstance(fact.svar.function, pddl.Predicate) and fact.value == pddl.FALSE:
                        #FIXME: handle negative preconditions that are true in the initial state
                        pass
                    elif fact not in reachable:
                        # print "not in reachable"
                        return False
                    checked.add(cond)
#                     logger2.debug("called: %s", fact)
                    return True
                else:
                    next_candidates.append([a for a in free_args[cond] if not a.is_instantiated()])
            elif isinstance(cond, conditions.Conjunction):
                results = [check(c) for c in cond.parts]
                #logger2.debug(map(str, cond.parts))
                #logger2.debug(map(str, results))
                if any(c == False for c in results):
                    return False
                if all(c == True for c in results):
                    checked.add(cond)
                    return True
            elif isinstance(cond, conditions.Disjunction):
                results = [check(c) for c in cond.parts]
                if any(c == True for c in results):
                    checked.add(cond)
                    return True
                if all(c == False for c in results):
                    return False
            elif isinstance(cond, conditions.QuantifiedCondition):
                combinations = product(*map(lambda a: list(init.problem.get_all_objects(a.type)), cond.args))
                results = list(instantianteAndCheck(cond, combinations, lambda: check(cond.condition)))
                if isinstance(cond, conditions.Conjunction):
                    if any(c == False for c in results):
                        return False
                    if all(c == True for c in results):
                        checked.add(cond)
                        return True
                elif isinstance(cond, conditions.ExistentialCondition):
                    if any(c == True for c in results):
                        checked.add(cond)
                        return True
                    if all(c == False for c in results):
                        return False
            else:
                assert False
            return None

        result = check(condition)
#         if (action.name == "approach"):
#             logger2.debug("%s, %s", map(str, condition.parts), result)
        if result == True:
            checked.add(condition)
            # print action.name, "accept:", [a.get_instance().name for a in  args]
            return True, None
        elif result == False:
            # print action.name, "reject:", [a.get_instance().name for a in  args]
            return None, None

        if forced:
            svar, val, lit = forced[0]
            checked.add(lit)
            # print "Forced %s = %s" % (str(svar), val.name)
            return (svar, val)
        if next_candidates:
            next_candidates = sorted(next_candidates, key=lambda l: len(l))
            # print "Next:", next_candidates[0][0]
            return next_candidates[0][0], None
        # print self, [a.get_instance().name for a in  args]
        return True, None
    return inst_func

class Plangraph(object):
    def __init__(self, problem):
        self.problem = problem
        self.domain = problem.domain
        self.init = state.State.from_problem(problem)

        self.compute_action_preconds()
        self.compute_reachable()
        self.compute_relaxed_distance()
        
        logger.debug("plangraph init: %s", map(str, self.init))

    def compute_action_preconds(self):
        self.precond_lookup = {}
        for a in self.domain.actions:
            self.precond_lookup[a] = {}
            cond = visitors.visit(a.precondition, visitors.collect_conditions, [])
            for c in cond:
                if (pddl.utils.get_function(c) == pddl.builtin.equals):
                    continue

                f, args, _, _, val = state.StateVariable.svar_args_from_literal(c)
                mapping = []
                for arg in chain(args, [val]):
                    if isinstance(arg, pddl.ConstantTerm):
                        mapping.append(arg.object)
                    else:
                        mapping.append(a.args.index(arg.object))
                self.precond_lookup[a][f] = mapping

    def compute_reachable(self):
        logger2.debug("Compute Reachable")
        static_functions = set(chain(self.domain.functions, self.domain.predicates))
        for a in self.domain.actions:
            for eff in visitors.visit(a.effect, visitors.collect_effects, []):
                static_functions.discard(pddl.utils.get_function(eff))

        triggered_actions = {a : set() for a in self.domain.actions}

        def trigger_action(f):
            for a in self.domain.actions:
                if f.svar.function not in self.precond_lookup[a]:
                    continue
                mapping = self.precond_lookup[a][f.svar.function]
                fargs = f.all_args()
                mismatch = False
                objects = [None]*len(a.args)
                for fa, m in zip(fargs, mapping):
                    if isinstance(m, pddl.TypedObject):
                        if fa != m:
                            mismatch = True
                            break
                    else:
                        objects[m] = fa
                        
                if not mismatch:
                    triggered_actions[a].add(tuple(objects))

        prev_triggered = None

        def get_triggers(action):
            if prev_triggered is None:
                return [(None,)*len(action.args)]
            return prev_triggered[action]

        self.props = {}
        self.ops = []
        reached_actions = set()
        reachable = set(self.init.iterfacts())
        for f in reachable:
            self.props[f] = Proposition(f, [])
        
        changed = True
        t0 = time.time()
        while changed:
#             logger2.debug("reachable: %s", map(str, reachable))
            t1 = time.time()
            changed = False
        
            for action in self.domain.actions:
#                 logger2.debug(action.name)
                inst_func = get_inst_func(action, reachable, self.init, static_functions)
                for trigger in get_triggers(action):
                    all_args = [list(self.problem.get_all_objects(a.type)) if ta is None else [ta] for a, ta in zip(action.args, trigger)]
#                     logger2.debug("- %s, %s, %s", action.name, map(lambda x: map(str,x), all_args), map(str, action.args))
                    for m in action.smart_instantiate(inst_func, action.args, all_args, self.problem):
                        args = tuple(m[a] for a in action.args)
                        key = (action.name, args) 
#                         if action.name == "approach":
#                             logger2.debug("%s, %s, %d", key[0], map(str, key[1]), key not in reached_actions)
#                             for x, y in reached_actions:
#                                 logger2.debug("-- %s, %s", x, map(str, y))
                        if key not in reached_actions:
                            reached_actions.add(key)
#                             if action.name == "approach":
#                                 logger2.debug("%s, %s", action.name, map(str, args))
#                                 logger2.debug(map(str, UnaryOp.from_action(action, args)))
                            for op in UnaryOp.from_action(action, args):
                                self.ops.append(op)
#                                 logger2.debug("check: %s, %s", action.name, op.effect)
                                if op.effect not in reachable:
#                                     logger2.debug("add: %s, %s", action.name, op.effect)
                                    # print "**", op.effect
                                    changed = True
                                    # trigger_action(op.effect)
                                    reachable.add(op.effect)
                                    self.props[op.effect] = Proposition(op.effect, [])
                                op.effect = self.props[op.effect]

                                for c in op.conditions:
                                    if c not in self.props:
                                        assert isinstance(c.svar.function, pddl.Predicate) and c.value == pddl.FALSE
                                        self.props[c] = Proposition(c, [])

                                    self.props[c].children.append(op)
            # prev_triggered = triggered_actions
            # triggered_actions = {a : set() for a in self.domain.actions}
            
            logger.debug("iteration: %f", (time.time()-t1))

        logger2.debug("exploration: %f", (time.time()-t0))
        logger2.debug("num reachable: %f", (len(reachable) - len(self.init)))

        self.facts = list(self.props.itervalues())
        self.reachable = reachable

    def compute_relaxed_distance(self):
        t0 = time.time()
        for op in self.ops:
            op.reset()

        queue = []
        for prop in self.facts:
            prop.reset()
            if prop.fact in self.init:
                # print prop, map(str, prop.children)
                prop.costs = 0
                heapq.heappush(queue, (0, prop))

        while queue:
            _, prop = heapq.heappop(queue)
            if prop.has_fired:
                continue

            # print prop
            for op in prop.fire():
                # print "fire:", op
                if op.effect.costs == -1 or op.h_add_costs < op.effect.costs:
                    op.effect.costs = op.h_add_costs
                    op.effect.reached_by = op
                    heapq.heappush(queue, (op.h_add_costs, op.effect))
                    
        logger.debug("h_add: %f", (time.time()-t0))

    def is_relaxed_reachable(self, fact):
        return fact in self.init or fact in self.props

    def get_h_add_distance(self, fact):
        return self.props[fact].costs
        
    def get_ff_distance(self, facts):
        def collect_relaxed_plan(prop, plan):
            op = prop.reached_by
            if op is not None:
                for c in op.conditions:
                    collect_relaxed_plan(self.props[c], plan)
                plan.add(op)

        plan = set()
        if isinstance(facts, state.Fact):
            facts = [facts]
        for f in facts:
            if f not in self.init:
                collect_relaxed_plan(self.props[f], plan)
        return sum(op.op_costs for op in plan)
            
            
