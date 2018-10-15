import time
import math, itertools

from itertools import chain, product
from collections import defaultdict

from pddllib import pddl
from pddllib.state import state

import constants

import logging
logger = logging.getLogger('root')
logger2 = logging.getLogger('highlighted')

class Reference(object):
    def __str__(self):
        return self.text(pddl.UNKNOWN)

    def ref_count(self):
        return 1

    def __ne__(self, other):
        return not self.__eq__(other)

    def semantic_equal(self, other):
        return False


class IdentityReference(Reference):
    def __init__(self, obj):
        self.object = obj
        
    def text(self, obj):
        return self.object.name

    def matches(self, o):
        return self.object == o

    def __hash__(self):
        return hash(self.object)

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.object == other.object


class OtherReference(Reference):
    def __init__(self, excluding_refs):
        self.excluding = tuple(excluding_refs)

    def text(self, obj):
        return "other ref [exclude: %s]" % map(str, self.excluding)

    def matches(self, o):
        return not any(r.matches(o) for r in self.excluding)

    def __hash__(self):
        return hash(self.excluding)

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.excluding == other.excluding

class TypenameReference(Reference):
    def __init__(self, typ):
        self.type = typ

    def text(self, obj):
        if obj != pddl.UNKNOWN:
            return "some %s (%s) " %(self.type.name, obj.name)

        return "some "+self.type.name

    def matches(self, o):
        return o.is_instance_of(self.type)

    def __hash__(self):
        return hash(self.type)

    def __eq__(self, other):
        #if self.__class__ == other.__class__:
            #print "__eq__(",self.type,", ",other.type, ") = ",self.type == other.type
        return self.__class__ == other.__class__ and self.type == other.type

    def semantic_equal(self, other):
        #print "EQ?:" ,self, " == ", other
        return self.__eq__(other)

class RelationReference(Reference):
    def __init__(self, function, arguments, value, ref_context):
        self.function = function
        self.args = arguments
        self.value = value
        self.context = ref_context
        self.parents = set()

        assert len(arguments) == len(function.args)

		#not needed: argtype
        self.argtype = None
        for arg, t in chain(zip(arguments, (a.type for a in function.args)), [(value, function.type)]):
            if arg is None:
                self.argtype = t
                break
        assert self.argtype is not None

        self.cached_values = None
        self.cached_partitions = None

    def text(self, obj):
        oname = obj.name if obj != pddl.UNKNOWN else "x"
        valstr = oname if self.value is None else self.value.text()
        argstrs = [oname if a is None else a.text() for a in self.args]
        return "%s(%s) = %s" % (self.function.name, ", ".join(argstrs), valstr)

    def get_type(self):
        if self.value is None:
            return self.function.type
        for a, fa in zip(self.args, self.function.args):
            if a is None:
                return fa.type

    def get_optimistic_partitions(self):
        if self.cached_partitions is None:
            # self.get_values()
            if self.value is None:
                if isinstance(self.function, pddl.Predicate):
                    return []
                i = len(self.args)
            else:
                i = self.args.index(None)

            # best = self.context.best_function_partitions[self.function][i]
#             logger2.debug(type(self.context.best_function_partitions))
#             for k, v in self.context.best_function_partitions.iteritems():
#                 logger2.debug(type(k))
#                 logger2.debug((k.name, k.type)+tuple(k.args))
#                 logger2.debug("%s, %s, %s, %s, %s, %s, %s", k, k.__hash__(), v, k.name, k.type.name, map(str, k.args),
#                               hash((k.name, k.type)+tuple(k.args)))
           # logger2.debug(map(str, self.context.best_function_partitions))

#             logger2.debug(self.function)
#             import copy
#             a = copy.deepcopy(self.function)
#             a._name = "asdasd"
#             logger2.debug(self.function.__hash__())
#             logger2.debug(a.__hash__())
#             
#             logger2.debug(type(self.function))
#             logger2.debug((self.function.name, self.function.type)+tuple(self.function.args))
#             logger2.debug("%s, %s, %s, %s, %s", self.function.__hash__(), self.function.name, self.function.type.name, map(str, self.function.args),
#                           hash((self.function.name, self.function.type)+tuple(self.function.args)))
#             logger2.debug(i)
#             logger2.debug(self.function in self.context.best_function_partitions)
#             logger2.debug(map(str, self.context.best_function_partitions[self.function]))
            best = self.context.best_function_partitions[self.function][i]
            
            self.cached_partitions = []
            for p in best:
                self.cached_partitions.append(frozenset(o for o in p if o in self.get_values()))
                #print "p:", map(lambda x:map(str, x), self.cached_partitions)

        # return self.cached_partitions
        return self.cached_partitions

    def get_values(self):
        if self.cached_values is not None:
            return self.cached_values

        arg_list = []
        for arg, param in zip(self.args, self.function.args):
            if arg is not None:
                arg_list.append(arg.get_matches())
            else:
                arg_list.append(list(self.context.problem.get_all_objects(param.type)))

        if any(len(l) == 0 for l in arg_list):
            self.cached_values = frozenset()
            self.cached_partitions = frozenset()
            return self.cached_values

        if self.value is None:
            values = set()
            for svar, val in self.context.init.iteritems():
                if all(arg in matches for arg, matches in zip(svar.args, arg_list)):
                    values.add(val)

            # for args in product(*arg_list):
            #     svar = state.StateVariable(self.function, [a for a in args])
            #     values.add(self.context.init[svar])

            self.cached_values = frozenset(values)
            # self.cached_partitions = frozenset(frozenset([o]) for o in values)
        else:
            i = self.args.index(None)
            # partitions = defaultdict(list)
            value_matches = self.value.get_matches()
            values = set()
            boolean_value = None
            if isinstance(self.function, pddl.Predicate):
                if len(value_matches) >= 2:
                    # true for all values
                    self.cached_values = frozenset(self.context.problem.get_all_objects(self.function.args[i].type))
                    return
                else:
                    assert len(value_matches) == 1
                    boolean_value = iter(value_matches).next()

            for svar, val in self.context.init.iteritems():
                if svar.function == self.function and all(svar.args[j] in arg_list[j] for j in xrange(0, len(arg_list)) if j != i):
                    if boolean_value is not None:
                        #always add TRUE values, so we can invert later
                        if val == pddl.TRUE:
                            values.add(svar.args[i])
                    elif val in value_matches:
                        values.add(svar.args[i])
            if boolean_value == pddl.FALSE:
                values = set(o for o in self.context.problem.get_all_objects(self.function.args[i].type) if o not in values)


            # for args in product(*arg_list):
            #     svar = state.StateVariable(self.function, args[:-1])
            #     if self.context.init[svar] == args[-1]:
            #         key = tuple(args[0:i] + args[i+1:])
            #         partitions[key].append(args[i])
            #         # print map(str,key), args[i]
            #         values.add(args[i])


            # self.cached_partitions = [frozenset(v) for v in partitions.itervalues()]
            # print len(self.cached_partitions), "unique values"
            self.cached_values = frozenset(values)

        return self.cached_values

    def ref_count(self):
        max_count = 0
        # print "self:", self
        for pe in chain(self.args, [self.value]):
            if pe is not None:
                # print "eval. parent", pe
                c = pe.ref_count()
                # print "    ", c
                max_count += c
        # print "final:", max_count + 1
        return max_count + 1

    def matches(self, o):
        # if not o.is_instance_of(self.argtype):
        #     return False
        return o in self.get_values()
        # if self.value is None:
        #     return o in self.get_values()

        arg_list = [[o] if a is None else a.get_matches() for a in self.args]
        for args in product(*arg_list):
            svar = state.StateVariable(self.function, args)
            if self.value.matches(self.context.init[svar]):
                return True
        return False

    def clone(self, args, value):
        clone = RelationReference(self.function, args, value, self.context)
        clone.__class__ = self.__class__
        clone.parents = set(self.parents)
        clone.parents.add(self)
        return clone

    def get_children(self):
        if len(self.function.args) == 0:
            return
        if isinstance(self.function, pddl.Predicate) and len(self.function.args) < 2:
            return

        for i in xrange(len(self.args)+1):
            for c in self.get_children_for_arg(i):
                yield c

    def get_children_for_arg(self, i):
        #print "get children for ", self, i
        if i == len(self.args):
            if self.value is None:
                return
            if len(self.value.get_matches()) < 2:
                return
            partitions = list(self.value.successors())
            #print self, "!!!I have %d children in %d partitions" % (sum(len(p.children) for p in partitions), len(partitions))
            for p in partitions:
                # print "c:", p, p.information()
                yield [self.clone(self.args, re) for re in p.children if not isinstance(re.ref, OtherReference)]
        else:
            if self.args[i] is None:
                return
            if len(self.args[i].get_matches()) < 2:
                return
            prefix = self.args[0:i]
            suffix = self.args[i+1:len(self.args)]
            partitions = list(self.args[i].successors())
            #print self, "!!!!!!!I have %d children in %d partitions" % (sum(len(p.children) for p in partitions), len(partitions))
            for p in partitions:
                #print "c:", p, p.information()
                yield [self.clone(prefix + [re] + suffix, self.value) for re in p.children  if not isinstance(re.ref, OtherReference)]

    def __hash__(self):
        return hash((self.function, frozenset(self.args), self.value))

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.function == other.function and len(self.args) == len(other.args) and all(a==a2 for a,a2 in zip(self.args, other.args)) and self.value == other.value

    def semantic_equal(self, other):
        return self.__class__ == other.__class__ and self.function == other.function

class FeatureReference(RelationReference):
    def __init__(self, function, value, ref_context):
        #value is a partition entry (currently only with an identity ref)
        RelationReference.__init__(self, function, [None], value, ref_context)

    # def text(self, obj):
    #     return "%s(%s) = %s" % (self.function.name, obj.name, self.value.text())

class FunctionReference(RelationReference):
    def __init__(self, function, args, ref_context):
        # args are generally *partition entry* objects, though we will
        # only use identity references for now
        RelationReference.__init__(self, function, args, None, ref_context)

    # def text(self, obj):
    #     return "%s(%s) = %s" % (self.function.name, " ".join(a.text() for a in self.args), obj.name)
