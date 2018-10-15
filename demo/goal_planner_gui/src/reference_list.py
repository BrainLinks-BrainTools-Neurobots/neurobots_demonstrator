import time, os
import math, itertools, heapq, ConfigParser

from itertools import chain, product
from collections import defaultdict

from pddllib import pddl
from pddllib.state import state

from utils import union

import rospkg

# import dill
# import pickle
# from pickle import dumps
import cloudpickle

import constants
from references import *
from partitions import *

import logging
logger = logging.getLogger('root')
logger2 = logging.getLogger('highlighted')

class ConfigLogger(object):
    def __init__(self, log):
        self.__log = log
    def __call__(self, config):
        self.__log.info("Config:")
        config.write(self)
    def write(self, data):
        # stripping the data makes the output nicer and avoids empty lines
        line = data.strip()
        self.__log.info(line)

class ReferenceList(object):
    def __init__(self, problem, filename):
        self.problem = problem
        self.domain = problem.domain
        self.read_config(filename)
        self.init = state.State.from_problem(problem)

        self.objects = set(o for o in chain(self.problem.objects, self.problem.domain.constants) if o not in (pddl.UNKNOWN,))
        self.objects.update((pddl.TRUE, pddl.FALSE))
        self.types.add(pddl.t_boolean)
        self.typenames.add(pddl.t_boolean)

        self.build_object_references()
        self.ref_objects = {}
        self.config = filename

        self.simple_only = False
        
    def serialize(self, file):
        import os.path
        output = open(file, 'w')
        cloudpickle.dump([self.type_partitions, self.type_information, self.connections, self.best_partitions,
                          self.best_function_partitions, self.best_partitions], output)
        output.close
        
    def deserialize(self, file):
        import os.path
        if os.path.isfile(file):
            logger2.debug("Deserialize partitions from %s", file)
            input = open(file, 'r')
            self.type_partitions, self.type_information, self.connections, self.best_partitions, self.best_function_partitions, self.best_partitions = cloudpickle.load(input)
            input.close()
            return True
        return False
        
    def read_config(self, filename):
        config_dir = os.path.dirname(filename)
        config = ConfigParser.RawConfigParser()
        config.read(filename)
#         config_logger = ConfigLogger(logger2)
#         config_logger(config)
        self.namedict = {pddl.t_object : "object"}
        self.imagedict = {}

        def read_image(name, image_value):
            if image_value in (None, ""):
                return image_value

            path = os.path.join(config_dir, image_value)
            if os.path.isfile(path):
                return path
            elif os.path.isdir(path):
                imgname = os.path.join(image_value, "%s.png" % name)
                if os.path.isfile(imgname):
                    return imgname
            return None

        def read_section(s, all_values, lookup_fn):
            def add(k, name, image):
                try:
                    elem = lookup_fn(k)
                except KeyError, e:
                    return []
                if isinstance(elem, list):
                    for e in elem:
                        self.namedict[e] = name
                        self.imagedict[e] = image
                    return elem
                self.namedict[elem] = name
                self.imagedict[elem] = read_image(name, image)
                return [elem]

            d = {}
            result = set()
            for k, v in config.items(s):
                if ":" in v:
                    name, image = v.split(":")
                    if not name:
                        name = k
                elif not v:
                    name = k
                    image = None
                else:
                    name = v
                    image = None
                    
                k = k.lower()
                if k == '__all':
                    for v in all_values:
                        if v not in d:
                            logger.debug(v)
                            d[v] = v.name
                            result.update(add(v.name, v.name, None))
#                 elif not name:
#                     d[k] = k
#                     result.update(add(k, k, image))
                else:
                    d[k] = name
#                     logger2.debug("k: %s, v: %s, name: %s, image: %s, result: %s", k, v, name, image, map(str, result))
                    result.update(add(k, name, image))
            return result

        def lookup_object(name):
            if name in self.domain:
                return self.domain[name]
            return self.problem[name]

#         logger2.debug(map(str, self.domain.types.itervalues()))

#         logger2.debug("read individuals")
        self.individuals = read_section('individual', [a for a in chain(self.domain.constants, self.problem.objects)], lookup_object)
        self.individuals.update((pddl.TRUE, pddl.FALSE))
#         logger2.debug("read types")
        self.types = read_section('individual types', [t for t in self.domain.types.itervalues()], lambda name: self.domain.types[name])
#         logger2.debug("read typenames")
        self.typenames = read_section('typenames', [t for t in self.domain.types.itervalues()], lambda name: self.domain.types[name])
        self.typenames.add(pddl.t_object)
#         logger2.debug("read functions")
        self.functions = read_section('functions', [f for f in self.domain.functions if not f.builtin], lambda name: self.domain.functions[name])
#         logger2.debug("read functions2")
        self.functions |= read_section('functions', [p for p in self.domain.predicates if not p.builtin] , lambda name: self.domain.predicates[name])
        self.actions = set(self.domain.get_action(a) for a in read_section('actions', (a2.name for a2 in self.domain.actions), lambda name: name))

    def get_name(self, elem):
        if elem not in self.namedict:
            return elem.name
        return self.namedict[elem]

    def get_image(self, elem):
        return self.imagedict.get(elem, None)

    def is_standalone_reference(self, obj):
        for t in self.types:
            if obj.is_instance_of(t):
                return True
        return False

    def build_type_groups(self):
        type_tree = defaultdict(set)
        populated = set()
        for t in self.problem.domain.types.itervalues():
            if t.__class__ == pddl.types.Type:
                for sup in t.supertypes:
                    type_tree[sup].add(t)
        for o in self.objects:
            populated.add(o.type)

        def remove_subtypes(types):
            for t in types:
                if not any(t.is_subtype_of(t2) for t2 in types):
                    yield t

        def collect_children(t):
            # print t, map(str,type_tree[t])
            # print t
            if not type_tree[t]:
                # print "leaf"
                if t in populated:
                    return [t]
                return []
            else:
                children = set()
                for sub in type_tree[t]:
                    children.update(collect_children(sub))
                    if sub in populated:
                        children.add(sub)
                # print t, "ch:", map(str, children)

                return children

        groups = []
        for t in self.problem.domain.types.itervalues():
            # if t in self.typenames:
            #     groups.append([TypenameReference(t)])

            children = collect_children(t)
            if not children:
                continue
            # logger2.debug(t)
            # logger2.debug(map(str, children))
#             logger2.debug(map(str, self.typenames))
            if all(t2 in self.typenames for t2 in children):
                # print "  children complete, ",
                if t not in populated:
                    # logger2.debug("  self unpopulated")
                    # children complete the partition
                    groups.append([TypenameReference(c) for c in remove_subtypes(children)])
                elif t in self.typenames:
                    # logger2.debug("  self referencable")
                    # we can complete the partition with "some other t"
                    refs = [TypenameReference(c) for c in remove_subtypes(children)]
                    groups.append(refs + [OtherReference(refs)])
            else:
                # logger2.debug("  children incomplete")
                if t in self.typenames:
                    # we can complete the partition with "some other t"
                    relevant_children = list(remove_subtypes([c for c in children if c in self.typenames]))
                    refs = [TypenameReference(c) for c in relevant_children]
                    groups.append(refs + [OtherReference(refs)])

        # for g in groups:
        #     print map(str, g)
        return groups

    def build_object_references(self):
        self.groups = []

        # partitioned by identity
        self.named_group = []
        self.named_objects = set()
        for obj in self.objects:
            if obj in self.individuals:
                self.named_group.append(IdentityReference(obj))
                self.named_objects.add(obj)
            else:
                for t in self.types:
                    if obj.is_instance_of(t):
                        self.named_group.append(IdentityReference(obj))
                        self.named_objects.add(obj)
                        break
        if self.named_group:
            self.groups.append(self.named_group + [OtherReference(self.named_group)])

        # partitioned by typename
        self.typegroups = self.build_type_groups()
        print len(self.typegroups)
        self.typegroups.extend([TypenameReference(t)] for t in self.typenames)
        self.groups.extend(self.typegroups)
        self.create_simple_partition()

        # # partitioned by function
        # self.groups.extend(self.build_function_groups())


    def create_simple_partition(self):
        functions = [f for f in self.functions if f.type == pddl.t_boolean and len(f.args) == 1]
        def get_values(f):
            for o in self.problem.get_all_objects(f.args[0].type):
                svar = state.StateVariable(f, [o])
                if svar in self.init:
                    yield o

        values = {}
        
        for f in functions:
            values[f] = set(get_values(f))
            
        used = set()
        group = []
        for f, val in values.iteritems():
            if not val & used:
                value = PartitionEntry.from_reference(IdentityReference(pddl.TRUE), self)
                group.append(FeatureReference(f, value, self))
                used |= val

        logger.debug(map(str, group))
        if len(group) > 1:
            group.append(OtherReference(group))
            self.simple_partition = group
        else:
            self.simple_partition = None

    def create_atomic_partitions(self):
        self.type_partitions = defaultdict(list)
        for t in self.domain.types.itervalues():
            objects = set(self.problem.get_all_objects(t))
            tp = PartitionEntry.from_reference(TypenameReference(t), self)
            for g in chain(self.generate_type_groups(objects), self.generate_relational_groups(objects), self.generate_named_groups(objects)):
                p = Partition(g, tp, self)
                # print p, p.other_count(), p.is_proper()
                if (p.other_count() == 0 or constants.ALLOW_OTHER) and p.is_proper() and p.information() > 0:
                    self.type_partitions[t].append(p)
        print "===================", len(self.type_partitions)            
        self.type_information = defaultdict(lambda:0)
        for t, partitions in self.type_partitions.iteritems():
            # calculate exact information for the combination best 5 partitions
            partitions = sorted(partitions, key=lambda p:-p.information())[:5]

            objects = set(self.problem.get_all_objects(t))
            all_refs = [[c.ref for c in p.children] for p in partitions]
            counts = []
            for refs in product(*all_refs):
                counts.append(len([o for o in objects if all(r.matches(o) for r in refs)]))
            total = float(sum(counts))
            I = sum(-c / total * math.log(c / total, 2) if c != 0 else 0 for c in counts)
            self.type_information[t] = I
            
        self.create_connection_graph()

    def create_connection_graph(self):
        def information(f, i, j):
            all_types = [a.type for a in f.args] + [f.type]
            arg_list = [list(self.problem.get_all_objects(t)) if t != pddl.t_boolean else [pddl.TRUE] for t in all_types]
            # print map(lambda x: map(str, x), arg_list)
            tuples = defaultdict(lambda: 0)
            ivalues = defaultdict(lambda: 0)
            jvalues = defaultdict(lambda: 0)

            # for args in product(*arg_list):
            #     svar = state.StateVariable(f, args[:-1])
            #     val = args[-1]
            #     res = (self.init[svar] == val)
            #     if res:
            # logger.debug("%s, %s, %s", f, i, j)
            for ai, aj in self.get_tuples_from_function(f, i, j):
                # logger.debug("%s, %s", ai, aj)
                tuples[(ai, aj)] += 1
                ivalues[ai] += 1
                jvalues[aj] += 1

            I = 0
            H = 0
            total = float(sum(jvalues.itervalues()))
            for (i, j), c in tuples.iteritems():
                Pi = ivalues[i] / total
                Pj = jvalues[j] / total
                Pij = c / total
                # print i,j,"     ",Pij,Pi,Pj, "->",Pij * math.log(Pij/(Pi*Pj),2)
                I += Pij * math.log(Pij / (Pi * Pj), 2)
                H += Pij * math.log(Pj / (Pij), 2)

            Ii = -sum(c / total * math.log(c / total, 2) for c in ivalues.itervalues())
            Ij = -sum(c / total * math.log(c / total, 2) for c in jvalues.itervalues())
            # print map(str, jvalues.itervalues())

            return H, I, Ii, Ij, total

        def get_subtypes(t):
            yield t
            for t2 in self.domain.types.itervalues():
                if t2.__class__ == pddl.types.Type and t2.is_subtype_of(t):
                    yield t2

        self.connections = defaultdict(lambda: defaultdict(list))
        for f in self.functions:
            all_types = [a.type for a in f.args] + [f.type]
            # logger.debug("all types: %s", " ".join(str(p) for p in all_types))
            for i, t in enumerate(all_types):
                if t == pddl.t_boolean:
                    continue
                for j, t2 in enumerate(all_types):
                    if i == j or t2 == pddl.t_boolean:
                        continue
                    H, I, I1, I2, count = information(f, i, j)
                    if count > 0:
                        for sub1 in get_subtypes(t):
                            for sub2 in get_subtypes(t2):
                                self.connections[sub2][sub1].append((H, f, j, i))
                                logger.debug("f: %s, sub2: %s, sub1: %s, H: %f, I: %f, I1: %f, I2: %f", f, sub2, sub1, H, I, I1, I2)

        for d in self.connections.itervalues():
            for arcs in d.itervalues():
                arcs.sort(key=lambda x:x[0])

    def get_tuples_from_function(self, f, i, j):
        for fact in self.init.iterfacts():
            if fact.svar.function == f:
                args = list(fact.all_args())
                yield (args[i], args[j])

    def join(self, p1, p2, f, i, j):
        buckets = defaultdict(set)
        for ai, aj in self.get_tuples_from_function(f, i, j):
            buckets[aj].add(ai)
        image2 = set(frozenset(union(buckets[aj] for aj in s)) for s in p2)
        image2 = make_partition(image2)

        # print map(lambda x:map(str, x), image2)

        # image2 = [[o for o in elems if o not in conflict] for elems in image2]
        result = []
        for s1 in p1:
            # print " old set:", map(str, s1)
            if True or not any(o in conflict for o in s1):
                all = set()
                for s2 in image2:
                    new = [o for o in s1 if o in s2]
                    if new:
                        # print " new set:", map(str, new)
                        result.append(new)
                        all.update(new)
                # if len(all) < len(s1):
                #     print " remaining:", map(str, [o for o in s1 if o not in all])
                #     result.append([o for o in s1 if o not in all])

        return result

    def create_extended_partitions(self):
        def get_initial(t):
            partition = [set(self.problem.get_all_objects(t))]
#             print t, len(partition[0]), len(self.type_partitions[t])
            for p in self.type_partitions[t]:
#                 print t, len(partition), len(p.children)
                next_p = []
                for objs in partition:
                    next_p += [objs & set(c.get_matches()) for c in p.children]
                partition = [e for e in next_p if e]
#                 print p, len(partition)
#                 t = ""
#                 for p1 in partition:
#                     text = ""
#                     for p2 in p1:
#                         text += p2.name + ", "
#                     t += text + "\n"
#                 print t
            return partition
        partitions = { t : get_initial(t) for t in self.typenames }
#         for a, b in partitions.iteritems():
#             print a, len(b)
        partitions[pddl.t_boolean] = [[pddl.TRUE], [pddl.FALSE]]


        def get_direct_subypes(t):
            for t2 in self.domain.types.itervalues():
                if t2.__class__ == pddl.types.Type and t in t2.supertypes:
                    yield t2

        def complete_partition(t):
            if t not in partitions:
                if isinstance(t, pddl.types.CompositeType):
                    pnew = sum((complete_partition(subt) for subt in t.types), [])
                    partitions[t] = pnew
                else:
                    # print "Composite"
                    # print "subtypes: ", map(str, get_direct_subypes(t))
                    pnew = sum((complete_partition(t2) for t2 in get_direct_subypes(t)), [])
                    # print t, map(lambda x:map(str, x), pnew)
                    all = set(self.problem.get_all_objects(t))
                    used = union(pnew)
                    assert used <= all
                    if used < all:
                        pnew.append(list(all - used))
                    # print (pnew)
                    # tmp = []
                    # for p in pnew:
                    #    print type(p)
                    #    if len(p)>1:
                    #
                    #        for el in p:
                    #            print el
                    #            tmp.append(el)
                    #    else:
                    #        #el = p.pop()
                    #        print "else ", p
                    #        pass
                    #        #tmp.append(el)
                    # print "tmp ", (tmp), "\n"
                    # print "pnew: ",t, map(lambda x:map(str, x), pnew)
                    partitions[t] = pnew

            return partitions[t]
        logger.debug(map(str, partitions))
        logger.debug(type(self.domain.types))
        for t in self.domain.types.values():
            if t not in partitions and t != pddl.types.t_any:
                # print "Complete: ",t
                complete_partition(t)
                # partitions[t] = [[item] for sublist in partitions[t] for item in sublist]
                # print "End Complete;",partitions[t]
        # for t, p in partitions.iteritems():
        #    print t, map(lambda x:map(str, x), p)
        # print "\n"
        logger.debug(map(str, partitions))

        for t, p in partitions.iteritems():
            logger.debug("%s %s", t, map(lambda x:map(str, x), p))

        open = set(self.domain.types.itervalues())
        while open:
            t = open.pop()
            for t2, arcs in self.connections[t].iteritems():
                if t2 not in self.typenames:
                    continue
                for (H, f, i, j) in arcs:
                    p = complete_partition(t)
                    p2 = complete_partition(t2)
                    pnew = self.join(p, p2, f, i, j)
#                    print "join %s,%s via %s" % (t,t2,f.name)
#                    print map(lambda x:map(str, x), pnew)
                    if len(pnew) > len(p):
#                        print "improvement"
                        partitions[t] = pnew
                        for t3, d in self.connections.iteritems():
                            if t in d:
#                                print "push", t3
                                open.add(t3)

        logger.debug("result:")
        for t, p in partitions.iteritems():
            logger.debug("%s %s", t, map(lambda x:map(str, x), p))

        self.best_partitions = partitions

    def create_optimistic_partitions(self):
        self.best_function_partitions = {}
        def get_partition(t):
            if t not in self.best_partitions:
                assert isinstance(t, pddl.types.CompositeType)
                pnew = sum((get_partition(subt) for subt in t.types), [])
                self.best_partitions[t] = pnew
            return self.best_partitions[t]

        logger.debug("Functions: %s", map(str, self.functions))
        for f in self.functions:
            logger.debug("Function: %s, args: %s, type: %s", f, map(str, f.args), f.type)
            self.best_function_partitions[f] = {}
            logger.debug("best function Partiiton: %s", map(str, self.best_function_partitions))
            for i, t in enumerate(chain((a.type for a in f.args), [f.type])):
                if t == pddl.t_boolean:
                    continue
                p1 = [set(self.problem.get_all_objects(t))]
                original = set(p1[0])
                logger.debug("")
                logger.debug("start %s %s %s", f, i, t)
                for j, t2 in enumerate(chain((a.type for a in f.args), [f.type])):
                    if i == j or t2 == pddl.t_boolean:
                        continue
                    logger.debug("inner %s %s", j, t2)
                    p2 = get_partition(t2)
                    logger.debug("%s %s %s", t2, map(lambda x:map(str, x), p1), map(lambda x:map(str, x), p2))

                    p1 = self.join(p1, p2, f, i, j)

                used = union(p1)
                if used < original:
                    p1.append(list(original - used))
                self.best_function_partitions[f][i] = p1
                logger.debug("%s %s", f, i)
                for p in p1:
                    logger.debug(map(str, p))



    def generate_type_groups(self, objects):
        types = set(o.type for o in objects)
        for g in self.typegroups:
            # print map(str, g)
            if len(g) <= 1:
                continue
            g2 = []
            for ref in g:
                if isinstance(ref, TypenameReference):
                    if all(ref.type.equal_or_supertype_of(t) for t in types):
                        g2 = []
                        # print "trivial"
                        break
                    elif any(ref.type.equal_or_supertype_of(t) for t in types):
                        g2.append(ref)
                # else:
                #     g2.append(ref)
            if g2:
                g2.append(OtherReference(g2))
                yield g2

    def generate_named_groups(self, objects):
        group = []
        for o in  self.named_objects.intersection(objects):
            group.append(IdentityReference(o))
        if group:
            return [group + [OtherReference(group)]]
        return []

    def get_relational_group(self, function, i):
        all_types = [a.type for a in function.args] + [function.type]

        partition_args = []
        values = None
        for t in all_types[0:i]:
            partition_args.append(PartitionEntry.from_reference(TypenameReference(t), self))
        if i < len(function.args):
            partition_args.append(None)
            for t in all_types[i + 1:len(function.args)]:
                partition_args.append(PartitionEntry.from_reference(TypenameReference(t), self))
            if function.type == pddl.t_boolean:
                values = [PartitionEntry.from_reference(IdentityReference(v), self) for v in (pddl.TRUE, pddl.FALSE)]
            else:
                values = [PartitionEntry.from_reference(TypenameReference(all_types[-1]), self)]
        else:
            pass

        if values is None:
            group = [FunctionReference(function, partition_args, self)]
        elif len(function.args) == 1:
            assert partition_args[0] is None
            group = [FeatureReference(function, v, self) for v in values]
        else:
            group = [RelationReference(function, partition_args, v, self) for v in values]
        group.append(OtherReference(group))
        return group

    def generate_relational_groups(self, objects):
        # maxtype = pddl.types.most_specific_type([o.type for o in objects])

        for f in self.functions:
            all_types = [a.type for a in f.args] + [f.type]
            for i, t in enumerate(all_types):
                if not any(t.is_compatible(o.type) for o in objects):
                    continue
                yield self.get_relational_group(f, i)
        if self.simple_partition:
            yield self.simple_partition

    def generate_groups(self, objects):
        return chain(self.generate_named_groups(objects), self.generate_type_groups(objects), self.generate_relational_groups(objects))


    def get_all_groups(self, typ):
        objects = set(self.problem.get_all_objects(typ))
        return list(chain(self.generate_named_groups(objects), self.generate_type_groups(objects), self.generate_relational_groups(objects)))

    def get_objects(self, ref):
        if ref not in self.ref_objects:
            self.ref_objects[ref] = set(o for o in self.objects if ref.matches(o))
        return self.ref_objects[ref]

    def print_reference(self, refs):
        pass
