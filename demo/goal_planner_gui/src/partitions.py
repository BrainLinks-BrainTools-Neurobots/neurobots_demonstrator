import time
import math, itertools, heapq

from itertools import chain, product
from collections import defaultdict

from pddllib import pddl
from pddllib.state import state

import constants
from references import *

import traceback
import copy

import translator

import logging
from docutils.nodes import reference
logger = logging.getLogger('root')

def make_partition(sets):
    all = set()
    used = set()
    sets = sorted(sets, key=lambda s: len(s))
    result = []
    for s in sets:
        all |= s
        if not s & used:
            result.append(s)
            used |= s
    if used < all:
        result.append(all - used)
    return result

class Partition(object):
    def __init__(self, refs, parent, ref_context=None):
        self.parent = parent
        self.context = parent.context if parent is not None else ref_context
        self.children = [PartitionEntry(r, self) for r in refs]
        self.hash = None
        self.forbidden = False
        self.expand_parent = None
        self.aditive_partitions = []
        self.alternative_parent = None

    def get_hash(self):
        p = frozenset(frozenset(c.get_matches()) for c in self.children)
        return hash(p)

    def __hash__(self):
        if not self.hash:
            self.hash = self.get_hash()
        return self.hash
    
    def __getstate__(self):
        d = dict(self.__dict__)
        del d['hash']  # no serialization of hash variable
        return d

    def __setstate__(self, d):
        self.hash = None
        self.__dict__.update(d)

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.hash == other.hash
    
    def __ne__(self, other):
        return not self.__eq__(other)

    def get_matches(self):
        return [child.get_matches() for child in self.children]

    def size(self):
        if self.parent:
            return len(self.parent.get_matches())
        return len(self.context.objects)

    def num_nonempty(self):
        return len([c for c in self.children if c.get_matches()])

    @staticmethod
    def get_initial_partition(typ, ref_context):
        return Partition([TypenameReference(typ)], None, ref_context)

    @staticmethod
    def from_references(refs, ref_context):
        parent = None
        assert False, "Why the loop below?"
        for r in refs:
            parent = Partition([r], parent, ref_context)
        return parent

    def other_count(self):
        for c in self.children:
            if isinstance(c.ref, OtherReference):
                return len(c.get_matches())

    def expand_unique(self):
        # print "unique expand: ", str(self), " children: ", len(self.children)
        if len(self.children) > 1:
            return None
        new_p = self.children[0].next(maxsize=1)
        if new_p is not None and len(new_p.children) == 1 and self.children[0].ref != new_p.children[0].ref:
            return new_p.expand_unique()
        return self

    def is_proper(self):
        all = set()
        for m in self.get_matches():
            if any(o in all for o in m):
                return False
            all.update(m)
        return True

    def make_proper(self):
        def conflicts(pe):
            m = set(pe.get_matches())
            for pe2 in self.children:
                if pe == pe2:
                    continue
                m2 = set(pe2.get_matches())
                if m & m2 and not m <= m2:
                    return True
            return False

        used = set()
        group = []
        for pe in self.children:
            if any(o in used for o in pe.get_matches()) or isinstance(pe.ref, OtherReference):
                continue
            # if not conflicts(pe):
            if len(pe.get_matches()) < self.size():
                group.append(pe.ref)
                used.update(pe.get_matches())

        group.append(OtherReference(group))
        return Partition(group, self.parent)

    def split_partition(self, entry):
        assert entry in self.children
        if len(self.children) == 2:
            return self
        if isinstance(entry.ref, OtherReference):
            return None

        other_ref = OtherReference([entry.ref])
        return Partition([entry.ref, other_ref], self.parent)

    def __str__(self):
        if self.parent is None:
            refstr = ""
        else:
            refstr = "%s: " % ", ".join(str(r) for r in self.parent.get_references())
        return "%s[ %s ]" % (refstr, ", ".join("%d: %s" % (len(c.get_matches()), str(c.ref)) for c in self.children))

    def ref_count(self, counted=set()):
        parent_count = 0 if self.parent is None else self.parent.partition.ref_count(counted)
        return parent_count + sum(c.ref.ref_count() for c in self.children)

    def information(self, potential=False):
        """Compute the information content I of this partition. If "potential"
        is set to True, compute I under the assumption that the
        reference can be extended to an optimal split."""

        weight = 1
        for c in self.children:
            if isinstance(c.ref, IdentityReference):
                weight = 10
            elif isinstance(c.ref, RelationReference):
                if c.ref.function.name == "in":
                    weight = 5
                    break
                elif c.ref.function.name == "aligned":
                    weight = 5
                    break

        sets = []
        for c in self.children:
            pc = set(c.get_matches())
            if isinstance(c.ref, RelationReference) and potential:
                # print c.ref, map(str,c.ref.get_values())
                # print c.ref, map(lambda x: len(x), c.ref.get_optimistic_partitions())
                sets += [p & pc for p in c.ref.get_optimistic_partitions()]
                # sets += [set([o) for o in pc]
            else:
                sets.append(pc)

#         logger2.debug("sets: %s", map(lambda x: map(str,x), sets))
        counts = [float(len(s)) for s in make_partition(sets)]
#         logger2.debug("counts: %s, %s", self, map(str, counts))

        # counts = [len(c.get_matches()) for c in self.children]
        # if self.parent is None:
        #     total = len(self.context.problem.objects) + len(self.context.domain.constants)
        # else:
        #     total = len(self.parent.get_matches())
        total = sum(counts)
        if total == 0:
            return 0.0
        # we only have one value > 0
#         elif sum(i > 0 for i in counts) == 1:
#             return 1.0
        
        # print map(str, counts), total
        probs = [float(c) / total for c in counts]
#         logger2.debug("probs: %s", map(str, probs))

        I = sum(-p * math.log(p, 2) if p != 0.0 else 0.0 for p in probs)
        # print "total: ", total, "probs: ",probs, "I: ", I
        # I = 0
        # for c in counts:
        #     print c/total, (1/c * math.log(1/c,2) + (c-1)/c * math.log((c-1)/c,2)) if c >= 2 else 0
        #     I += c/total * (1/c * math.log(1/c,2) + (c-1)/c * math.log((c-1)/c,2)) if c >= 2 else 0
#         logger2.debug("I: %f, total: %f", -I, total)
        return weight * I

    def remaining_size(self, target, potential=False):
        """Return the number of objects that are indistinguishable from "target" given the
        current partitioning.
        """

        weight = 1
        for c in self.children:
            if isinstance(c.ref, IdentityReference):
                weight = 10
            elif isinstance(c.ref, RelationReference):
                if c.ref.function.name == "in":
                    weight = 5
                    break
                elif c.ref.function.name == "aligned":
                    weight = 5
                    break
            

        sets = []
        for c in self.children:
            pc = set(c.get_matches())
            if isinstance(c.ref, RelationReference) and potential:
                sets += [p & pc for p in c.ref.get_optimistic_partitions()]
            else:
                sets.append(pc)
        # print "size target: ", len(pc)
        target_set_list = [pc for pc in sets if target in pc]
        target_set = None
        if len(target_set_list) > 0:
            target_set = target_set_list[0]
        if not target_set or len(target_set) == 1:
            return weight * 2
#         logger2.debug(map(str, target_set))
        return weight * 1 / float(len(target_set) - 1)


    def get_child_for_object(self, obj):
        for pe in self.children:
            if pe.matches(obj):
                return pe

    def dump_stats(self):
        if self.parent is None:
            refstr = ""
        else:
            refstr = "%s: " % ", ".join(str(r) for r in self.parent.get_references())

        child_strings = []
        for c in self.children:
            s = "%d: %s" % (len(c.get_matches()), str(c.ref))
            child_strings.append(s)

        return "%.2f/%.2f: %s[ %s ]" % (self.information(), self.information(potential=True), refstr, ", ".join(child_strings))

class EmptyPartition(Partition):
    def __init__(self):
        Partition.__init__(self, [], None, None)

    def __str__(self):
        return "none"

class PartitionEntry(object):
    def __init__(self, ref, partition):
        self.ref = ref
        self.partition = partition
        self.context = partition.context

        self.reachable_objects = None

        self._all_refs = None
        self._matches = None

    def __hash__(self):
        return hash(str(self.ref.__hash__()) + str(self.partition.__hash__()))
    
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.__hash__() == other.__hash__()
    
    def __ne__(self, other):
        return not self.__eq__(other)
    
    @staticmethod
    def from_reference(ref, ref_context):
        part = Partition([ref], None, ref_context)
        return part.children[0]
    
    #other: PartitionEntry
    #returns a list of different references
    def get_reference_diff(self, other): 
        this_refs = self.get_references()
        other_refs = other.get_references()
        return list(set(this_refs) - set(other_refs))

    def make_existential(self):
        return ExistentialPartitionEntry(self.ref, self.partition)

    def make_alternative(self, new_partition):
        # self.lastpartitions[self._last_key].children[0].ref , self.lastpartitions[self._last_key]
        # alter_part = AlternativePartitionEntry(self.lastpartitions[self._last_key].children[0].ref, self.lastpartitions[self._last_key])

        # alter_part = AlternativePartitionEntry(self.lastpartitions[self._last_key].children[0].ref , self.partition)
        logger.debug("\033[92m New alternative Partition Entry:\033[0m")
        # print "self: ", str(alter_part.ref)
        # print "ref_type: ", type(alter_part.ref)
        # print "all_references: ",map(str, alter_part.get_references())
        # print "partition: ", str(alter_part.partition)
        # print "partition Children: ", map(str, [map(str, c.get_matches()) for c in self.partition.children])
        # print "new Palrtition: ", str(new_partition)
        # print "new Partition children: ", map(str, new_partition.children)
        # print "new Partition refs: ", [map(str, c.get_references()) for c in new_partition.children]
        logger.debug("self PartitionEntry: %s", self)
        logger.debug("self ref: %s", self.ref)
        logger.debug("self refs: %s", map(str, self.get_references()))
        if not new_partition.expand_parent:
            # print "replace expand parent with:",new_partition.children[0]
            new_partition.expand_parent = new_partition.children[0]
        # new_partition.expand_parent = self
        # print "new Partition expand_parent: ", new_partition.expand_parent
        # print "expand_parent ref: ", new_partition.expand_parent.ref
        # alter_part = AlternativePartitionEntry(new_partition.expand_parent.ref , new_partition)
        # alter_part.set_reachable(self.reachable_objects)
        # for child in new_partition.children:
            # child.set_reachable(self.reachable_objects)

        if isinstance(self, AlternativePartitionEntry):
            alter_part = AlternativePartitionEntry(self.get_references()[0], new_partition, self, new_partition)
        else:
            alter_part = AlternativePartitionEntry(self.get_references()[0], new_partition, self.partition.alternative_parent, new_partition)
        # alter_part.set_reachable(self.reachable_objects)

    # else:
        #    alter_part = AlternativePartitionEntry(self.ref , new_partition)
        #    alter_part.expand_parent = self

        # print "last entrys: ", map(str, [c.ref for c in self.lastpartitions[self._last_key].children])
        # print "parent: ", self.lastpartitions[self._last_key].parent


        # alter_part.lastpartitions = list(self.lastpartitions)
        # alter_part.lastpartitions[self._last_key].forbidden = True
        # alter_part.set_expanded_partitions(new_partitions)
        # alter_part.expanded_partitions = self.expanded_partitions
        # alter_part._last_key = self._last_key
        # expando = self.search_best_alternative()
        # print "expanded: ", str(expando)

        # alter_part.lastpartitions = self.lastpartitions
        # alter_part = AlternativePartitionEntry(self.ref, expando)
        # print "new expand: ", str(alter_part)
        logger.debug("Finish Alternative: %s", alter_part)
        # alter_part._all_refs = alter_part.get_references()[1:]
        logger.debug("With Refs: %s", map(str, alter_part.get_references()))

        return alter_part

    # Only temporal
    def get_alternative_parents(self):
        # if isinstance(self, AlternativePartitionEntry):
        #    if self.alternative_parent is None:
        #        return []
        #    else:
        #        return self.alternative_parent.get_alternative_parents() + [self.alternative_parent]
        # else:
        if self.partition.alternative_parent is None:
            return []
        else:
            return self.partition.alternative_parent.get_alternative_parents() + [self.partition.alternative_parent]


    def make_universal(self):
        return UniversalPartitionEntry(self.ref, self.partition)

    def clone(self):
        entry = self.__class__(self.ref, self.partition)
        entry.set_reachable(self.reachable_objects)
        entry._all_refs = self._all_refs
        return entry

    def is_quantified(self):
        return False

    def set_reachable(self, objects):
        if not self.reachable_objects:
            self.reachable_objects = objects
        else:
            logger.debug("[%s] Dont overrite reachable", self)
        self._matches = None

    def ref_count(self, counted=set()):
        # identity subsumes everything else
        if self.get_identity() is not None:
            return 1
        count = self.ref.ref_count() if self.ref not in counted else 0
        counted = counted | set([self.ref])
        count += 0 if self.partition.parent is None else self.partition.parent.ref_count(counted)
        return count


    def get_references(self):
        if self._all_refs is None:
            if self.partition.parent is None:
                self._all_refs = [self.ref]
            else:
                self._all_refs = list()
                # get parent refs, which are not specialized in this partition entry
#                 logger2.debug(type(self.ref))
#                         if isinstance(r, TypenameReference) and not self.ref.type.is_subtype_of(r.type):
#                             self._all_refs.append(r)
#                         elif not isinstance(r, TypenameReference):

                # not optimal, but it helps to remove things like pos(x)=a table, pos(x)=couchtable
                for r in self.partition.parent.get_references():
#                     logger2.debug("%s, %s, %s, %s", type(r), r, type(self.ref), self.ref)
                    if ((isinstance(self.ref, FeatureReference) and isinstance(r, FeatureReference) and self.ref.function.name == r.function.name) 
                            or (isinstance(self.ref, FunctionReference) and isinstance(r, FunctionReference) and self.ref.function.name == r.function.name)):
                        continue
                    
                    self._all_refs.append(r)
                        
#                         logger2.debug("%s %s | %s %s", type(self.ref.value), self.ref.value, type(r.value), r.value)
#                         logger2.debug("%d, %d", len(self.ref.value.get_matches()), len(r.value.get_matches()))
#                 else:

#                 self._all_refs = list(self.partition.parent.get_references())
                self._all_refs = list(set(self._all_refs + [self.ref]))
            
        return self._all_refs

    def matches(self, obj):
        if self.ref.matches(obj):
            if self.partition.parent is None:
                return True
            return self.partition.parent.matches(obj)
        return False

    def get_matches(self):
        if self._matches is None:
			#debug, not used?
            if self.reachable_objects is not None:
#                 logger.debug("reachable_objects: %s", map(str, self.reachable_objects))
                objects = self.reachable_objects
            elif self.partition.parent is None:
                objects = self.context.objects
            else:
                objects = self.partition.parent.get_matches()
            self._matches = frozenset(o for o in objects if self.ref.matches(o))
        return self._matches

    def is_empty(self):
        return not self.get_matches()

    def is_unique(self):
        return len(self.get_matches()) == 1

    def get_identity(self):
        for ref in self.get_references():
            if isinstance(ref, IdentityReference):
                return ref
        return None

    def get_relation_refs(self):
        for ref in self.get_references():
            if ref.__class__ == RelationReference:
                yield ref

    def get_function_refs(self):
        for ref in self.get_references():
            if isinstance(ref, FunctionReference):
                yield ref

    def get_features(self):
        refs = self.get_references()
        for ref in refs:
            if isinstance(ref, FeatureReference):
                # compare argument, if supertype
                yield ref

    def get_most_specific_typeref(self, hidden_refs=False):
        matches = self.get_matches()
        if not matches:
            best = TypenameReference(pddl.t_object)
            for ref in self.get_references():
#                 logger2.debug("%s, %s", type(ref), ref)
                if isinstance(ref, TypenameReference) and ref.type.is_subtype_of(best.type):
                    best = ref
            return best

        all_types = set(self.context.domain.types.itervalues()) if hidden_refs else set(self.context.typenames)
        valid_types = set(t for t in all_types if all(o.is_instance_of(t) for o in matches))
        
#         if len(valid_types) == 1:
#             for t in all_types:
#                 logger2.debug("type: %s", t.name)
#                 if all(o.is_instance_of(t) for o in matches):
#                     logger2.debug("all objects %s are an instance of %s", map(str, matches), t.name)
#         logger2.debug(map(str, all_types))
#         logger2.debug(map(str, valid_types))
#         logger2.debug(map(str, matches))
        
        best = pddl.t_object
        for t in valid_types:
            if t.is_subtype_of(best) and t != pddl.types.t_any:
                best = t
        return TypenameReference(best)

    def get_type_qualifier(self):
        typeref = self.get_most_specific_typeref()
        references = self.get_references()

        other = ""
        if any(isinstance(r, OtherReference) for r in references):
            other = "other "

        if len(self.get_matches()) == 1:
            return "the %s%s" % (other, self.context.get_name(typeref.type))
        elif isinstance(self, UniversalPartitionEntry):
            return "all %s%ss" % (other, self.context.get_name(typeref.type))
        elif isinstance(self, ExistentialPartitionEntry):
            return "an arbitrary %s%s" % (other, self.context.get_name(typeref.type))
        elif isinstance(self, AlternativePartitionEntry):
            return "%s some %s" % (other, self.context.get_name(typeref.type))
        elif not self.get_matches():
            return "a nonexisting %s%s" % (other, self.context.get_name(typeref.type))
        else:
            typename = self.context.get_name(typeref.type);
            article = "an" if typename[0] in ["a", "o", "u", "i", "e"] else "a"
            return "%s %s%s" % (article, other, typename)
        
    def get_other_or_arbitrary(self):
        #logger2.debug("%s", map(str, self.get_references()))
        if isinstance(self, ExistentialPartitionEntry):
            return "arbitrary"
        
        diff = []
        if self.partition.parent:
            diff = self.get_reference_diff(self.partition.parent)
        
        if any(isinstance(r, OtherReference) for r in diff):# and all(not isinstance(r, FeatureReference) or isinstance(r, FunctionReference) for r in self.get_references()):
            return "other"
        
        return None

    def text(self):
        """ Return a textual description of this partition entry"""
        ident = self.get_identity()
        if ident:
            return self.context.get_name(ident.object)
            # return "%s %s" % (typeref.name, ident.object.name)

        function_qualifiers = []
        for r in self.get_function_refs():
            argstrings = [pe.text() for pe in r.args]
            if len(r.get_values()) == 1:
                return translator.translate("%s = %s" % (self.context.get_name(r.function), ", ".join(argstrings)))
            function_qualifiers.append(translator.translate("%s = %s" % (self.context.get_name(r.function), ", ".join(argstrings))))

        feature_qualifiers = []
        for r in self.get_features():
            # FIXME: value != identityref?
            feature_qualifiers.append(translator.translate("%s = %s" % (self.context.get_name(r.function), r.value.text())))

        for r in self.get_relation_refs():
            argstrings = [pe.text() if pe is not None else "x" for pe in r.args]
            feature_qualifiers.append(translator.translate("%s(%s) = %s" % (self.context.get_name(r.function), ", ".join(argstrings), r.value.text())))


        type_qualifier = self.get_type_qualifier()
        
#         logger2.debug("type_qualifier: %s", type_qualifier)

        if isinstance(self, AlternativePartitionEntry):
            logger.debug("infotext for alternative")
            function_qualifiers = []

            for r in self.forbid_partiiton.expand_parent.get_function_refs():
                argstrings = [pe.text() for pe in r.args]
                if len(r.get_values()) == 1:
                    return translator.translate("%s = %s" % (self.context.get_name(r.function), ", ".join(argstrings)))
                function_qualifiers.append(translator.translate("%s = %s" % (self.context.get_name(r.function), ", ".join(argstrings))))


            feature_qualifiers = []
            for r in self.forbid_partiiton.expand_parent.get_features():
                feature_qualifiers.append(translator.translate("%s = %s" % (self.context.get_name(r.function), r.value.text())))
            # print "function_qualifiers: ",map(str, function_qualifiers)
                # feature_qualifiers.append(str(self.forbid_partiiton.expand_parent.ref))
            # return "%s where %s don't care" % (type_qualifier, " and ".join(feature_qualifiers).split('=')[0])
            if function_qualifiers:
                # print "function_qualifiers:", map(str, function_qualifiers)
                return translator.translate("%s where %s and %s don't care" % (type_qualifier, " , ".join(function_qualifiers[:-1]), function_qualifiers[-1]))

            if feature_qualifiers:
                # print "feature_qualifiers: ", map(str, feature_qualifiers)
                return translator.translate("%s with %s and %s don't care" % (type_qualifier, " , ".join(feature_qualifiers[:-1]), feature_qualifiers[-1].split('=')[0]))

            return translator.translate("%s with other ref" % type_qualifier)


        if function_qualifiers and feature_qualifiers:
            return translator.translate("%s that is %s with %s" % (type_qualifier, " and ".join(function_qualifiers), " and ".join(feature_qualifiers)))

        if feature_qualifiers:
            return translator.translate("%s with %s" % (type_qualifier, " and ".join(feature_qualifiers)))

        if function_qualifiers:
            return translator.translate("%s that is %s" % (type_qualifier, " and ".join(function_qualifiers)))

        return type_qualifier


    def description(self):
        """Return a list of (text,image) tuples that can be used to symbolize
        this partition entry.

        Expressions that are more complicated than "feature(x) =
        object" will be generated as text the same as in the text() method.

        """
        
        results = []
        text_results = []

        typeref = self.get_most_specific_typeref()
        prefix = self.get_other_or_arbitrary()
        
#         type_qualifier = self.get_type_qualifier()
#         arbitray_or_other = False
#         if "arbitrary" in type_qualifier:
#             results.append(("arbitrary", None))
#             arbitray_or_other = True
#         if "other" in type_qualifier:
#             results.append(("other", None))
#             arbitray_or_other = True
            
        # easy: we can just reference the object by name

        if prefix:
            results.append((prefix, None))

        ident = self.get_identity()
        if ident:
            results.append((self.context.get_name(ident.object), self.context.get_image(ident.object)))
            return results
        else:
            results.append((self.context.get_name(typeref.type), self.context.get_image(typeref.type)))

        # don't add other attributes in this case
#         if arbitray_or_other:
#             return results

        # if the image is explicitely empty (not None!) for the
        # "function" part, then only show the value.
        # This allows us to configure the UI to skip
        # e.g. "color" in "color, red", as "red" is descriptive
        # enough.
        for r in self.get_features():
            # we can only show feature = value references as images
            if isinstance(r.value.ref, IdentityReference):
#                 logger2.debug("function: %s, %d, %d", self.context.get_name(r.function), len(results), len(text_results))
                obj = r.value.ref.object
                if self.context.get_image(r.function) != "":
                    if r.function.name != "aligned":
                        results.append((self.context.get_name(r.function), self.context.get_image(r.function))) #function
                        results.append(("=", None))

                if r.function.name != "aligned":
                    results.append((self.context.get_name(obj), self.context.get_image(obj))) #value
                else:
                    results.insert(0, (self.context.get_name(obj), self.context.get_image(obj))) #value
                
#                 if r.function.name == "aligned":
#                     results += [results.pop(0)]
            else:
                t = translator.translate("%s = %s" % (self.context.get_name(r.function), r.value.text()))
                text_results.append((t, None))

        # show other references as text only
        for r in self.get_function_refs():
            argstrings = [pe.text() for pe in r.args]
            t = translator.translate("%s = %s" % (self.context.get_name(r.function), ", ".join(argstrings)))
            text_results.append((t, None))

        for r in self.get_relation_refs():
            argstrings = [pe.text() if pe is not None else "x" for pe in r.args]
            t = translator.translate("%s(%s) = %s" % (self.context.get_name(r.function), ", ".join(argstrings), r.value.text()))
            text_results.append((t, None))


        return results + text_results


    def all_next(self, expand_subrefs=True):
        typ = self.get_most_specific_typeref(hidden_refs=True).type
        partitions = []
        # print "expanding", self, self.partition.information(potential=True), self.partition.information()
        if expand_subrefs:
            partitions += self.expand_relation_ref()
            # don't add other references if this one is still uninformative
            if self.partition.information() == 0.0:
                return partitions

        if isinstance(self.ref, RelationReference):
            if not self.partition.is_proper():
                partitions.append(self.partition.make_proper())

        # print "type:", typ
        objects = self.get_matches()
        for g in self.context.generate_groups(objects):
            p = Partition(g, self)
            # print "  add partition:", p, p.information(potential=True), p.information()
            partitions.append(p)
        # print "done"

        return partitions


    def expand_relation_ref(self):
        if not isinstance(self.ref, RelationReference):
            return []

        partitions = []
#         logger.debug("  expanding relation %s of type %s", self.ref, type(self.ref))
        for ref in self.ref.get_children():
#             logger2.debug("%s", map(str, ref))
            ref.append(OtherReference(ref))
            # p = Partition(g, self)
            p = Partition(ref, self.partition.parent)
            p.expand_parent = self
            # print "      add sub-partition:", p, p.information(potential=True), p.information()
            partitions.append(p)
        # print "  relation done"
        # print len(partitions), "partitions"
        return partitions

    def next(self, maxsize=7):
        try:
            return self._next
        except:
            self._next = self.search_best(maxsize, check_information=False)
            if self.reachable_objects:
                logger.debug("%s, %d, %s", self, id(self), map(str, self.reachable_objects))
            # print [map(str, pe.get_matches()) for pe in self._next.children]
            return self._next

    def nextall(self, maxsize=7):

        all_next = self.search_all(maxsize)
        return all_next


    def successors(self, run_size_check=True):
        objects = self.get_matches()
        size = len(objects)
#         if run_size_check and size < 2:
#             return

        for g in self.context.generate_named_groups(objects):
            pnext = Partition(g, self, self.context)
#             logger2.debug("yield named group: %s", str(pnext))
            yield pnext

        for g in self.context.generate_type_groups(objects):
            pnext = Partition(g, self, self.context)
            # print "yield type group: ", str(pnext)
            yield pnext

        current_type = self.get_most_specific_typeref().type
        I = -1 / float(size) * math.log(1 / float(size), 2)
        if self.context.type_information[current_type] > I:
            # print "current type %s has max I: %.2f >  %.2f" %(current_type, self.context.type_information[current_type],I)
            for p in self.context.type_partitions[current_type]:
                refs = [c.ref for c in p.children]
                pnext = Partition(refs, self, self.context)
                if pnext.information() >= I and (constants.ALLOW_OTHER or pnext.other_count() == 0):
                    # print "partition %s has I = %.2f" %(pnext, pnext.information())
                    yield pnext

        arcs_done = set()
        for t2, arcs in self.context.connections[current_type].iteritems():
            # print "from", typ, "to", t2
            for H, f, i, j in arcs:
                if (f, j, i) not in arcs_done and (f, i, j) not in arcs_done:
#                     logger.debug("H of %s to %s via %s = %.2f" %(current_type, t2, f.name, H))
                    refs = self.context.get_relational_group(f, i)
                    pnext = Partition(refs, self, self.context)
                    arcs_done.add((f, i, j))  # filter permutations
                    arcs_done.add((f, j, i))  # filter permutations
                    if (constants.ALLOW_OTHER or pnext.other_count() == 0):
                        # print "yield context connection: ", str(pnext)
                        yield pnext

        for p in self.expand_relation_ref():
            # print "yield expand relation: ", str(p)
            yield p


    def search_best(self, maxsize=7, maxdepth=1, target=None, check_information=True):
        """Search for the reference that best splits this partition.
        Performs a depth-limited best-first search over the space of
        compound references.

        If "target" is specified, search for a split so that the set
        that contains it is as small as possible.
        """
        # logger2.debug("\033[92m expand me: %s, target: %s\033[0m", self, target)
        def get_score(p):
            if target is None:
                return p.information(), p.information(potential=True)
            else:
                return p.remaining_size(target), p.remaining_size(target, potential=True)

        def sort_key(p, index, depth=0):
            other_group = int(any(isinstance(c.ref, OtherReference) for c in p.children))
#             t0 = time.time()
            I, Ip = get_score(p)
            # print "took", time.time()-t0
            return (-Ip, -I, p.ref_count(), depth, other_group, index)

        def expand(p):
            relevant = [c for c in p.children if not c.is_empty()]
            result = []
            for pnext in relevant:
                for p in pnext.expand_relation_ref():
                    if target is not None and isinstance(p.get_child_for_object(target), OtherReference):
                        # don't allow the use of "other" references when searching a reference to a target object
                        continue
                    # yield p
                    result.append(p)
            # print "%d sub-successors" % len(result)
            return result
        def forbid_partiitons(partitions, toforbid):
            def get_expand_ref(parent):
                if parent is None:
                    return []
                else:
                    return [parent.ref] + get_expand_ref(parent.partition.parent)
            logger.debug("\033[92mtoforbid: " + toforbid)
            logger.debug("expand parent: " + toforbid.expand_parent.ref)
            logger.debug("parent refs:  %s", map(str, toforbid.parent.get_references()))
            logger.debug("toforbid children:" + len(toforbid.children))
            logger.debug("forbid adaptive: " + [map(str, p.get_references()[1:]) for p in toforbid.aditive_partitions])
            # forbid_refs = [toforbid.expand_parent.ref] + toforbid.parent.get_references()[1:]
            forbid_refs = [child.ref for child in toforbid.children] + toforbid.parent.get_references()[1:] + [ref for p in toforbid.aditive_partitions for ref in p.get_references()[1:]]
            logger.debug("all refs to forbid:  %s", map(str, forbid_refs))
            for part in partitions:
                logger.debug("Check Partition: " + part)
                part_refs = list(chain(*[child.get_references() for child in part.children]))
                part_refs = [child.ref for child in part.children]
                logger.debug("partrefs:  %s", map(str, part_refs) + map(type, part_refs))
                skip = False
                for forbid_ref in forbid_refs:
                    logger.debug("To Forbid: " + forbid_ref + type(forbid_ref))
                    if isinstance(forbid_ref, FeatureReference):
                        if any([forbid_ref.function.name == ref.function.name for ref in part_refs if isinstance(ref, FeatureReference)]):
                            logger.debug("Forbid Feature:" + part)
                            part.forbidden = True
                            skip = True
                            break
                    elif isinstance(forbid_ref, TypenameReference):
                        if any([forbid_ref == ref for ref in part_refs if isinstance(ref, TypenameReference)]):
                            logger.debug("Forbid Typename:" + part)
                            part.forbidden = True
                            skip = True
                            break
                    else:
                        logger.debug("Other Ref:" + type(forbid_ref))
                    if any([ref for ref in part_refs if isinstance(ref, TypenameReference)]):
                        # print "Forbid crappy Typename:", part
                        # part.forbidden = True
                        # skip = True
                        # break
                        pass

                    if skip:
                        continue

        successors = [p  for p in self.successors(check_information) if (p.other_count() == 0 or constants.ALLOW_OTHER)]
        logger.debug("succesors: %s", ('\n'.join(map(str, successors))))
        logger.debug("")

        partitions = []
        forbidden_partitions = []
        forbidden_refs = []
        used = set()
        for p in successors:
#             for c in p.children:
#                 logger2.debug("%s, %s", type(c.ref).__name__, c)
#             if target is not None:
#                 logger2.debug("%s", type(p.get_child_for_object(target)))
            # print "  %.3f/%.3f/%.3f" % (p.information()/p.num_nonempty(), p.information(), p.information(potential=True)), p
            if target is not None and isinstance(p.get_child_for_object(target), OtherReference):
#                 logger2.debug("type: %s", type(p.get_child_for_object(target)).__name__)
#                 logger2.debug("Don't use other")
                # don't allow the use of "other" references when searching a reference to a target object
                continue

            if p in used and not any(isinstance(c.ref, RelationReference) for c in p.children):
                # print "duplicate"
                pass
            else:
                partitions.append(p)
            used.add(p)

        # logger2.debug("partitions: %s", ('\n'.join(map(str, partitions))))

        if isinstance(self, AlternativePartitionEntry):
            logger.debug("\033[92mAll Alternative Parents: %s", self.get_alternative_parents())
            forbidden_partitions = [p.forbid_partiiton for p in self.get_alternative_parents()] + [self.forbid_partiiton]
            forbidden_refs = [p.expand_parent.ref for p in [p.forbid_partiiton for p in self.get_alternative_parents()] + [self.forbid_partiiton]]
            logger.debug("forbidden refs: %s", map(str, forbidden_refs))
            # print "To Forbid: ", map(str, forbidden_partitions), "\033[0m"
        else:
            # print "alternative parent", self.partition.alternative_parent
            forbidden_partitions = [p.forbid_partiiton for p in self.get_alternative_parents()]
            forbidden_refs = [p.expand_parent.ref for p in [p.forbid_partiiton for p in self.get_alternative_parents()]]

            # print "Normal forbidden refs:", map(str, forbidden_refs)

#         if isinstance(self, AlternativePartitionEntry) and False:
#             # partitions = self.lastpartitions
#             logger.debug("Remove new_partition: " + self.partition)
#             logger.debug("Remove me: " + self)
#             logger.debug("Parent new_partition: " + self.partition.parent)
#             logger.debug("Parent partition new_partition: " + self.partition.parent.partition)
#             logger.debug("Parent partitions parent: " + self.partition.parent.partition.parent)
#             logger.debug("Children of Parent:  %s", map(str, self.partition.parent.partition.children))
#             logger.debug("remove:  %s", map(str, [a for a in partitions if a.forbidden]))
#             logger.debug("Expand Parent: " + self.partition.expand_parent, type(self.partition.expand_parent))
#             # successors += [p  for p in self.partition.expand_parent.successors() if (p.other_count() == 0 or constants.ALLOW_OTHER)]
#             logger.debug("Add successors:  %s", map(str, [p  for p in self.partition.expand_parent.successors() if (p.other_count() == 0 or constants.ALLOW_OTHER)]))
#             forbid_partiitons(partitions, self.partition)
#             # print "additional forbid:", partitions[-1]
#             # partitions[0].forbidden = False
#             # for part in partitions:
#                 # if str(part) == str(self.new_partition):
#                     # print "found same Partition"
#                     # part.forbidden = True
#             logger.debug("\033[0mallowed partitions:  %s", map(str, [p for p in partitions if p.forbidden == False]))

#         logger2.debug("currently matching: %s", map(str, self.get_matches()))
        self.lastpartitions = list(partitions)
        if not partitions:
            return None
#         logger2.debug("new partitions: %s", map(str, [part for part in partitions if part.forbidden == False]))
        keys = [sort_key(p, i) for i, p in enumerate(partitions)]
        best = None
        best_I = 0.0
        best_rc = 0
        heapq.heapify(keys)

        layer_I = defaultdict(lambda: 0)

        maxgrace = 100
        grace = maxgrace
        # print "Start While"
        # print "keys: ", map(str, keys)
        while keys:
            k = heapq.heappop(keys)
            Ip, I, rc, depth, _, i = k
            Ip = -Ip; I = -I
            p = partitions[i]
            # if len(partitions) == 22:
                # print map(str, [p for p in partitions if p.forbidden == False])
            # print "best I:", best_I
            # print "pop:", Ip,I, p, rc, depth
            # print "[",p,"] refs:", map(str, [child.ref for child in p.children])
            p_refs = [child.ref for child in p.children]
            # print "if ", p_refs[0], type(p_refs[0])," == ", zip(map(str, [ref for ref in forbidden_refs]), [type(ref) for ref in forbidden_refs])
            if p.other_count() > 0 and not constants.ALLOW_OTHER:
                continue
            if any([p_refs[0].semantic_equal(r) for r in forbidden_refs]):
                logger.debug("Skip: %s", p)
                continue
            # print "k: ", k
            # print "[",i,"]Partiiton: ", str(p), " I: ",I, "I_p: ", Ip
            layer_I[depth] = max(I, layer_I[depth])
            if I > best_I or (I == best_I and rc < best_rc) :
                logger.debug("#### found best: p=%s, I=%f, Ip=%f depth=%d, rc=%d", p, I, Ip, depth, rc)
                best_I = I
                best_rc = rc
                best = p
                logger.debug("best#: %s", best)
                # self._last_key = i
            if Ip - I < 0.01:
#                 logger2.debug("I diff < 0.01")
                # continue
                break

            # print grace, depth, I, layer_I[depth-1]
            # if depth == maxdepth and I >= layer_I[depth-1]:
            if I >= layer_I[depth - 1]:
                if grace == 0:
                    logger.debug("grace == 0")
                    break
                else:
                    grace -= 1

            # else:
            #     grace = maxgrace

            if depth >= maxdepth:
                continue
            # print "expand: ", p
            for pnext in expand(p):
                i2 = len(partitions)
                k = sort_key(pnext, i2, depth + 1)
                Ip = -k[0]; I = -k[1]
                if Ip >= best_I:
#                     logger.debug("Ip=%f >= best_i=%f", Ip, best_I)
                    partitions.append(pnext)
                    heapq.heappush(keys, k)
                    # print "push:", pnext, k

        # print "best", str(best)
        # print "Type: ", type(best)
        # print "partitions size", len(partitions)
        if best is None:
            return None
        # print self.partition.information(), best.information(potential=True)
        if not best.is_proper():
            best = best.make_proper()
            logger.debug("not proper! %s", best)

        # print self.partition.information(), "vs", best.information(potential=True) , "-----", self.partition.size(), "vs", best.size()

        if best.parent != self:
            # if isinstance(self, AlternativePartitionEntry) and False:
            #    best.parent = self
                # print "Replace Parent with: ", self.partition.parent
                # print "Self: ", best
                # print "Parent new_partition: ", best.parent
                # print "Parent partition new_partition: ", best.parent.partition
                # print "Parent partitions parent: ", best.parent.partition.parent
                # print "Children of Parent: ", map(str, best.parent.partition.children)
                # print "Children", map(str, best.children)
                # print "matches: ", [map(str, match) for match in best.get_matches()]

                # best.aditive_partitions = self.partition.aditive_partitions
                # best.aditive_partitions.append(self)
                # for child in best.children:
                #    child.set_reachable(self.reachable_objects)
            # else:
            best.parent = self
        if isinstance(self, AlternativePartitionEntry):
            best.alternative_parent = self
        else:
            best.alternative_parent = self.partition.alternative_parent

        information = best.information(potential=True)
        logger.debug("best: %s, information: %f, length matches: %d", best, information, len(best.get_matches()))
        
#         len_matches = len(best.get_matches())
        
        # if self.partition.information() >= best.information(potential=True):# and self.partition.size() <= best.size():
        # 
        if check_information and (information <= 0.0001):  # or (constants.ALLOW_OTHER and len_matches <= 2) or (not constants.ALLOW_OTHER and len_matches <= 1)):
            # print "best is no improvement"
            return None
        # print "partitions: ", map(str, partitions)
        return best
        # Ip, I, rc, depth, _, i = heapq.heappop(keys)
        # return partitions[i]




    def search_all(self, maxsize=7, maxdepth=2, target=None):
        """ Customized search for Debug. Returns all Possible Children

        If "target" is specified, search for a split so that the set
        that contains it is as small as possible.
        """
        logger.debug("\033[92m expand me: %s\033[0m", self)
        def get_score(p):
            if target is None:
                return p.information(), p.information(potential=True)
            else:
                return p.remaining_size(target), p.remaining_size(target, potential=True)

        def sort_key(p, index, depth=0):
            other_group = int(any(isinstance(c.ref, OtherReference) for c in p.children))
            t0 = time.time()
            I, Ip = get_score(p)
            # print "took", time.time()-t0
            return (-Ip, -I, p.ref_count(), depth, other_group, index)

        def expand(p):
            relevant = [c for c in p.children if not c.is_empty()]
            result = []
            for pnext in relevant:
                for p in pnext.expand_relation_ref():
                    if target is not None and isinstance(p.get_child_for_object(target), OtherReference):
                        # don't allow the use of "other" references when searching a reference to a target object
                        continue
                    # yield p
                    result.append(p)
            # print "%d sub-successors" % len(result)
            return result
        def forbid_partiitons(partitions, toforbid):
            def get_expand_ref(parent):
                if parent is None:
                    return []
                else:
                    return [parent.ref] + get_expand_ref(parent.partition.parent)
            logger.debug("\033[92mtoforbid: " + toforbid)
            logger.debug("expand parent: " + toforbid.expand_parent.ref)
            logger.debug("parent refs:  %s", map(str, toforbid.parent.get_references()))
            logger.debug("toforbid children:" + len(toforbid.children))
            # forbid_refs = [toforbid.expand_parent.ref] + toforbid.parent.get_references()[1:]
            forbid_refs = [child.ref for child in toforbid.children] + toforbid.parent.get_references()[1:]
            logger.debug("all refs to forbid:  %s", map(str, forbid_refs))
            for part in partitions:
                logger.debug("Check Partition: " + part)
                part_refs = list(chain(*[child.get_references() for child in part.children]))
                part_refs = [child.ref for child in part.children]
                logger.debug("partrefs:  %s", map(str, part_refs) + map(type, part_refs))
                skip = False
                for forbid_ref in forbid_refs:
                    logger.debug("To Forbid: " + forbid_ref + type(forbid_ref))
                    if isinstance(forbid_ref, FeatureReference):
                        if any([forbid_ref.function.name == ref.function.name for ref in part_refs if isinstance(ref, FeatureReference)]):
                            logger.debug("Forbid Feature:" + part)
                            part.forbidden = True
                            skip = True
                            break
                    elif isinstance(forbid_ref, TypenameReference):
                        if any([forbid_ref == ref for ref in part_refs if isinstance(ref, TypenameReference)]):
                            logger.debug("Forbid Typename:" + part)
                            part.forbidden = True
                            skip = True
                            break
                    else:
                        logger.debug("Other Ref:" + type(forbid_ref))
                    if any([ref for ref in part_refs if isinstance(ref, TypenameReference)]):
                        logger.debug("Forbid crappy Typename:" + part)
                        part.forbidden = True
                        skip = True
                        break

                    if skip:
                        continue



        successors = [p  for p in self.successors() if (p.other_count() == 0 or constants.ALLOW_OTHER)]
        logger.debug("succesors:  %s", map(str, successors))
        # print "%d successors" % len(successors)
        # partitions = [p  for p in self.all_next(expand_subrefs=False) if (p.other_count() == 0 or constants.ALLOW_OTHER)]
        # # if isinstance(self.ref, RelationReference):
        # #     partitions.append(self.partition)
        # print self, "from", self.partition
        # for p in self.successors():
        #     print p, get_score(p)
        # print "-------------------"
        # print "initial partitions:"
        # print "-------------------"

        partitions = []
        used = set()
        for p in successors:
            # print "  %.3f/%.3f/%.3f" % (p.information()/p.num_nonempty(), p.information(), p.information(potential=True)), p
            if target is not None and isinstance(p.get_child_for_object(target), OtherReference):
                # don't allow the use of "other" references when searching a reference to a target object
                continue

            if p in used and not any(isinstance(c.ref, RelationReference) for c in p.children):
                # print "duplicate"
                pass
            else:
                partitions.append(p)
            used.add(p)

        if isinstance(self, AlternativePartitionEntry):
            # partitions = self.lastpartitions
            logger.debug("Remove new_partition: " + self.partition)
            logger.debug("Remove me: " + self)
            logger.debug("Parent new_partition: " + self.partition.parent)
            logger.debug("Parent partition new_partition: " + self.partition.parent.partition)
            logger.debug("Parent partitions parent: " + self.partition.parent.partition.parent)
            logger.debug("Children of Parent:  %s", map(str, self.partition.parent.partition.children))
            logger.debug("remove:  %s", map(str, [a for a in partitions if a.forbidden]))
            logger.debug("Expand Parent: " + self.partition.expand_parent + type(self.partition.expand_parent))
            forbid_partiitons(partitions, self.partition)
            # print "additional forbid:", partitions[-1]
            # partitions[0].forbidden = False
            # for part in partitions:
                # if str(part) == str(self.new_partition):
                    # print "found same Partition"
                    # part.forbidden = True
            logger.debug("\033[0mallowed partitions:  %s", map(str, [p for p in partitions if p.forbidden == False]))

        if not partitions:
            return None
        logger.debug("1. Step Expand")
        expands = []
        for p in partitions:
            for pnext in expand(p):
                expands.append(pnext)

        partitions = partitions + expands

        logger.debug("Add Parents")
        for part in partitions:
            if part.parent != self:
                part.parent = self
        logger.debug("Return Partition")
        return partitions





    def is_leaf(self):
        """ Return True if this partition has no children """

        if self.is_quantified() or self.is_unique() or self.is_empty():
            return True

        # test whether there are successors
        new_partition = self.next(maxsize=constants.MAX_ENTRIES)

        if new_partition and len(new_partition.children) > 1:
            return False
        return True

    def expand(self, allow_universal=False):
        """Return a list of successors of this partition entry. If
        "allow_universal" is true, include universally quantified
        references in the successors.

        If this entry is a leaf node, it this method will return a
        list containing only itself.
        """

        # print "reference: expand", str(self), allow_universal

        # shortcut: those entries cannot be expanded any more
        if self.is_leaf():
#             logger2.debug("exanded: leaf")
            return [self]
        
        new_partition = self.next(maxsize=constants.MAX_ENTRIES)
        # print "new Partition cildren types: ", [type(p) for p in new_partition.children]
        # print "expanded:", self
        # copy_part = cPickle.loads(cloud.serialization.cloudpickle.dumps(new_partition))
#         logger2.debug("expanded: existential %s", self.make_existential())
        if not allow_universal:
            # print "\033[92m Existential: ", map(str, [self.make_existential(), self.make_alternative()]), "\033[0m"
            # print "Type part child: ", type(new_partition.children), " type self", type(self)
            # return new_partition.children + [self.make_existential(), self.make_alternative(new_partition)]
            return new_partition.children + [self.make_existential(), self.make_alternative(new_partition)]
            # return new_partition.children + [self.make_existential(), self.make_alternative(copy_part)]
        else:
            # print "\033[92m Existential: ", map(str, [self.make_existential(), AlternativePartitionEntry()]), "\033[0m"
            # print "Type part child: ", type(new_partition.children), " type self", type(self)
            # return new_partition.children + [self.make_existential(), self.make_alternative(copy_part), self.make_universal()]
            return new_partition.children + [self.make_existential(), self.make_alternative(new_partition), self.make_universal()]

    def expandall(self, allow_universal=False):
        """Return a list of successors of this partition entry. If
        "allow_universal" is true, include universally quantified
        references in the successors.

        If this entry is a leaf node, it this method will return a
        list containing only itself.
        """

        # print "reference: expand", str(self), allow_universal

        # shortcut: those entries cannot be expanded any more
        if self.is_leaf():
            # print "reference: expanded"
            return [self]

        new_partition = self.nextall(maxsize=constants.MAX_ENTRIES)
        # print "new Partition cildren types: ", [type(p) for p in new_partition.children]
        # print "expanded:", self
        if not allow_universal:
            # print "\033[92m Existential: ", map(str, [self.make_existential(), self.make_alternative()]), "\033[0m"
            # print "Type part child: ", type(new_partition.children), " type self", type(self)
            # return new_partition.children + [self.make_existential(), self.make_alternative(new_partition)]
            return [part.children for part in new_partition] + [self.make_existential()]
        else:
            # print "\033[92m Existential: ", map(str, [self.make_existential(), AlternativePartitionEntry()]), "\033[0m"
            # print "Type part child: ", type(new_partition.children), " type self", type(self)
            return [children for part in new_partition for children in part.children] + [self.make_existential(), self.make_universal()]
            # return new_partition.children + [self.make_existential(), self.make_universal()]



    def expand_single(self, target, _check_information):
        """return the smallest successor to this partition that contains
        "target". """

#         successors = self.successors()
#         best = self

#         logger2.debug(target)
        next = self.search_best(9999, target=target, check_information=_check_information)
#         logger2.debug(next)
        if next is None:
            return None
#         logger2.debug("%s %s", type(next.get_child_for_object(target)), next.get_child_for_object(target))
        return next.get_child_for_object(target)

    def __str__(self):
        return "[" + ("\n".join(str(r) for r in self.get_references())) + "]"

class ExistentialPartitionEntry(PartitionEntry):
    def __str__(self):
        return "E:" + ", ".join(str(r) for r in self.get_references())

    def is_quantified(self):
        return True

class AlternativePartitionEntry(PartitionEntry):
    def __init__(self, ref, partition, alternative_parent, forbid_partiiton):
        super(AlternativePartitionEntry, self).__init__(ref, partition)
        self.alternative_parent = alternative_parent
        self.forbid_partiiton = forbid_partiiton
        self.expand_partition = None

    def __str__(self):
        return "A:" + ", ".join(str(r) for r in self.get_references())


    def get_alternative_parents(self):
        if self.alternative_parent is None:
            return []
        else:
            return list(set(self.alternative_parent.get_alternative_parents() + [self.alternative_parent]))

    def is_quantified(self):
        return False

class UniversalPartitionEntry(PartitionEntry):
    def __str__(self):
        return "U:" + ", ".join(str(r) for r in self.get_references())

    def is_quantified(self):
        return True
