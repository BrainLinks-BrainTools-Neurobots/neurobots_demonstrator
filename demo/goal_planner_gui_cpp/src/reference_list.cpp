/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 26, 2017
 *      Author: kuhnerd
 * 	  Filename: reference_list.cpp
 */

#include <goal_planner_gui/config_parser.h>
#include <goal_planner_gui/partition_entry.h>
#include <goal_planner_gui/reference_list.h>
#include <goal_planner_gui/constants.h>
#include <boost/range/join.hpp>
#include <goal_planner_gui/partitions.h>
#include <goal_planner_gui/pddl/actions.h>
#include <goal_planner_gui/pddl/state/facts.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/references/feature_reference.h>
#include <goal_planner_gui/references/function_reference.h>
#include <goal_planner_gui/references/other_reference.h>
#include <memory>
#include <queue>
#include <numeric>
#include <cmath>

namespace goal_planner_gui
{

void collectChildren(const std::shared_ptr<pddl::Type>& t,
		pddl::TypeUnorderedSet& children,
		std::unordered_map<std::shared_ptr<pddl::Type>, pddl::TypeUnorderedSet, pddl::TypeHasher>& typeTree,
		pddl::TypeUnorderedSet& populated)
{
	if (CONTAINS_NOT(t, typeTree) || typeTree[t].empty())
	{
		if (CONTAINS(t, populated))
		{
			children.insert(t);
		}
		return;
	}
	else
	{
		for (auto& sub : typeTree[t])
		{
			collectChildren(sub, children, typeTree, populated);
			if (CONTAINS(sub, populated))
				children.insert(sub);
		}
	}
}

ReferenceList::ReferenceList(std::shared_ptr<pddl::Problem> problem) :
				m_problem(problem),
				m_domain(m_problem->getDomain())
{
}

void ReferenceList::init(const std::string& filename)
{
	readConfig(filename);
	m_init = pddl::State::fromProblem(m_problem);

	for (auto& o : m_problem->m_objects)
		if (o != pddl::DefaultTypedObjects::unknownObject())
			m_objects.insert(o);
	for (auto& o : m_domain->m_constants)
		if (o != pddl::DefaultTypedObjects::unknownObject())
			m_objects.insert(o);
	m_objects.insert(pddl::DefaultTypedObjects::trueObject());
	m_objects.insert(pddl::DefaultTypedObjects::falseObject());

	m_types.insert(pddl::DefaultTypes::booleanType());
	m_typenames.insert(pddl::DefaultTypes::booleanType());

	buildObjectReferences();
}

ReferenceList::~ReferenceList()
{
}

std::string getImagePath(const std::string& name,
		const std::string& value,
		const std::string& configDir)
{
	using namespace boost::filesystem;

	if (value.empty())
		return value;

	path p = path(configDir) / value;
	if (is_regular_file(p))
	{
		return p.string();
	}
	else if (is_directory(p))
	{
		path imgname = p / (name + ".png");
		if (is_regular_file(imgname))
			return imgname.string();
		else
			return std::string();
	}
	else
	{
		return std::string();
	}
}

void addString(const std::string k,
		const std::string& name,
		const std::string& image,
		const std::string& configDir,
		std::function<std::string(const std::string& name)>& lookupFn,
		std::unordered_map<std::size_t, std::string>& nameDict,
		std::unordered_map<std::size_t, std::string>& imageDict,
		std::vector<std::string>& result)
{
//	LOG_INFO("add " << name << " " << " to dict");
	const std::string elem = lookupFn(k);
	if (elem.empty())
	{
		result.push_back(elem);
		return;
	}

	std::size_t h = 0;
	pddl::hash_combine(h, name);

	nameDict[h] = name;
	imageDict[h] = getImagePath(name, image, configDir);
	result.push_back(elem);
}

template<class Type>
void addElement(const std::string k,
		const std::string& name,
		const std::string& image,
		const std::string& configDir,
		std::function<Type(const std::string& name)>& lookupFn,
		std::unordered_map<std::size_t, std::string>& nameDict,
		std::unordered_map<std::size_t, std::string>& imageDict,
		std::vector<Type>& result)
{
	const Type elem = lookupFn(k);
	if (!elem)
	{
		result.push_back(elem);
		return;
	}

	nameDict[elem->hash()] = name;
	imageDict[elem->hash()] = getImagePath(name, image, configDir);
	result.push_back(elem);
}

void addList(const std::string k,
		const std::string& name,
		const std::string& image,
		const std::string& configDir,
		std::function<std::vector<std::shared_ptr<pddl::Function>>(const std::string& name)>& lookupFn,
		std::unordered_map<std::size_t, std::string>& nameDict,
		std::unordered_map<std::size_t, std::string>& imageDict,
		std::vector<std::shared_ptr<pddl::Function>>& result)
{
	const std::vector<std::shared_ptr<pddl::Function>> elem = lookupFn(k);

	for (auto& it : elem)
	{
		nameDict[it->hash()] = name;
		imageDict[it->hash()] = getImagePath(name, image, configDir);
		result.push_back(it);
	}
}

#define AddFnDecl(Type) std::function<void(const std::string,\
	const std::string&,\
	const std::string&,\
	const std::string&,\
	std::function<Type(const std::string& name)>&,\
	std::unordered_map<std::size_t, std::string>&,\
	std::unordered_map<std::size_t, std::string>&,\
	std::vector<Type>&)>

#define AddListFnDecl(Type) std::function<void(const std::string k,\
	const std::string& name,\
	const std::string& image,\
	const std::string& configDir,\
	std::function<std::vector<std::shared_ptr<Type>>(const std::string& name)>& lookupFn,\
	std::unordered_map<std::size_t, std::string>& nameDict,\
	std::unordered_map<std::size_t, std::string>& imageDict,\
	std::vector<std::shared_ptr<Type>>& result)>

bool ReferenceList::readConfig(const std::string& filename)
{
	boost::filesystem::path p(filename);
	const std::string configDir = p.parent_path().string();
	ConfigParser config(filename);

	m_nameDict[pddl::DefaultTypes::objectType()->hash()] = "object";

	//individuals
	{
		auto allValues = m_domain->m_constants;
		allValues.insert(m_problem->m_objects.begin(), m_problem->m_objects.end());
		AddFnDecl(std::shared_ptr<pddl::TypedObject>)addFn = addElement<std::shared_ptr<pddl::TypedObject>>;
		readSection<std::shared_ptr<pddl::TypedObject>, pddl::TypedObjectHasher, std::shared_ptr<pddl::TypedObject>>("individual", allValues, configDir, config, m_individuals,
				[&] (const std::string& name) -> std::shared_ptr<pddl::TypedObject>
				{
					if (m_domain->contains(name))
					{
						return m_domain->get(name);
					}
					return m_problem->get(name);
				}, addFn);
		m_individuals.insert(pddl::DefaultTypedObjects::trueObject());
		m_individuals.insert(pddl::DefaultTypedObjects::falseObject());

		for (auto& it : m_individuals)
		{
			LOG_INFO("Found individual " << it->getName());
		}
	}

	//individual types & typenames
	{
		std::unordered_set<std::shared_ptr<pddl::Type>, pddl::TypeHasher> allValues;
		for (auto& it : m_domain->m_types)
			allValues.insert(it.second);
		AddFnDecl(std::shared_ptr<pddl::Type>)addFn = addElement<std::shared_ptr<pddl::Type>>;
		readSection<std::shared_ptr<pddl::Type>, pddl::TypeHasher, std::shared_ptr<pddl::Type>>("individual types", allValues, configDir, config, m_types,
				[&] (const std::string& name) -> std::shared_ptr<pddl::Type>
				{
					return m_domain->m_types[name];
				}, addFn);

		readSection<std::shared_ptr<pddl::Type>, pddl::TypeHasher, std::shared_ptr<pddl::Type>>("typenames", allValues, configDir, config, m_typenames,
				[&] (const std::string& name) -> std::shared_ptr<pddl::Type>
				{
					return m_domain->m_types[name];
				}, addFn);

		m_typenames.insert(pddl::DefaultTypes::objectType());

		for (auto& it : m_types)
		{
			LOG_INFO("Found type " << it->getName());
		}

		for (auto& it : m_typenames)
		{
			LOG_INFO("Found typename " << it->getName());
		}
	}

	//functions
	{
		std::unordered_set<std::shared_ptr<pddl::Function>, pddl::FunctionHasher> allValues;

		AddListFnDecl(pddl::Function)addFn = addList;

		for (auto& it : *m_domain->m_functions)
			for (auto& it2 : it.second)
				if (!it2->isBuiltin())
					allValues.insert(it2);
		readSection<std::shared_ptr<pddl::Function>, pddl::FunctionHasher, std::vector<std::shared_ptr<pddl::Function>>>("functions", allValues, configDir, config, m_functions,
				[&] (const std::string& name) -> std::vector<std::shared_ptr<pddl::Function>>
				{
					return m_domain->m_functions->operator [](name);
				}, addFn);

		for (auto& it : m_functions)
		{
			LOG_INFO("Found function " << it->getName());
		}
	}

	//function translations
	{
		std::vector<std::pair<std::string, std::string>> items = config.items("function translations");
		for (auto& it : items)
		{
			m_functionTranslations[it.first] = it.second;
		}
	}

	//feature translations
	{
		std::vector<std::pair<std::string, std::string>> items = config.items("feature translations");
		for (auto& it : items)
		{
			m_featureTranslations[it.first] = it.second;
		}
	}

	//predicates
	{
		std::unordered_set<std::shared_ptr<pddl::Function>, pddl::FunctionHasher> allValues;
		std::unordered_set<std::shared_ptr<pddl::Function>, pddl::FunctionHasher> preds;

		AddListFnDecl(pddl::Function)addFn = addList;

		for (auto& it : *m_domain->m_predicates)
			for (auto& it2 : it.second)
				if (!it2->isBuiltin())
					allValues.insert(it2);
		readSection<std::shared_ptr<pddl::Function>, pddl::FunctionHasher, std::vector<std::shared_ptr<pddl::Function>>>("functions", allValues, configDir, config, preds,
				[&] (const std::string& name) -> std::vector<std::shared_ptr<pddl::Function>>
				{
					return m_domain->m_predicates->operator [](name);
				}, addFn);

		for (auto& it : preds)
			m_functions.insert(it);

		for (auto& it : preds)
		{
			LOG_INFO("Found predicate " << it->getName());
		}
	}

	//actions
	{
		std::unordered_set<std::string> allValues;
		for (auto& a2 : m_domain->m_actions)
			allValues.insert(a2->m_name);

		AddFnDecl(std::string)addFn = addString;
		std::unordered_set<std::string> actions;
		readSection<std::string, std::hash<std::string>, std::string>("actions", allValues, configDir, config, actions,
				[&] (const std::string& name) -> std::string
				{
					return name;
				}, addFn);

		for (auto& it : actions)
		{
			m_actions.insert(m_domain->getAction(it));
			LOG_INFO("Found action " << it);
		}
	}

	return true;
}

void ReferenceList::buildObjectReferences()
{
	for (auto& obj : m_objects)
	{
		if (CONTAINS(obj, m_individuals))
		{
			m_namedGroup.push_back(std::shared_ptr<Reference>(new IdentityReference(obj)));
			m_namedObjects.insert(obj);
		}
		else
		{
			for (auto& t : m_types)
			{
				if (obj->isInstanceOf(t))
				{
					m_namedGroup.push_back(std::shared_ptr<Reference>(new IdentityReference(obj)));
					m_namedObjects.insert(obj);
					break;
				}
			}
		}
	}

	if (!m_namedGroup.empty())
	{
		auto vec = m_namedGroup;
		vec.push_back(std::shared_ptr<OtherReference>(new OtherReference(m_namedGroup)));
		m_groups.push_back(vec);
	}

	buildTypeGroups();

	for (auto& t : m_typenames)
	{
		m_typeGroups.push_back(ReferenceVector { std::shared_ptr<Reference>(new TypenameReference(t)) });
	}

	for (auto& t : m_typeGroups)
	{
		m_groups.push_back(t);
	}

	createSimplePartition();
}

void ReferenceList::buildTypeGroups()
{
	std::unordered_map<std::shared_ptr<pddl::Type>, pddl::TypeUnorderedSet, pddl::TypeHasher> typeTree;
	pddl::TypeUnorderedSet populated;

	for (auto& t : m_domain->m_types)
		if (t.second->t() == pddl::Types::Type)
			for (auto& sup : t.second->m_supertypes)
				typeTree[sup].insert(t.second);

	for (auto& o : m_objects)
		populated.insert(o->getType());

	//collect children lambda
	auto comp = [this](const std::shared_ptr<pddl::Type>& type) -> bool
	{
		return CONTAINS(type, m_typenames);
	};

	//remove subtypes
	auto removeSubtypes = [](const pddl::TypeUnorderedSet& children,
			pddl::TypeUnorderedSet& result) -> void
	{
		for (auto& t: children)
		{
			bool isSub = false;
			for (auto& t2: children)
			{
				if (t->isSubtypeOf(t2))
				{
					isSub = true;
					break;
				}
			}

			if (!isSub)
			{
				result.insert(t);
			}
		}
	};

	for (auto& t : m_domain->m_types)
	{
		pddl::TypeUnorderedSet children;
		collectChildren(t.second, children, typeTree, populated);

		if (children.empty())
			continue;

		if (pddl::all<std::shared_ptr<pddl::Type>>(children, comp))
		{
			if (CONTAINS_NOT(t.second, populated))
			{
				ReferenceVector group;
				pddl::TypeUnorderedSet types;
				removeSubtypes(children, types);
				for (auto& c : types)
				{
					group.push_back(std::shared_ptr<TypenameReference>(new TypenameReference(c)));
				}
//				LOG_INFO("Add type name refs of " << t.first);
//				LOG_INFO_CONTAINER_STR(group);
				m_typeGroups.push_back(group);
			}
			else if (CONTAINS(t.second, m_typenames))
			{
				ReferenceVector refs, otherRefs;

				pddl::TypeUnorderedSet relevantChildren;
				removeSubtypes(children, relevantChildren);
				for (auto& c : relevantChildren)
				{
					refs.push_back(std::shared_ptr<TypenameReference>(new TypenameReference(c)));
				}

				refs.push_back(std::shared_ptr<OtherReference>(new OtherReference(refs)));

//				LOG_INFO("Add type name refs of " << t.first);
//				LOG_INFO_CONTAINER_STR(refs);
				m_typeGroups.push_back(refs);
			}
		}
		else
		{
			if (CONTAINS(t.second, m_typenames))
			{
				ReferenceVector refs, otherRefs;

				pddl::TypeUnorderedSet relevantChildren;
				pddl::TypeUnorderedSet childrenInTypenames;
				for (auto& c : children)
				{
					if (CONTAINS(c, m_typenames))
					{
						childrenInTypenames.insert(c);
					}
				}
				removeSubtypes(childrenInTypenames, relevantChildren);
				for (auto& c : relevantChildren)
				{
					refs.push_back(std::shared_ptr<TypenameReference>(new TypenameReference(c)));
				}
				refs.push_back(std::shared_ptr<OtherReference>(new OtherReference(refs)));

//				LOG_INFO("Add type name refs of " << t.first);
//				LOG_INFO_CONTAINER_STR(refs);
				m_typeGroups.push_back(refs);
			}
		}
	}
}

void ReferenceList::createSimplePartition()
{
	pddl::FunctionUnorderedSet filteredFunctions;
	for (auto& f : m_functions)
	{
		if (f->getType() == pddl::DefaultTypes::booleanType() && f->getArgs().size() == 1)
			filteredFunctions.insert(f);
	}

	std::unordered_map<std::shared_ptr<pddl::Function>, pddl::TypedObjectUnorderedSet, pddl::FunctionHasher> values;
	for (auto& f : filteredFunctions)
	{
		auto& set = values[f];
		pddl::TypedObjectUnorderedSet objects;
		m_problem->getAllObjects(f->getArgs()[0]->getType(), objects);
		for (auto& o : objects)
		{
			std::shared_ptr<pddl::StateVariable> svar(new pddl::StateVariable(f, { o }));
			if (m_init->contains(svar))
			{
				set.insert(o);
			}
		}
	}

	pddl::TypedObjectUnorderedSet used;
	ReferenceVector group;

	for (auto& it : values)
	{
		auto& f = it.first;
		auto& val = it.second;
		if (pddl::unordered_helpers::makeIntersection(val, used).empty())
		{
			std::shared_ptr<PartitionEntry> value = PartitionEntry::fromReference(std::shared_ptr<IdentityReference>(
					new IdentityReference(pddl::DefaultTypedObjects::trueObject())), shared_from_this());
			group.push_back(std::shared_ptr<FeatureReference>(new FeatureReference(f, value, shared_from_this())));
			used = pddl::unordered_helpers::makeUnion(used, val);
		}
	}

	for (auto& it : group)
		LOG_INFO(it->str());

	if (group.size() > 1)
	{
		group.push_back(std::shared_ptr<OtherReference>(new OtherReference(group)));
		m_simpleGroup = group;
	}
	else
	{
		m_simpleGroup.clear();
	}
}

void ReferenceList::createAtomicPartitions()
{
	m_typePartitions.clear();
	m_typeInformation.clear();

	for (auto& t : m_domain->m_types)
	{
		pddl::TypedObjectUnorderedSet objects;
		m_problem->getAllObjects(t.second, objects);
		std::shared_ptr<PartitionEntry> tp = PartitionEntry::fromReference(std::shared_ptr<TypenameReference>(new TypenameReference(t.second)), shared_from_this());

		std::vector<std::vector<std::shared_ptr<Reference>>> groups;
		generateTypeGroups(objects, groups);
		generateRelationalGroups(objects, groups);
		generateNamedGroups(objects, groups);

		for (auto& g : groups)
		{
//			LOG_INFO_CONTAINER_STR(g);
			std::shared_ptr<Partition> p(new Partition());
			p->init(g, tp, Partition::Init, shared_from_this());
			if ((p->otherCount() == 0 || Constants::ALLOW_OTHER) && p->isProper() && p->information() < 0)
			{
				auto& typePartitions = m_typePartitions[t.second];
				if (std::find(typePartitions.begin(), typePartitions.end(), p) == typePartitions.end())
				{
					typePartitions.push_back(p);
				}
			}
		}
//		LOG_INFO("---");
	}

	for (auto& tp : m_typePartitions)
	{
		auto& t = tp.first;
		auto& partitions = tp.second;

		std::sort(partitions.begin(), partitions.end(), SortInformationGain());

		pddl::TypedObjectUnorderedSet objects;
		m_problem->getAllObjects(t, objects);

		int c = 0;
		std::vector<std::vector<std::shared_ptr<Reference>>> allRefs;
		for (auto& p : partitions)
		{
			std::vector<std::shared_ptr<Reference>> refs;
			if (c >= Constants::COMBINATION_OF_NO_BEST_PARTITIONS)
				break;

			for (auto& c : p->getChildren())
			{
				refs.push_back(c->getRef());
			}
			allRefs.push_back(refs);
			++c;
		}

		std::vector<int> counts;

		std::vector<std::vector<std::shared_ptr<Reference>>> productSet;
		helpers::ProductVector<std::shared_ptr<Reference>>::compute(allRefs, productSet);
		for (auto& refs : productSet)
		{
			int counter = 0;
			for (auto& o : objects)
			{
				std::function<bool(const std::shared_ptr<Reference>&)> f = [&o](const std::shared_ptr<Reference>& r)
				{
					return r->matches(o);
				};

				if (pddl::all(refs, f))
				{
					++counter;
				}
			}

			if (counter > 0)
			{
				counts.push_back(counter);
			}
		}

		//TODO Performance: total is always the same as |objects|!!!
		double total = std::accumulate(counts.begin(), counts.end(), 0);
		double I = 0;
		for (auto& c : counts)
		{
			I += -((double) c / total) * log2((double) c / total);
		}
		m_typeInformation[t].val = I;
	}

//	LOG_INFO("TypePartitions:");
//	for (auto& it2 : m_typePartitions)
//	{
//		LOG_INFO("=> type: " << it2.first);
//		LOG_INFO_CONTAINER_STR(it2.second);
//	}

	createConnectionGraph();
}

void ReferenceList::createExtendedPartitions()
{
	std::unordered_map<pddl::TypePtr, std::vector<pddl::TypedObjectUnorderedSet>, pddl::TypeHasher> partitions;
	for (auto& t : m_typenames)
	{
		__getInitial(t, partitions[t]);
//		LOG_INFO(t << " " << partitions[t].size());
	}

	partitions[pddl::DefaultTypes::booleanType()] =
	{
		{	pddl::DefaultTypedObjects::trueObject()},
		{	pddl::DefaultTypedObjects::falseObject()}};

	for (auto& it : m_domain->m_types)
	{
		auto& t = it.second;
		if (CONTAINS_NOT(t, partitions) && t != pddl::DefaultTypes::anyType())
		{
			__completePartition(t, partitions);
		}
	}

	std::queue<pddl::TypePtr> open;
	for (auto& it : m_domain->m_types)
		open.push(it.second);

	while (!open.empty())
	{
		pddl::TypePtr t = open.front();
		open.pop();
		for (auto& it : m_connections[t])
		{
			auto& t2 = it.first;
			auto& arcs = it.second;

			if (CONTAINS_NOT(t2, m_typenames))
				continue;

			for (auto& it2 : arcs)
			{
				auto& p = __completePartition(t, partitions);
				auto& p2 = __completePartition(t2, partitions);
				std::vector<pddl::TypedObjectUnorderedSet> pnew;
				join(p, p2, it2.f, it2.i, it2.j, pnew);
				if (pnew.size() > p.size())
				{
					partitions[t] = pnew;
					for (auto& it : m_connections)
					{
						auto& t3 = it.first;
						auto& d = it.second;
						if (CONTAINS(t, d))
						{
							open.push(t3);
						}
					}
				}
			}
		}
	}

	LOG_INFO("results:");
	for (auto& it : partitions)
	{
		std::string text;
		for (auto& it2 : it.second)
		{
			for (auto& it3 : it2)
			{
				text += it3->str() += ", ";
			}
		}
		if (!text.empty())
		{
			text.pop_back();
			text.pop_back();
		}
//		LOG_INFO(it.first << ": " << text);
	}

	m_bestPartitions = partitions;
}

void ReferenceList::createOptimisticPartitions()
{
	m_bestFunctionPartitions.clear();

	for (auto& f : m_functions)
	{
		pddl::TypeVector args = f->getArgTypes();
		args.push_back(f->getType());
		m_bestFunctionPartitions[f].resize(args.size());
		for (size_t i = 0; i < args.size(); ++i)
		{
			auto& t = args[i];
			if (t == pddl::DefaultTypes::booleanType())
			{
				continue;
			}

			std::vector<pddl::TypedObjectUnorderedSet> p1(1);
			m_problem->getAllObjects(t, p1[0]);
			auto& original = p1[0];
//			LOG_INFO("P1: " << p1.size());

			for (size_t j = 0; j < args.size(); ++j)
			{
				auto& t2 = args[j];
				if (i == j || t2 == pddl::DefaultTypes::booleanType())
				{
					continue;
				}
				auto& p2 = __getPartition(t2);
//				LOG_INFO("P2: " << p2.size());
				std::vector<pddl::TypedObjectUnorderedSet> p1New;
				join(p1, p2, f, i, j, p1New);
				p1 = p1New;
//				LOG_INFO("Joining: " << p1.size());
			}

			pddl::TypedObjectUnorderedSet used;
			for (auto& it : p1)
				used.insert(it.begin(), it.end());

//			LOG_INFO(used.size() << " " << original.size())
			if (pddl::unordered_helpers::isSubset(used, original))
			{
				p1.push_back(pddl::unordered_helpers::minus(original, used));
			}

			m_bestFunctionPartitions[f][i] = p1;

//			LOG_INFO(f << " " << i);
//			for (auto& p : p1)
//			{
//				for (auto& it : p)
//				{
//					LOG_INFO("\t - " << it);
//				}
//			}
		}

//		LOG_INFO("Found " << m_bestFunctionPartitions[f].size() << " optimistic partitions for " << f->str())
	}
}

void ReferenceList::createConnectionGraph()
{
	for (auto& f : m_functions)
	{
		pddl::TypeVector allTypes = f->getArgTypes();
		allTypes.push_back(f->getType());

		for (size_t i = 0; i < allTypes.size(); ++i)
		{
			auto& t = allTypes[i];
			if (t == pddl::DefaultTypes::booleanType())
			{
				continue;
			}

			pddl::TypeVector tSubtypes;
			__getSubtypes(t, tSubtypes);

			for (size_t j = 0; j < allTypes.size(); ++j)
			{
				auto& t2 = allTypes[j];
				if (i == j || t2 == pddl::DefaultTypes::booleanType())
				{
					continue;
				}

				double H, I, I1, I2;
				int count;
				__information(f, i, j, allTypes, H, I, I1, I2, count);

				if (count > 0)
				{
					pddl::TypeVector t2Subtypes;
					__getSubtypes(t2, t2Subtypes);
					for (auto& sub2 : t2Subtypes)
					{
						auto& conn = m_connections[sub2];
						for (auto& sub1 : tSubtypes)
						{
							conn[sub1].push_back(GraphContent { H, f, j, i });
//							LOG_INFO("f: " << f << ", sub2: " << sub2 << ", sub1: " << sub1 << ", H: " << H
//									<< ", I: " << I << ", I1: " << I1 << ", I2: " << I2 << ", count: " << count);
						}
					}
				}
			}
		}
	}

	for (auto& d : m_connections)
	{
		for (auto& args : d.second)
		{
			std::sort(args.second.begin(), args.second.end(),
					[](const GraphContent& a,
							const GraphContent& b)
					{
						return a.H < b.H;
					});
		}
	}
}

void ReferenceList::getTuplesFromFunction(const std::shared_ptr<pddl::Function>& f,
		const size_t& i,
		const size_t& j,
		std::vector<std::pair<pddl::TypedObjectPtr, pddl::TypedObjectPtr>>& result)
{
	for (auto& fact : m_init->iterfacts())
	{
		if (fact->getSvar()->getFunction() == f)
		{
			auto& args = fact->allArgs();
			result.push_back(std::make_pair(args[i], args[j]));
		}
	}
}

void ReferenceList::generateTypeGroups(const pddl::TypedObjectUnorderedSet& objects,
		std::vector<std::vector<std::shared_ptr<Reference>>>& result)
{
	pddl::TypeUnorderedSet types;
	for (auto& o : objects)
		types.insert(o->getType());

//	LOG_INFO("-------------------------");
//	LOG_INFO("Types: ");
//	LOG_INFO_CONTAINER_STR(types);

	for (auto& g : m_typeGroups)
	{
		if (g.size() <= 1)
			continue;

//		LOG_INFO("Type group: ");
//		LOG_INFO_CONTAINER_STR(g);

		ReferenceVector g2;
		for (auto& ref : g)
		{
			if (isInstanceSharedCast(ref, refType, TypenameReference))
			{
//				LOG_INFO("typeref found: " << ref->str());
				auto comp = [&refType](const std::shared_ptr<pddl::Type>& type)
				{
					return refType->getType()->equalOrSupertypeOf(type);
				};

				if (pddl::all<std::shared_ptr<pddl::Type>>(types, comp))
				{
//					LOG_INFO("all");
					g2.clear();
					break;
				}
				else
				if (pddl::any<std::shared_ptr<pddl::Type>>(types, comp))
				{
//					LOG_INFO("any");
					g2.push_back(ref);
				}
			}
		}

		if (!g2.empty())
		{
//			LOG_INFO("add:");
//			LOG_INFO_CONTAINER_STR(g2);
			g2.push_back(std::shared_ptr<OtherReference>(new OtherReference(g2)));
			result.push_back(g2);
		}
	}
}

void ReferenceList::generateRelationalGroups(const pddl::TypedObjectUnorderedSet& objects,
		std::vector<std::vector<std::shared_ptr<Reference>>>& result)
{
	for (auto& f : m_functions)
	{
		std::vector<std::shared_ptr<pddl::Type>> allTypes;
		for (auto& a : f->getArgs())
			allTypes.push_back(a->getType());
		allTypes.push_back(f->getType());

		for (size_t i = 0; i < allTypes.size(); ++i)
		{
			auto& t = allTypes[i];
			if (!pddl::any<std::shared_ptr<pddl::TypedObject>>(objects, [&t](const std::shared_ptr<pddl::TypedObject>& o)
			{	return t->isCompatible(o->getType());}))
			{
				continue;
			}

			ReferenceVector group;
			getRelationalGroup(f, i, allTypes, group);
			result.push_back(group);
		}

		if (!m_simpleGroup.empty())
		{
			result.push_back(m_simpleGroup);
		}
	}
}

void ReferenceList::getRelationalGroup(const std::shared_ptr<pddl::Function>& f,
		const int& i,
		const std::vector<std::shared_ptr<pddl::Type>>& allTypes,
		ReferenceVector& group)
{
	assert(i < allTypes.size());
	std::vector<std::shared_ptr<PartitionEntry>> partitionArgs;
	std::vector<std::shared_ptr<PartitionEntry>> values;

	for (size_t k = 0; k < i; ++k)
	{
		auto& t = allTypes[k];
		partitionArgs.push_back(PartitionEntry::fromReference(
				std::shared_ptr<TypenameReference>(new TypenameReference(t)), shared_from_this()));
	}

	const int argSize = f->getArgs().size();
//	LOG_INFO(f->str() << " " << argSize << " " << i)
	if (i < argSize)
	{
		partitionArgs.push_back(std::shared_ptr<PartitionEntry>()); //None
		for (size_t k = i; k < argSize; ++k)
		{
			auto& t = allTypes[k];
			partitionArgs.push_back(PartitionEntry::fromReference(
					std::shared_ptr<TypenameReference>(new TypenameReference(t)), shared_from_this()));
		}
		if (f->getType() == pddl::DefaultTypes::booleanType())
		{
			for (auto& v : { pddl::DefaultTypedObjects::trueObject(), pddl::DefaultTypedObjects::falseObject() })
			{
				values.push_back(PartitionEntry::fromReference(
						std::shared_ptr<IdentityReference>(new IdentityReference(v)), shared_from_this()));
			}
		}
		else
		{
//			LOG_INFO("values ");
//			LOG_INFO_CONTAINER_STR(allTypes)
			values.push_back(PartitionEntry::fromReference(
					std::shared_ptr<TypenameReference>(new TypenameReference(allTypes.back())), shared_from_this()));
		}
	}

	if (values.empty())
	{
//		LOG_INFO("Create Function " << f->str())
		group.push_back(std::shared_ptr<FunctionReference>(new FunctionReference(f, partitionArgs, shared_from_this())));
	}
	else if (argSize == 1)
	{
		for (auto& v : values)
		{
//			LOG_INFO("Create Feature " << f->str() << " " << v->str())
			group.push_back(std::shared_ptr<FeatureReference>(new FeatureReference(f, v, shared_from_this())));
		}
	}
	else
	{
		for (auto& v : values)
		{
//			LOG_INFO("Create Relational " << f->str() << " " << v->str())
			group.push_back(std::shared_ptr<RelationalReference>(new RelationalReference(f, partitionArgs, v, shared_from_this())));
		}
	}

	group.push_back(std::shared_ptr<OtherReference>(new OtherReference(group)));
}

pddl::TypeVector ReferenceList::getDirectSubtypes(const pddl::TypePtr& t) const
		{
	pddl::TypeVector res;
	for (auto& it : m_domain->m_types)
	{
		auto& t2 = it.second;
		if (t2->isDirectSupertype(t))
		{
			res.push_back(t2);
		}
	}
	return res;
}

void ReferenceList::join(const std::vector<pddl::TypedObjectUnorderedSet>& p1,
		const std::vector<pddl::TypedObjectUnorderedSet>& p2,
		const pddl::FunctionPtr& f,
		const size_t& i,
		const size_t& j,
		std::vector<pddl::TypedObjectUnorderedSet>& res)
{
	std::unordered_map<pddl::TypedObjectPtr, pddl::TypedObjectUnorderedSet, pddl::TypedObjectHasher> buckets;
	std::vector<std::pair<pddl::TypedObjectPtr, pddl::TypedObjectPtr>> result;
	getTuplesFromFunction(f, i, j, result);
	for (auto& it : result)
	{
		buckets[it.second].insert(it.first);
	}

	std::vector<pddl::TypedObjectUnorderedSet> image2;
	image2.resize(p2.size());
	int c = 0;
	for (auto& s : p2)
	{
		auto& set = image2[c++];
		//union
		for (auto& aj : s)
		{
			auto& b = buckets[aj];
			set.insert(b.begin(), b.end());
		}
	}

//	LOG_INFO("image2")
//	for (auto& it2 : image2)
//	{
//		LOG_INFO("- Set")
//		for (auto& it3 : it2)
//		{
//			LOG_INFO("-- " << it3)
//		}
//	}

	std::vector<pddl::TypedObjectUnorderedSet> img2;
	Partition::makePartition(image2, img2);

	for (auto& s1 : p1)
	{
//		LOG_INFO("s1");
//		LOG_INFO_CONTAINER_STR(s1);
		for (auto& s2 : img2)
		{
//			LOG_INFO("s2");
//			LOG_INFO_CONTAINER_STR(s2);
			pddl::TypedObjectUnorderedSet newObjs;
			for (auto& o : s1)
			{
				if (CONTAINS(o, s2))
				{
					newObjs.insert(o);
				}
			}
			if (!newObjs.empty())
			{
//				LOG_INFO("newObjs");
//				LOG_INFO_CONTAINER_STR(newObjs);
				res.push_back(newObjs);
			}
		}
	}

}

void ReferenceList::generateNamedGroups(const pddl::TypedObjectUnorderedSet& objects,
		std::vector<std::vector<std::shared_ptr<Reference>>>& result)
{
	ReferenceVector group;
	for (auto& o : pddl::unordered_helpers::makeIntersection(m_namedObjects, objects))
	{
		group.push_back(std::shared_ptr<IdentityReference>(new IdentityReference(o)));
	}
	if (!group.empty())
	{
		group.push_back(std::shared_ptr<OtherReference>(new OtherReference(group)));
		result.push_back(group);
		return;
	}
	//result.clear();
}

bool ReferenceList::SortInformationGain::operator ()(const std::shared_ptr<Partition>& p1,
		const std::shared_ptr<Partition>& p2)
{
	return -p1->information() < -p2->information();
}

const pddl::TypedObjectUnorderedSet& ReferenceList::getObjects() const
{
	return m_objects;
}

const std::shared_ptr<pddl::Domain>& ReferenceList::getDomain() const
{
	return m_domain;
}

const std::shared_ptr<pddl::Problem>& ReferenceList::getProblem() const
{
	return m_problem;
}

const std::shared_ptr<pddl::State>& ReferenceList::getInit() const
{
	return m_init;
}

ReferenceList::BestFunctionPartitions& ReferenceList::getBestFunctionPartitions()
{
	return m_bestFunctionPartitions;
}

const pddl::ActionUnorderedSet& ReferenceList::getActions() const
{
	return m_actions;
}

void ReferenceList::__information(const std::shared_ptr<pddl::Function>& f,
		const size_t& i,
		const size_t& j,
		const pddl::TypeVector& allTypes,
		double& H,
		double& I,
		double& Ii,
		double& Ij,
		int& total)
{
	std::vector<pddl::TypedObjectUnorderedSet> allList;
	std::unordered_map<std::pair<pddl::TypedObjectPtr, pddl::TypedObjectPtr>, pddl::IntDefaultZero, pddl::TypedObjectPairHasher> tuples;
	std::unordered_map<pddl::TypedObjectPtr, pddl::IntDefaultZero, pddl::TypedObjectHasher> ivalues;
	std::unordered_map<pddl::TypedObjectPtr, pddl::IntDefaultZero, pddl::TypedObjectHasher> jvalues;

	for (auto& t : allTypes)
	{
		if (t != pddl::DefaultTypes::booleanType())
		{
			pddl::TypedObjectUnorderedSet res;
			m_problem->getAllObjects(t, res);
			allList.push_back(res);
		}
		else
		{
			allList.push_back( { pddl::DefaultTypedObjects::trueObject() });
		}
	}

	std::vector<std::pair<pddl::TypedObjectPtr, pddl::TypedObjectPtr>> tup;
	getTuplesFromFunction(f, i, j, tup);
	for (auto& it : tup)
	{
		auto& ai = it.first;
		auto& aj = it.second;
		tuples[it].val++;
		ivalues[ai].val++;
		jvalues[aj].val++;
	}

	I = 0;
	H = 0;
	total = 0;
	for (auto& it : jvalues)
		total += it.second.val;

	for (auto& it : tuples)
	{
		auto& i = it.first.first;
		auto& j = it.first.second;
		auto& c = it.second.val;
		double Pi = (double) ivalues[i].val / total;
		double Pj = (double) jvalues[j].val / total;
		double Pij = (double) c / total;
		I += Pij * log2(Pij / (Pi * Pj));
		H += Pij * log2(Pj / Pij);
	}

	Ii = 0;
	for (auto& c : ivalues)
		Ii += (double) c.second.val / total * log2((double) c.second.val / total);

	Ij = 0;
	for (auto& c : jvalues)
		Ij += (double) c.second.val / total * log2((double) c.second.val / total);
}

void ReferenceList::__getSubtypes(const pddl::TypePtr& t,
		pddl::TypeVector& subtypes)
{
	subtypes.push_back(t);
	for (auto& t2 : m_domain->m_types)
	{
		if (t2.second->isSubtypeOf(t))
			subtypes.push_back(t2.second);
	}
}

void ReferenceList::__getInitial(const pddl::TypePtr& t,
		std::vector<pddl::TypedObjectUnorderedSet>& partition)
{
//TODO: potential difference!
	partition.resize(1);
	m_problem->getAllObjects(t, partition[0]);
//	LOG_INFO(t<< " " << m_typePartitions.size() << " " << m_typePartitions[t].size());
	for (auto& p : m_typePartitions[t])
	{
		auto& children = p->getChildren();
//		LOG_INFO(t << " " << partition.size() << " " << children.size());
		std::vector<pddl::TypedObjectUnorderedSet> nextP;
		for (auto& objs : partition)
		{
//			for (auto& o: objs) {
//				LOG_INFO(o->getName());
//			}
			for (auto& c : children)
			{
//				LOG_INFO(c);
//				LOG_INFO_CONTAINER(objs);
//				LOG_INFO_CONTAINER(c->getMatches());
//				LOG_INFO_CONTAINER(pddl::unordered_helpers::makeIntersection(objs, c->getMatches()));
				nextP.push_back(pddl::unordered_helpers::makeIntersection(objs, c->getMatches()));
			}
		}

		partition.clear();
		for (auto& e : nextP)
		{
			if (!e.empty())
			{
				partition.push_back(e);
			}
		}

//		LOG_INFO(p << " " << partition.size())
//		std::string objs;
//		for (auto& it:partition) {
//			std::string r;
//			for (auto& it2:it) {
//			r += it2->getName() + ", ";
//			}
//			if (!r.empty()){
//				r.pop_back();
//				r.pop_back();
//			}
//			objs += r + "\n";
//		}
//		LOG_INFO(objs);
	}
}

std::vector<pddl::TypedObjectUnorderedSet>& ReferenceList::__completePartition(const pddl::TypePtr& t,
		std::unordered_map<pddl::TypePtr, std::vector<pddl::TypedObjectUnorderedSet>, pddl::TypeHasher>& partitions)
{
	if (CONTAINS_NOT(t, partitions))
	{
		if (isInstanceSharedCast(t, tComp, pddl::CompositeType))
		{
			std::vector<pddl::TypedObjectUnorderedSet> pnew;
			for (auto& subt: tComp->getTypes())
			{
				std::vector<pddl::TypedObjectUnorderedSet> r = __completePartition(subt, partitions);
				pnew.insert(pnew.end(), r.begin(), r.end());
			}
			partitions[t] = pnew;
		}
		else
		{
			std::vector<pddl::TypedObjectUnorderedSet> pnew;
			for (auto& t2: getDirectSubtypes(t))
			{
				std::vector<pddl::TypedObjectUnorderedSet> r = __completePartition(t2, partitions);
				pnew.insert(pnew.end(), r.begin(), r.end());
			}
			pddl::TypedObjectUnorderedSet all;
			m_problem->getAllObjects(t, all);

			pddl::TypedObjectUnorderedSet used;
			for (auto& it: pnew)
			{
				used.insert(it.begin(), it.end());
			}

			if (pddl::unordered_helpers::isSubset(used, all))
			{
				pnew.push_back(pddl::unordered_helpers::minus(all, used));
			}

			partitions[t] = pnew;
		}
	}

	return partitions[t];
}

const std::unordered_set<std::shared_ptr<pddl::Type>, pddl::TypeHasher>& ReferenceList::getTypenames() const
{
	return m_typenames;
}

const std::unordered_map<std::shared_ptr<pddl::Type>, pddl::DoubleDefaultZero>& ReferenceList::getTypeInformation() const
{
	return m_typeInformation;
}

double ReferenceList::getTypeInformation(const std::shared_ptr<pddl::Type>& type)
{
	return m_typeInformation[type].val;
}

const std::unordered_map<std::shared_ptr<pddl::Type>, std::vector<std::shared_ptr<Partition>>, pddl::TypeHasher>& ReferenceList::getTypePartitions() const
{
	return m_typePartitions;
}

const std::vector<std::shared_ptr<Partition> >& ReferenceList::getTypePartitions(const std::shared_ptr<pddl::Type>& type)
{
	return m_typePartitions[type];
}

const std::map<pddl::TypePtr, std::map<pddl::TypePtr, std::vector<GraphContent> > >& ReferenceList::getConnections() const
{
	return m_connections;
}

const std::map<pddl::TypePtr, std::vector<GraphContent> >& ReferenceList::getConnections(const pddl::TypePtr& type)
{
	return m_connections[type];
}

std::string ReferenceList::getFunctionTranslation(const pddl::FunctionPtr& func)
{
	const std::string& name = func->getName();
	if (CONTAINS(name, m_functionTranslations))
	{
		return m_functionTranslations[func->getName()] + " ";
	}
	else
	{
		return getName(func) + " = ";
	}
}

std::string ReferenceList::getFeatureTranslation(const pddl::FunctionPtr& func)
{
	const std::string& name = func->getName();
	if (CONTAINS(name, m_featureTranslations))
	{
		return m_featureTranslations[func->getName()] + " ";
	}
	else
	{
		return getName(func) + " = ";
	}
}

std::vector<pddl::TypedObjectUnorderedSet>& ReferenceList::__getPartition(const pddl::TypePtr& t)
{
	if (CONTAINS_NOT(t, m_bestPartitions))
	{
		if (isInstanceSharedCast(t, tc, pddl::CompositeType))
		{
			std::vector<pddl::TypedObjectUnorderedSet> pnew;
			for(auto& subt: tc->getTypes())
			{
				auto& t = __getPartition(subt);
				pnew.insert(pnew.end(), t.begin(), t.end());
			}
			m_bestPartitions[t] = pnew;
		}
		else
		{
			throw pddl::Exception("Only composite types in if!");
		}
	}

	return m_bestPartitions[t];
}

std::string ReferenceList::getName(const std::string& elem)
{
	std::size_t h = 0;
	pddl::hash_combine(h, elem);
	if (CONTAINS_NOT(h, m_nameDict))
	{
		throw pddl::Exception("Name " + elem + " is unknown");
	}
	return m_nameDict[h];
}

std::string ReferenceList::getImage(const std::string& elem)
{
	std::size_t h = 0;
	pddl::hash_combine(h, elem);
	if (CONTAINS_NOT(h, m_imageDict))
	{
		throw pddl::Exception("Name " + elem + " is unknown");
	}
	return m_imageDict[h];
}

}

/* namespace goal_planner_gui */

