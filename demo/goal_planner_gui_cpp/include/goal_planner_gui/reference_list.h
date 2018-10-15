/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 26, 2017
 *      Author: kuhnerd
 * 	  Filename: reference_list.h
 */

#ifndef H5A5FE532_6DE4_4EDC_9A59_C18CBA2B92FF
#define H5A5FE532_6DE4_4EDC_9A59_C18CBA2B92FF
#include <boost/algorithm/string/split.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <goal_planner_gui/config_parser.h>
#include <goal_planner_gui/pddl/actions.h>
#include <goal_planner_gui/pddl/domain.h>
#include <goal_planner_gui/pddl/function_table.h>
#include <goal_planner_gui/pddl/functions.h>
#include <goal_planner_gui/pddl/problem.h>
#include <goal_planner_gui/pddl/state/state.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/structs.h>
#include <map>
#include <memory>

namespace goal_planner_gui
{

class Reference;
class Partition;

class ReferenceList: public std::enable_shared_from_this<ReferenceList>
{
public:
	typedef std::unordered_map<std::shared_ptr<pddl::Function>, std::vector<std::vector<pddl::TypedObjectUnorderedSet>>, pddl::FunctionHasher> BestFunctionPartitions;

public:
	ReferenceList(std::shared_ptr<pddl::Problem> problem);
	virtual ~ReferenceList();

	void init(const std::string& filename);

	void createAtomicPartitions();
	void createExtendedPartitions();
	void createOptimisticPartitions();

	void generateTypeGroups(const pddl::TypedObjectUnorderedSet& objects,
			std::vector<std::vector<std::shared_ptr<Reference>>>& result);
	void generateNamedGroups(const pddl::TypedObjectUnorderedSet& objects,
			std::vector<std::vector<std::shared_ptr<Reference>>>& result);
	void getRelationalGroup(const std::shared_ptr<pddl::Function>& f,
			const int& i,
			const std::vector<std::shared_ptr<pddl::Type>>& allTypes,
			std::vector<std::shared_ptr<Reference>>& group);

	const pddl::TypedObjectUnorderedSet& getObjects() const;
	const std::shared_ptr<pddl::Domain>& getDomain() const;
	const std::shared_ptr<pddl::Problem>& getProblem() const;
	const std::shared_ptr<pddl::State>& getInit() const;
	BestFunctionPartitions& getBestFunctionPartitions();
	pddl::TypeVector getDirectSubtypes(const pddl::TypePtr& t) const;
	const pddl::ActionUnorderedSet& getActions() const;
	const std::unordered_set<std::shared_ptr<pddl::Type>, pddl::TypeHasher>& getTypenames() const;

	const std::unordered_map<std::shared_ptr<pddl::Type>, pddl::DoubleDefaultZero>& getTypeInformation() const;
	double getTypeInformation(const std::shared_ptr<pddl::Type>& type);
	const std::unordered_map<std::shared_ptr<pddl::Type>, std::vector<std::shared_ptr<Partition> >, pddl::TypeHasher>& getTypePartitions() const;
	const std::vector<std::shared_ptr<Partition> >& getTypePartitions(const std::shared_ptr<pddl::Type>& type);
	const std::map<pddl::TypePtr, std::map<pddl::TypePtr, std::vector<GraphContent> > >& getConnections() const;
	const std::map<pddl::TypePtr, std::vector<GraphContent> >& getConnections(const pddl::TypePtr& type);
	std::string getFunctionTranslation(const pddl::FunctionPtr& func);
	std::string getFeatureTranslation(const pddl::FunctionPtr& func);
	std::string getName(const std::string& elem);
	template<class T>
	std::string getName(const T& elem)
	{
		std::size_t h = elem->hash();
		if (CONTAINS_NOT(h, m_nameDict))
		{
			throw pddl::Exception("Name " + elem->getName() + " is unknown");
		}
		return m_nameDict[h];
	}

	std::string getImage(const std::string& elem);
	template<class T>
	std::string getImage(const T& elem)
	{
		std::size_t h = elem->hash();
		if (CONTAINS_NOT(h, m_imageDict))
		{
			throw pddl::Exception("Name " + elem->getName() + " is unknown");
		}
		return m_imageDict[h];
	}

private:
	bool readConfig(const std::string& filename);

	void buildObjectReferences();
	void buildTypeGroups();
	void createSimplePartition();
	void join(const std::vector<pddl::TypedObjectUnorderedSet>& p1,
			const std::vector<pddl::TypedObjectUnorderedSet>& p2,
			const pddl::FunctionPtr& f,
			const size_t& i,
			const size_t& j,
			std::vector<pddl::TypedObjectUnorderedSet>& res);

	void createConnectionGraph();

	void getTuplesFromFunction(const std::shared_ptr<pddl::Function>& f,
			const size_t& i,
			const size_t& j,
			std::vector<std::pair<pddl::TypedObjectPtr, pddl::TypedObjectPtr>>& result);

	void generateRelationalGroups(const pddl::TypedObjectUnorderedSet& objects,
			std::vector<std::vector<std::shared_ptr<Reference>>>& result);

	void __information(const std::shared_ptr<pddl::Function>& f,
			const size_t& i,
			const size_t& j,
			const pddl::TypeVector& allTypes,
			double& H,
			double& I,
			double& Ii,
			double& Ij,
			int& total);
	void __getSubtypes(const pddl::TypePtr& t,
			pddl::TypeVector& subtypes);
	void __getInitial(const pddl::TypePtr& t,
			std::vector<pddl::TypedObjectUnorderedSet>& partition);
	std::vector<pddl::TypedObjectUnorderedSet>& __completePartition(const pddl::TypePtr& t,
			std::unordered_map<pddl::TypePtr, std::vector<pddl::TypedObjectUnorderedSet>, pddl::TypeHasher>& partitions);
	std::vector<pddl::TypedObjectUnorderedSet>& __getPartition(const pddl::TypePtr& t);

	template<class Type, class TypeHasher, class ReturnValueLookupFn>
	bool readSection(const std::string& section,
			const std::unordered_set<Type, TypeHasher>& allValues,
			const std::string& configDir,
			ConfigParser& config,
			std::unordered_set<Type, TypeHasher>& result,
			std::function<ReturnValueLookupFn(const std::string& name)> lookupFn,
			std::function<void(const std::string k,
					const std::string& name,
					const std::string& image,
					const std::string& configDir,
					std::function<ReturnValueLookupFn(const std::string& name)>& lookupFn,
					std::unordered_map<std::size_t, std::string>& nameDict,
					std::unordered_map<std::size_t, std::string>& imageDict,
					std::vector<Type>& result)> addFn)
	{
		std::unordered_map<std::string, std::string> d;
		for (auto& it : config.items(section))
		{
			std::string key = it.first;
			std::string value = it.second;

			std::string name, image;

			if (value.find_first_of(":") != value.npos)
			{
				std::vector<std::string> result;
				boost::algorithm::split(result, value, boost::algorithm::is_any_of(":"));
				if (result.size() != 2)
					throw pddl::DefaultParseError("Please only provide a single : in " + value);

				name = result[0];
				image = result[1];

				if (name.empty())
					name = key;
			}
			else if (value.empty())
			{
				name = key;
				image.clear();
			}
			else
			{
				name = value;
				image.clear();
			}

			boost::algorithm::to_lower(key);
			d[key] = name;
			std::vector<Type> r;
			addFn(key, name, image, configDir, lookupFn, m_nameDict, m_imageDict, r);
			result.insert(r.begin(), r.end());
		}

		return true;
	}

private:
	struct SortInformationGain
	{
		bool operator()(const std::shared_ptr<Partition>& p1,
				const std::shared_ptr<Partition>& p2);
	};

	std::shared_ptr<pddl::Problem> m_problem;
	std::shared_ptr<pddl::Domain> m_domain;
	std::shared_ptr<pddl::State> m_init;

	std::unordered_map<std::size_t, std::string> m_nameDict;
	std::unordered_map<std::size_t, std::string> m_imageDict;

	pddl::TypedObjectUnorderedSet m_objects;
	pddl::TypedObjectUnorderedSet m_individuals;
	std::unordered_set<std::shared_ptr<pddl::Type>, pddl::TypeHasher> m_types;
	std::unordered_set<std::shared_ptr<pddl::Type>, pddl::TypeHasher> m_typenames;
	std::unordered_set<std::shared_ptr<pddl::Function>, pddl::FunctionHasher> m_functions;
	pddl::ActionUnorderedSet m_actions;
	std::unordered_map<std::shared_ptr<pddl::Type>, std::vector<std::shared_ptr<Partition>>, pddl::TypeHasher> m_typePartitions;
	std::unordered_map<std::shared_ptr<pddl::Type>, pddl::DoubleDefaultZero> m_typeInformation;
	BestFunctionPartitions m_bestFunctionPartitions;
	std::map<pddl::TypePtr, std::map<pddl::TypePtr, std::vector<GraphContent>>> m_connections;
	std::unordered_map<pddl::TypePtr, std::vector<pddl::TypedObjectUnorderedSet>, pddl::TypeHasher> m_bestPartitions;
	std::unordered_map<std::string, std::string> m_functionTranslations;
	std::unordered_map<std::string, std::string> m_featureTranslations;

	typedef std::vector<std::shared_ptr<Reference>> ReferenceVector;
	ReferenceVector m_namedGroup;
	ReferenceVector m_simpleGroup;
	std::vector<ReferenceVector> m_groups;
	std::vector<ReferenceVector> m_typeGroups;
	pddl::TypedObjectUnorderedSet m_namedObjects;
};

} /* namespace goal_planner_gui */

#endif /* H5A5FE532_6DE4_4EDC_9A59_C18CBA2B92FF */
