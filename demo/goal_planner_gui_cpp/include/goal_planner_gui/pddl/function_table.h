/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 29, 2017
 *      Author: kuhnerd
 * 	  Filename: function_table.h
 */

#ifndef H773CDF21_1BBF_4D54_844E_84B9336CFDD6
#define H773CDF21_1BBF_4D54_844E_84B9336CFDD6
#include <goal_planner_gui/pddl/constants.h>
#include <memory>
#include <unordered_map>
#include <vector>

namespace pddl
{

class Function;
class Term;
class Type;
class TypedObject;
class Parameter;

/**
 * This class is used to store and retrieve PDDL Function objects
 * according to name and argument types.
 */
class FunctionTable
{
public:
	FunctionTable();
	FunctionTable(const std::vector<std::shared_ptr<Function>>& functions);
	virtual ~FunctionTable();

	void add(std::shared_ptr<Function> function);
	void remove(std::shared_ptr<Function> function);
	bool contains(const std::shared_ptr<Function>& key) const;
	bool contains(const std::string& key) const;

	void getAll(const std::string& name,
			const std::vector<std::shared_ptr<Term>>& args,
			std::vector<std::shared_ptr<Function>>& functions,
			int functionScope = SCOPE_ALL);
	void getAll(const std::string& name,
			const std::vector<std::shared_ptr<TypedObject>>& args,
			std::vector<std::shared_ptr<Function>>& functions,
			int functionScope = SCOPE_ALL);
	void getAll(const std::string& name,
			const std::vector<std::shared_ptr<Parameter>>& args,
			std::vector<std::shared_ptr<Function>>& functions,
			int functionScope = SCOPE_ALL);
	void getAll(const std::string& name,
			const std::vector<std::shared_ptr<Type>>& args,
			std::vector<std::shared_ptr<Function>>& functions,
			int functionScope = SCOPE_ALL);

	std::shared_ptr<Function> get(const std::string& name,
			const std::vector<std::shared_ptr<Term>>& args,
			int functionScope = SCOPE_ALL);
	std::shared_ptr<Function> get(const std::string& name,
			const std::vector<std::shared_ptr<TypedObject>>& args,
			int functionScope = SCOPE_ALL);
	std::shared_ptr<Function> get(const std::string& name,
			const std::vector<std::shared_ptr<Parameter>>& args,
			int functionScope = SCOPE_ALL);
	std::shared_ptr<Function> get(const std::string& name,
			const std::vector<std::shared_ptr<Type>>& args,
			int functionScope = SCOPE_ALL);

	size_t size() const;
	std::string str() const;

//	std::vector<std::shared_ptr<Function>> operator [](const std::shared_ptr<Function>& i) const;
	std::vector<std::shared_ptr<Function>>& operator [](const std::shared_ptr<Function>& i);

//	std::vector<std::shared_ptr<Function>> operator [](const std::string& i) const;
	std::vector<std::shared_ptr<Function>>& operator [](const std::string& i);

	std::unordered_map<std::string, std::vector<std::shared_ptr<Function>>>::iterator begin() noexcept;
	std::unordered_map<std::string, std::vector<std::shared_ptr<Function>>>::iterator end() noexcept;

	std::unordered_map<std::string, std::vector<std::shared_ptr<Function>>>::const_iterator begin() const noexcept;
	std::unordered_map<std::string, std::vector<std::shared_ptr<Function>>>::const_iterator end() const noexcept;

private:
	std::unordered_map<std::string, std::vector<std::shared_ptr<Function>>> m_functions;
};

}
/* namespace pddl */

#endif /* H773CDF21_1BBF_4D54_844E_84B9336CFDD6 */
