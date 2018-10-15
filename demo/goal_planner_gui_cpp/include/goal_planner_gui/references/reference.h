/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 11, 2017
 *      Author: kuhnerd
 * 	  Filename: reference.h
 */

#ifndef H8B9E55F7_A1E4_4A55_A872_A46B560E5D39
#define H8B9E55F7_A1E4_4A55_A872_A46B560E5D39
#include <goal_planner_gui/pddl/types.h>

namespace goal_planner_gui
{

class Reference
{
public:
	enum Type
	{
		TypeReference,
		TypeRelationalReference,
		TypeFeatureReference,
		TypeFunctionReference,
		TypeIdentityReference,
		TypeOtherReference,
		TypeTypenameReference
	};

	Reference();
	virtual ~Reference();

	virtual bool operator==(const Reference& rhs) const;
	virtual bool operator!=(const Reference& rhs) const;
	virtual size_t hash() const = 0;
	virtual bool semanticEqual(const std::shared_ptr<Reference>& other) const;
	virtual bool matches(const std::shared_ptr<pddl::TypedObject>& o) const;
	virtual std::string str() const;
	virtual std::string str(const std::shared_ptr<pddl::TypedObject>& obj) const = 0;
	virtual int refCount();
	virtual inline Reference::Type getReferenceType() const
	{
		return TypeReference;
	}
};

HASH_AND_COMPARISON_OPERATOR(Reference);

} /* namespace goal_planner_gui2 */

#endif /* H8B9E55F7_A1E4_4A55_A872_A46B560E5D39 */
