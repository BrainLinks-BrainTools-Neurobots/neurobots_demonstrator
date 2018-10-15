/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 11, 2017
 *      Author: kuhnerd
 * 	  Filename: typename_reference.h
 */

#ifndef H1DEF05D7_5A86_4D20_AA05_85DA7E0E825B
#define H1DEF05D7_5A86_4D20_AA05_85DA7E0E825B

#include <goal_planner_gui/references/reference.h>

namespace goal_planner_gui
{

class TypenameReference: public Reference
{
public:
	TypenameReference(std::shared_ptr<pddl::Type> type);
	virtual ~TypenameReference();

	virtual size_t hash() const;
	virtual bool semanticEqual(const std::shared_ptr<Reference>& other) const;
	virtual bool matches(const std::shared_ptr<pddl::TypedObject>& o) const;
	virtual std::string str(const std::shared_ptr<pddl::TypedObject>& obj) const;
	const std::shared_ptr<pddl::Type>& getType() const;
	virtual inline Reference::Type getReferenceType() const
	{
		return TypeTypenameReference;
	}

protected:
	std::shared_ptr<pddl::Type> m_type;
};

} /* namespace goal_planner_gui2 */

#endif /* H1DEF05D7_5A86_4D20_AA05_85DA7E0E825B */
