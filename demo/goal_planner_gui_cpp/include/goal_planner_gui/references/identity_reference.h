/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 11, 2017
 *      Author: kuhnerd
 * 	  Filename: identity_reference.h
 */

#ifndef H8B28B983_8FBF_45BE_9EEB_58FF0539117D
#define H8B28B983_8FBF_45BE_9EEB_58FF0539117D

#include <goal_planner_gui/references/reference.h>

namespace goal_planner_gui
{

class IdentityReference: public goal_planner_gui::Reference
{
public:
	IdentityReference(std::shared_ptr<pddl::TypedObject> obj);
	virtual ~IdentityReference();

	virtual size_t hash() const;
	virtual bool matches(const std::shared_ptr<pddl::TypedObject>& o) const;
	virtual std::string str(const std::shared_ptr<pddl::TypedObject>& obj) const;
	const std::shared_ptr<pddl::TypedObject>& getObj() const;
	void setObj(const std::shared_ptr<pddl::TypedObject>& obj);

	virtual inline Reference::Type getReferenceType() const
	{
		return TypeIdentityReference;
	}

protected:
	std::shared_ptr<pddl::TypedObject> m_obj;
};

} /* namespace goal_planner_gui2 */

#endif /* H8B28B983_8FBF_45BE_9EEB_58FF0539117D */
