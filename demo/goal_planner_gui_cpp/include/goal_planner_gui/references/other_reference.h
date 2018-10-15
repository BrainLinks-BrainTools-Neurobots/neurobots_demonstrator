/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 11, 2017
 *      Author: kuhnerd
 * 	  Filename: other_reference.h
 */

#ifndef H3D330907_3D44_49DF_B4B8_29FB0D0F4F53
#define H3D330907_3D44_49DF_B4B8_29FB0D0F4F53

#include <goal_planner_gui/references/reference.h>

namespace goal_planner_gui
{

class OtherReference: public goal_planner_gui::Reference
{
public:
	OtherReference(const std::vector<std::shared_ptr<Reference>>& excludingRefs);
	virtual ~OtherReference();

	virtual size_t hash() const;
	virtual bool matches(const std::shared_ptr<pddl::TypedObject>& o) const;
	virtual std::string str(const std::shared_ptr<pddl::TypedObject>& obj) const;

	virtual inline Reference::Type getReferenceType() const
	{
		return TypeOtherReference;
	}

protected:
	std::vector<std::shared_ptr<Reference>> m_excludingRefs;
};

} /* namespace goal_planner_gui2 */

#endif /* H3D330907_3D44_49DF_B4B8_29FB0D0F4F53 */
