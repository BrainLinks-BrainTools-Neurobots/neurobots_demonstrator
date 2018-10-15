/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 28, 2017
 *      Author: kuhnerd
 * 	  Filename: base_visitor.h
 */

#ifndef H1E13B445_0D54_47DE_B2A1_064CEE202BB9
#define H1E13B445_0D54_47DE_B2A1_064CEE202BB9
#include <boost/any.hpp>
#include <memory>
#include <string>
#include <vector>

namespace pddl
{

class BaseVisitor
{
public:
	BaseVisitor();
	virtual ~BaseVisitor();

	virtual boost::any operator()(boost::any value,
			const std::vector<boost::any>& result) = 0;
};
} /* namespace pddl */

#endif /* H1E13B445_0D54_47DE_B2A1_064CEE202BB9 */
