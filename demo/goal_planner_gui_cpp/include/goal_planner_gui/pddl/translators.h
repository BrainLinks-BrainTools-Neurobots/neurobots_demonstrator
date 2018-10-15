/*
 * translators.h
 *
 *  Created on: Oct 1, 2017
 *      Author: kuhnerd
 */

#ifndef DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_TRANSLATORS_H_
#define DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_TRANSLATORS_H_

namespace pddl
{

class Translator
{
public:
	Translator();
	virtual ~Translator();
};

class ObjectFluentCompiler: public Translator
{
public:
};

} /* namespace pddl */

#endif /* DEMO_PYDDL_CPP_INCLUDE_PYDDL_PDDLLIB_PDDL_TRANSLATORS_H_ */
