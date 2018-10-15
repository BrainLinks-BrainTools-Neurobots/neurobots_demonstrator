/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 26, 2017
 *      Author: kuhnerd
 * 	  Filename: exception.cpp
 */

#include <goal_planner_gui/pddl/exception.h>

namespace pddl
{

Exception::Exception(const std::string& msg) :
				m_msg(msg)
{
}

Exception::Exception() :
				m_msg("Unknown exception!")
{
}

const char* Exception::what() const noexcept
{
	return m_msg.c_str();
}

FileNotFoundExcetion::FileNotFoundExcetion(const std::string& filename) :
				Exception("File not found: " + filename)
{
}

NotImplementedException::NotImplementedException() :
				Exception("Not implemented")
{
}

NotImplementedException::NotImplementedException(const std::string& file,
		int line) :
				Exception("Not implemented: " + file + ", line " + std::to_string(line))
{
}

DontCallDirectlyException::DontCallDirectlyException() :
				Exception("Don't call this function/method (only in child classes)")
{
}

EndOfListException::EndOfListException() :
				Exception("End of list reached!")
{
}

SizeException::SizeException() :
				Exception("Size problem (e.g., different sizes)")
{
}

NotInMapException::NotInMapException(const std::string& key) :
				Exception("Element is not in map: " + key)
{
}

DefaultParseError::DefaultParseError(const std::string& msg) :
				Exception(msg)
{
}

}

/* namespace pddl */

