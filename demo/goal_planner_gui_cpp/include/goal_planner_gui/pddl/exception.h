/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 26, 2017
 *      Author: kuhnerd
 * 	  Filename: exception.h
 */

#ifndef H2AC5EA63_F97C_417C_B36B_94D62C34E9E5
#define H2AC5EA63_F97C_417C_B36B_94D62C34E9E5
#include <string>

#define FILE_AND_LINE __FILE__, __LINE__

namespace pddl
{

class Exception: public std::exception
{
public:
	Exception(const std::string& msg);
	Exception();

	virtual const char* what() const noexcept override;

	std::string m_msg;
};

class FileNotFoundExcetion: public Exception
{
public:
	FileNotFoundExcetion(const std::string& filename);
};

class NotImplementedException: public Exception
{
public:
	NotImplementedException();
	NotImplementedException(const std::string& file,
			int line);
};

class DontCallDirectlyException: public Exception
{
public:
	DontCallDirectlyException();
};

class EndOfListException: public Exception
{
public:
	EndOfListException();
};

class SizeException: public Exception
{
public:
	SizeException();
};

class NotInMapException: public Exception
{
public:
	NotInMapException(const std::string& key);
};

class DefaultParseError: public Exception
{
public:
	DefaultParseError(const std::string& msg);
};

} /* namespace pddl */

#endif /* H2AC5EA63_F97C_417C_B36B_94D62C34E9E5 */
