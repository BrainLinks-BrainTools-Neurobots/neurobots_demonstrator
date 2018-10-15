/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 26, 2017
 *      Author: kuhnerd
 * 	  Filename: parser.h
 */

#ifndef H76C13567_7A62_4F4B_B078_7B712BF4B1E2
#define H76C13567_7A62_4F4B_B078_7B712BF4B1E2
#include <memory>
#include <string>
#include <vector>

#include <goal_planner_gui/pddl/exception.h>
#include <unordered_map>

namespace pddl
{

class Token
{
public:
	Token(const std::string& token,
			int line,
			const std::string& file);
	Token();
	virtual ~Token();

	void error(const std::string& msg);
	void checkKeyword(const std::string& keyword);
	bool operator==(const Token& rhs) const;
	bool operator==(const std::string& rhs) const;
	bool operator!=(const Token& rhs) const;
	bool operator!=(const std::string& rhs) const;
	bool isNone();
	virtual size_t hash() const;

	std::string string;
	int line;
	std::string file;
};

class Element: public std::enable_shared_from_this<Element>
{
public:
	static const std::string ExpectedTypeNone;
	static const std::string ExpectedTypeList;
	static const std::string ExpectedTypeTerminal;

public:
	Element(const Token& token);
	Element(const Token& token,
			const std::vector<std::shared_ptr<Element>>& children);
	virtual ~Element();

	void append(std::shared_ptr<Element> child);
	void end(const Token& token);
	bool isTerminal() const;
	void checkNoMoreTokens(const std::string& message = "");

	std::shared_ptr<Element> get(const std::string& expected = ExpectedTypeNone,
			std::string message = "");
	std::shared_ptr<Element> peek(const std::string& expected = ExpectedTypeNone,
			std::string message = "",
			int count = 1);

	virtual std::string str() const;

	std::vector<std::shared_ptr<Element>>::iterator current();
	std::vector<std::shared_ptr<Element>>::iterator end();
	void reset();

	std::shared_ptr<Element> parent;
	Token token;
	Token endtoken;
	bool terminal;

	std::vector<std::shared_ptr<Element>> m_children;

private:
	std::vector<std::shared_ptr<Element>> m_peeked;
	std::vector<std::shared_ptr<Element>>::iterator m_iter;
};

class ParseError: public Exception
{
public:
	ParseError(const Token& token,
			const std::string& msg);
};

class UnexpectedTokenError: public Exception
{
public:
	UnexpectedTokenError(const Token& token,
			const std::string& msg,
			const std::string& = std::string());
};

class Parser
{
public:
	Parser(const std::vector<std::string>& lines,
			const std::string& source,
			std::vector<std::string> seperators);
	virtual ~Parser();

	void tokenize(const std::vector<std::string>& lines,
			std::vector<Token>& tokens,
			const std::string source = "");

	std::shared_ptr<Element> parse(const Token& head,
			const std::vector<Token>& tokens,
			int& i);

	const std::shared_ptr<Element>& getRoot() const;

	static std::shared_ptr<Parser> parseFile(const std::string& filename,
			const std::vector<std::string>& seperators = std::vector<std::string>());

	template<class LeftFunctionReturnValue, class RightFunctionReturnValue>
	static void parseTypedList(std::vector<std::shared_ptr<Element> >::iterator itCurrent,
			std::vector<std::shared_ptr<Element> >::iterator itEnd,
			std::function<LeftFunctionReturnValue(std::shared_ptr<Element>)> leftFunc,
			std::function<RightFunctionReturnValue(std::shared_ptr<Element>)> rightFunc,
			std::vector<std::vector<LeftFunctionReturnValue>>& leftResult,
			std::vector<RightFunctionReturnValue>& rightResult,
			const std::string expectedLeft = "identifiers",
			const std::string expectedRight = "identifiers",
			const bool rightSideRequired = false)
	{
		std::vector<LeftFunctionReturnValue> left;
		bool foundSep = false;

		for (auto it = itCurrent; it != itEnd; ++it)
		{
			auto elem = (*it);
			if (elem->token == "-")
			{
				if (left.empty() || foundSep)
					throw ParseError(elem->token, "expected " + expectedLeft + " before '-'");
				foundSep = true;
				continue;
			}

			if (foundSep)
			{
				auto right = rightFunc(elem);
				leftResult.push_back(left);
				rightResult.push_back(right);
				left.clear();
				foundSep = false;
				continue;
			}

			left.push_back(leftFunc(elem));
		}

		if (foundSep)
			throw UnexpectedTokenError((*itEnd)->token, expectedRight);

		if (!left.empty())
		{
			if (rightSideRequired)
			{
				throw UnexpectedTokenError((*itEnd)->token, "-");
			}
			else
			{
				leftResult.push_back(left);
				rightResult.push_back(RightFunctionReturnValue());
			}
		}
	}

private:
	std::vector<std::string> m_seperators;
	std::string m_source;
	std::shared_ptr<Element> m_root;
};

} /* namespace pddl */

std::ostream& operator<<(std::ostream& stream,
		const pddl::Token& token);
std::ostream& operator<<(std::ostream& stream,
		const pddl::Element& token);

namespace std
{

template<>
class hash<pddl::Token>
{
public:
	size_t operator()(const pddl::Token& token) const
			{
		return token.hash();
	}
};

}

#endif /* H76C13567_7A62_4F4B_B078_7B712BF4B1E2 */
