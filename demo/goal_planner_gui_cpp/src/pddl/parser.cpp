/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 26, 2017
 *      Author: kuhnerd
 * 	  Filename: parser.cpp
 */

#include <goal_planner_gui/pddl/parser.h>
#include <boost/algorithm/string.hpp>
#include <goal_planner_gui/pddl/utils.h>
#include <fstream>

namespace pddl
{

Token::Token(const std::string& token,
		int line,
		const std::string& file) :
				string(token),
				line(line),
				file(file)
{
}

Token::Token() :
				line(-1)
{
}

Token::~Token()
{
}

void Token::error(const std::string& msg)
{
	throw ParseError(*this, msg);
}

void Token::checkKeyword(const std::string& keyword)
{
	if (string != keyword)
		throw UnexpectedTokenError(*this, "checkKeyword: " + keyword);
}

bool Token::operator ==(const Token& rhs) const
		{
	return string == rhs.string && line == rhs.line && file == rhs.file;
}

bool Token::operator ==(const std::string& rhs) const
		{
	return string == rhs;
}

bool Token::operator !=(const Token& rhs) const
		{
	return !(*this == rhs);
}

bool Token::operator !=(const std::string& rhs) const
		{
	return !(*this == rhs);
}

bool Token::isNone()
{
	return line == -1;
}

const std::string Element::ExpectedTypeNone = "";
const std::string Element::ExpectedTypeList = "__list__";
const std::string Element::ExpectedTypeTerminal = "__terminal__";

Element::Element(const Token& token) :
				parent(NULL),
				token(token),
				terminal(true)
{
	m_iter = m_children.begin();
}

Element::Element(const Token& token,
		const std::vector<std::shared_ptr<Element>>& children) :
				parent(NULL),
				token(token),
				m_children(children),
				terminal(false)
{
	if (!m_children.empty())
		for (auto& child : m_children)
		{
			append(child);
			child->parent = shared_from_this();
		}
	m_iter = m_children.begin();
}

Element::~Element()
{
}

void Element::append(std::shared_ptr<Element> child)
{
	if (terminal)
		return;
	m_children.push_back(child);
	m_iter = m_children.begin();
}

void Element::end(const Token& token)
{
	endtoken = token;
}

bool Element::isTerminal() const
{
	return terminal;
}

std::shared_ptr<Element> Element::get(const std::string& expected,
		std::string message)
{
	if (m_children.empty())
		throw Exception("Children empty!");

	if (m_iter == m_children.end())
		throw EndOfListException();

	std::shared_ptr<Element> elem = *m_iter;
	++m_iter;

	if (expected != ExpectedTypeNone)
	{
		if (expected == ExpectedTypeList && elem->isTerminal())
		{
			if (message.empty())
				message = "'('";
			throw UnexpectedTokenError(elem->token, "expected case 1: " + message);
		}
		else if (expected == ExpectedTypeTerminal && !elem->isTerminal())
		{
			if (message.empty())
				message = "identifier";
			throw UnexpectedTokenError(elem->token, "expected case 2: " + message);
		}
		else if (expected != ExpectedTypeList && expected != ExpectedTypeTerminal && elem->token.string != expected)
		{
			if (message.empty())
				message = expected;
			throw UnexpectedTokenError(elem->token, "expected case 3: " + message);
		}
	}

	return elem;
}

std::shared_ptr<Element> Element::peek(const std::string& expected,
		std::string message,
		int count)
{
	auto currentIt = m_iter;
	for (size_t i = 0; i < count - 1; ++i)
	{
		m_iter++;
	}
	auto elem = get(expected, message);
	m_iter = currentIt;
	return elem;
}

std::string Element::str() const
{
	std::string str = token.string + " [" + std::to_string(token.line) + "]\n";
	for (auto& it : m_children)
		str += "\t" + it->token.string + " [" + std::to_string(it->token.line) + "]\n";
	str += "current iterator position: " + (*m_iter)->token.string;
	return str;
}

std::vector<std::shared_ptr<Element>>::iterator Element::current()
{
	return m_iter;
}

void Element::checkNoMoreTokens(const std::string& message)
{
	//if iterator is already at the end, return;
	if (m_iter == m_children.end())
		return;

	++m_iter;
	if (m_iter == m_children.end())
		return;

	if (message.empty())
		throw UnexpectedTokenError(token, "')'");
	else
		throw ParseError((*m_iter)->token, message);
}

std::vector<std::shared_ptr<Element>>::iterator Element::end()
{
	return m_children.end();
}

void Element::reset()
{
	m_iter = m_children.begin();
}

ParseError::ParseError(const Token& token,
		const std::string& msg)
{
	m_msg = "Error in line " + std::to_string(token.line) + " of " + token.file + ": " + msg;
}

UnexpectedTokenError::UnexpectedTokenError(const Token& token,
		const std::string& msg,
		const std::string& expected)
{
	if (!expected.empty())
		m_msg = "Expected " + expected + ", found " + token.string + ", message: " + msg;
	else
		m_msg = "Unexpected token: " + token.string + ", message: " + msg;
}

Parser::Parser(const std::vector<std::string>& lines,
		const std::string& source,
		std::vector<std::string> seperators) :
				m_seperators(seperators),
				m_source(source)
{
	m_seperators.push_back("(");
	m_seperators.push_back(")");

	std::vector<Token> tokens;
	int i = 0;

	tokenize(lines, tokens, source);

	if (tokens.empty())
		throw ParseError(Token("", 0, source), "Empty file");

	Token& token = tokens[i];
	if (token != "(")
		throw UnexpectedTokenError(token, "'('");

	m_root = parse(token, tokens, i);

	if (i < tokens.size() - 1)
		throw UnexpectedTokenError(tokens[i], "end of file");
}

Parser::~Parser()
{
}

void Parser::tokenize(const std::vector<std::string>& lines,
		std::vector<Token>& tokens,
		const std::string source)
{
	std::string line;
	for (size_t i = 0; i < lines.size(); ++i)
	{
		std::vector<std::string> strs;
		boost::split(strs, lines[i], boost::is_any_of(";"));
		if (strs.empty())
			throw Exception("Cannot split line: " + lines[i]);
		line = strs[0];
		for (auto& sep : m_seperators)
			boost::replace_all(line, sep, " " + sep + " ");
		boost::split(strs, line, boost::is_any_of("\t\n "));
		for (auto& t : strs)
		{
			boost::trim_if(t, boost::is_any_of(" \t\n"));
			if (!t.empty())
			{
				boost::to_lower(t);
				tokens.push_back(Token(t, i + 1, source));
			}
		}
	}
}

std::shared_ptr<Element> Parser::parse(const Token& head,
		const std::vector<Token>& tokens,
		int& i)
{
	std::shared_ptr<Element> element(new Element(head, { }));

	++i;
	if (i == tokens.size())
		throw ParseError(head, "No closing ')' before end of file");

	Token token = tokens[i];

	while (token != ")")
	{
		if (token == "(")
			element->append(parse(token, tokens, i));
		else
			element->append(std::shared_ptr<Element>(new Element(token)));

		++i;
		if (i == tokens.size())
			throw ParseError(head, "No closing ')' before end of file");
		token = tokens[i];
	}

	element->end(token);
	return element;
}

std::shared_ptr<Parser> Parser::parseFile(const std::string& filename,
		const std::vector<std::string>& seperators)
{
	std::ifstream file(filename);
	if (!file.is_open())
		throw FileNotFoundExcetion(filename);

	std::vector<std::string> lines;
	std::string line;
	while (std::getline(file, line))
		lines.push_back(line);

	std::shared_ptr<Parser> parser(new Parser(lines, filename, seperators));
	file.close();
	return parser;
}

const std::shared_ptr<Element>& Parser::getRoot() const
{
	return m_root;
}

size_t Token::hash() const
{
	std::size_t h = 0;
	hash_combine(h, string, line, file);
	return h;
}

} /* namespace pddl */

std::ostream& operator <<(std::ostream& stream,
		const pddl::Token& token)
{
	stream << token.string << " (line: " << token.line << ", file: " << token.file << ")";
	return stream;
}

std::ostream& operator <<(std::ostream& stream,
		const pddl::Element& elem)
{
	stream << elem.str();
	return stream;
}

