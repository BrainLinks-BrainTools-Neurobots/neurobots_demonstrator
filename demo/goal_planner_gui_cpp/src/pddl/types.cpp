/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 27, 2017
 *      Author: kuhnerd
 * 	  Filename: types.cpp
 */

#include <boost/algorithm/string/trim.hpp>
#include <goal_planner_gui/pddl/parser.h>
#include <goal_planner_gui/pddl/scope.h>
#include <goal_planner_gui/pddl/types.h>
#include <goal_planner_gui/pddl/utils.h>

#include <algorithm>
#include <unordered_set>

namespace pddl
{

Type::Type(const std::string& type) :
				m_name(type),
				m_hash(0)
{
	if (type != "object")
	{
		static std::shared_ptr<Type> object = DefaultTypes::objectType();
		m_supertypes.push_back(object);
	}
}

Type::Type(const std::string& type,
		const std::vector<std::shared_ptr<Type>>& supertypes) :
				m_name(type),
				m_hash(0)
{
	if (type != "object")
	{
		if (supertypes.empty())
		{
			static std::shared_ptr<Type> object = DefaultTypes::objectType();
			m_supertypes.push_back(object);
		}
		else
		{
			m_supertypes = supertypes;
		}
	}
}

Type::Type() :
				m_hash(0),
				m_name("Empty Type")
{
}

Type::~Type()
{
}

void Type::getSupertypes(std::unordered_set<std::shared_ptr<Type>, TypeHasher>& supertypes)
{
//	std::unordered_set<std::shared_ptr<Type>> types;
//	types.insert(supertypes.begin(), supertypes.end());

	for (auto& it : m_supertypes)
	{
		supertypes.insert(it);
		it->getSupertypes(supertypes);
	}
}

bool Type::isDirectSupertype(const std::shared_ptr<Type>& t)
{
	return std::find(m_supertypes.begin(), m_supertypes.end(), t) != m_supertypes.end();
}

bool Type::isSupertypeOf(const std::shared_ptr<Type>& other)
{
	return other->isSubtypeOf(shared_from_this());
}

bool Type::equalOrSupertypeOf(const std::shared_ptr<Type>& other)
{
	return other->equalOrSubtypeOf(shared_from_this());
}

bool Type::isSubtypeOf(const std::shared_ptr<Type>& other)
{
	if (t() != other->t())
		return other->isSupertypeOf(shared_from_this());

	if (std::find(m_supertypes.begin(), m_supertypes.end(), other) != m_supertypes.end())
		return true;

	for (auto& sup : m_supertypes)
		if (sup->isSubtypeOf(other))
			return true;

	return false;
}

bool Type::equalOrSubtypeOf(const std::shared_ptr<Type>& other)
{
//	LOG_INFO("called " << str() << "(" << t() << ") other: " << other->str()<< "(" << other->t() << ")");
	if (t() != other->t())
	{
		return other->equalOrSupertypeOf(shared_from_this());
	}

	if (std::find(m_supertypes.begin(), m_supertypes.end(), other) != m_supertypes.end() || *this == *other)
		return true;

	for (auto& sup : m_supertypes)
		if (sup->isSubtypeOf(other))
			return true;

	return false;
}

bool Type::isCompatible(const std::shared_ptr<Type>& other)
{
	return equalOrSubtypeOf(other) || equalOrSupertypeOf(other);
}

bool Type::operator ==(const Type& rhs) const
		{
	return t() == rhs.t() && m_name == rhs.getName();
}

bool Type::operator !=(const Type& rhs) const
		{
	return !(*this == rhs);
}

bool Type::operator <(const Type& rhs) const
		{
	return m_name < rhs.getName();
}

const std::string& Type::getName() const
{
	return m_name;
}

size_t Type::hash() const
{
	if (m_hash == 0)
	{
		std::size_t h = 0;
		hash_combine(h, t(), m_name);
		m_hash = h;
	}

	return m_hash;
}

std::string Type::str() const
{
	return m_name;
}

void Type::parseTypeList(std::vector<std::shared_ptr<Element> >::iterator itCurrent,
		std::vector<std::shared_ptr<Element> >::iterator itEnd,
		std::unordered_map<Token, Token>& types)
{
	auto checkFunc = [](std::shared_ptr<Element> elem)
	{
		if (!elem->isTerminal())
		throw UnexpectedTokenError(elem->token, "identifier");
		return elem->token;
	};

	std::vector<std::vector<Token>> leftResult;
	std::vector<Token> rightResult;
	Parser::parseTypedList<Token, Token>(itCurrent, itEnd, checkFunc, checkFunc, leftResult, rightResult);

	assert(leftResult.size() == rightResult.size());

	for (size_t i = 0; i < leftResult.size(); ++i)
	{
		auto& subtypes = leftResult[i];
		auto& super = rightResult[i];
		if (super.string == "")
			super = Token("object", 0, "");
		for (auto type : subtypes)
			types[type] = super;
	}
}

std::shared_ptr<Type> Type::parse(std::shared_ptr<Element> it,
		std::unordered_map<std::string, std::shared_ptr<Type> > types,
		std::shared_ptr<Scope> scope)
{
	if (it->isTerminal())
	{
		if (types.find(it->token.string) == types.end())
			throw ParseError(it->token, "Unknown type: '" + it->token.string + "'");
		return types[it->token.string];
	}

	Token first = it->get(Element::ExpectedTypeTerminal)->token;
	if (first.string == "either")
	{
		std::vector<std::shared_ptr<Type>> ctypes;
		for (auto it2 = it->current(); it2 != it->end(); ++it2)
			ctypes.push_back(Type::parse(it, types));
		return std::shared_ptr<Type>(new CompositeType(ctypes));
	}
	else if (first.string == "function")
	{
		std::shared_ptr<Type> ftype = Type::parse(it, types);
		it->checkNoMoreTokens();
		return std::shared_ptr<Type>(new FunctionType(ftype));
	}
	else if (first.string == "typeof" && scope)
	{
		Token param = it->get(Element::ExpectedTypeTerminal)->token;
		it->checkNoMoreTokens();
		if (!scope->contains(param.string))
			throw ParseError(param, "Unknown identifier: '" + param.string + "'");
		return std::shared_ptr<ProxyType>(new ProxyType(std::dynamic_pointer_cast<Parameter>(scope->get(param.string))));
	}

	throw DontCallDirectlyException();
}

std::shared_ptr<Type> Type::copy()
{
	return std::shared_ptr<Type>(new Type(m_name, m_supertypes));
}

CompositeType::CompositeType(const std::vector<std::shared_ptr<Type>>& types) :
				m_types(types)
{
	for (auto& it : types)
		m_name += "-" + it->getName();

	std::sort(m_types.begin(), m_types.end());
}

CompositeType::~CompositeType()
{
}

bool CompositeType::isSupertypeOf(const std::shared_ptr<Type>& other)
{
	if (other->t() == Types::CompositeType)
	{
		staticSharedPointerCast(other, o, CompositeType);
		bool strictSupertype = false;
		for (auto& t : m_types)
		{
			for (auto& t2 : o->getTypes())
			{
				strictSupertype = true;
				break;
			}
			if (strictSupertype)
				break;
		}

		if (!strictSupertype)
			return false;

		for (auto& t : m_types)
			if (!equalOrSupertypeOf(t))
				return false;

		return true;
	}

	for (auto& t : m_types)
		if (t->equalOrSupertypeOf(other))
			return true;

	return false;
}

bool CompositeType::equalOrSupertypeOf(const std::shared_ptr<Type>& other)
{
	return *this == *other || isSupertypeOf(other);
}

bool CompositeType::isSubtypeOf(const std::shared_ptr<Type>& other)
{
	if (other->t() == Types::CompositeType)
		return other->isSupertypeOf(shared_from_this());

	for (auto& sup : m_types)
		if (!sup->isSubtypeOf(other))
			return false;

	return true;
}

bool CompositeType::equalOrSubtypeOf(const std::shared_ptr<Type>& other)
{
	return *this == *other || isSubtypeOf(other);
}

bool CompositeType::operator ==(const Type& rhs) const
		{
	if (rhs.t() == Types::CompositeType)
	{
		staticPointerCast(&rhs, o, CompositeType);
		const auto& otherTypes = o->getTypes();
		if (m_types.size() != otherTypes.size())
		{
			return false;
		}
		else
		{
			for (size_t i = 0; i < m_types.size(); ++i)
				if (m_types[i] != otherTypes[i])
					return false;
			return true;
		}
	}
	return false;
}

size_t CompositeType::hash() const
{
	if (m_hash == 0)
	{
		std::size_t h = 0;
		hash_combine(h, t());
		for (auto& it : m_types)
			hash_combine(h, it->hash());
		m_hash = h;
	}
	return m_hash;
}

std::string CompositeType::str() const
{
	std::string joined;
	for (auto& it : m_types)
		joined = it->getName() + " ";
	boost::algorithm::trim(joined);
	return "(either " + joined + ")";
}

std::shared_ptr<Type> CompositeType::copy()
{
	std::vector<std::shared_ptr<Type>> types;
	COPY_VECTOR(m_types, types);
	return std::shared_ptr<Type>(new CompositeType(types));
}

FunctionType::FunctionType(const std::shared_ptr<Type>& type) :
				m_type(type)
{
	m_name = "function(" + type->str() + ")";
}

FunctionType::~FunctionType()
{
}

bool FunctionType::isSupertypeOf(const std::shared_ptr<Type>& other)
{
	if (other->t() == Types::FunctionType)
	{
		staticSharedPointerCast(other, o, FunctionType);
		return m_type->isSupertypeOf(o->getType());
	}
	return false;
}

bool FunctionType::equalOrSupertypeOf(const std::shared_ptr<Type>& other)
{
	if (other->t() == Types::FunctionType)
	{
		staticSharedPointerCast(other, o, FunctionType);

		LOG_INFO(m_type << " " <<o->getType());
		return m_type->equalOrSupertypeOf(o->getType());
	}
	return false;
}

bool FunctionType::isSubtypeOf(const std::shared_ptr<Type>& other)
{
	if (other->t() == Types::FunctionType)
	{
		staticSharedPointerCast(other, o, FunctionType);
		return m_type->isSubtypeOf(o->getType());
	}
	return m_type->equalOrSubtypeOf(other);
}

bool FunctionType::equalOrSubtypeOf(const std::shared_ptr<Type>& other)
{
	if (other->t() == Types::FunctionType)
	{
		staticSharedPointerCast(other, o, FunctionType);
		return m_type->equalOrSubtypeOf(o->getType());
	}
	return m_type->equalOrSubtypeOf(other);
}

bool FunctionType::operator ==(const Type& rhs) const
		{
	return t() == rhs.t() && m_type == static_cast<const FunctionType*>(&rhs)->getType();
}

size_t FunctionType::hash() const
{
	if (m_hash == 0)
	{
		std::size_t h = 0;
		hash_combine(h, t(), m_name, m_type->hash());
		m_hash = h;
	}
	return m_hash;
}

std::string FunctionType::str() const
{
	return "(function of " + m_type->str() + ")";
}

std::shared_ptr<Type> FunctionType::copy()
{
	return std::shared_ptr<Type>(new FunctionType(m_type->copy()));
}

ProxyType::ProxyType(const std::shared_ptr<Parameter> param) :
				m_parameter(param)
{
	assert(param->getType()->t() == Types::FunctionType);
	m_name = "typeof(" + param->getName() + ")";
}

ProxyType::~ProxyType()
{
}

std::shared_ptr<Type> ProxyType::effectiveType() const
{
//	if (m_parameter->isInstantiated()) {
//		m_parameter->getInstance()->
//	}
	throw NotImplementedException(FILE_AND_LINE);
//	if (m_parameter->isInstantiated())
//		return m_parameter->getInstance().
}

bool ProxyType::isSupertypeOf(const std::shared_ptr<Type>& other)
{
	return effectiveType()->isSupertypeOf(other);
}

bool ProxyType::equalOrSupertypeOf(const std::shared_ptr<Type>& other)
{
	return effectiveType()->equalOrSupertypeOf(other);
}

bool ProxyType::isSubtypeOf(const std::shared_ptr<Type>& other)
{
	return effectiveType()->isSubtypeOf(other);
}

bool ProxyType::equalOrSubtypeOf(const std::shared_ptr<Type>& other)
{
	return effectiveType()->equalOrSubtypeOf(other);
}

const std::shared_ptr<Parameter>& ProxyType::getParameter() const
{
	return m_parameter;
}

bool ProxyType::operator ==(const Type& rhs) const
		{
	return t() == rhs.t() && m_parameter == static_cast<const ProxyType*>(&rhs)->getParameter();
}

size_t ProxyType::hash() const
{
	if (m_hash == 0)
	{
		std::size_t h = 0;
		hash_combine(h, t(), m_parameter->hash());
		m_hash = h;
	}

	return m_hash;
}

std::string ProxyType::str() const
{
	return "(type of " + m_parameter->getName() + ")";
}

std::shared_ptr<Type> ProxyType::copy()
{
	return std::shared_ptr<Type>(new ProxyType(m_parameter->getCopy<Parameter>()));
}

AnyType::AnyType(const std::string& name)
{
	m_name = name;
}

AnyType::~AnyType()
{
}

bool AnyType::isSupertypeOf(const std::shared_ptr<Type>& other)
{
	return true;
}

bool AnyType::equalOrSupertypeOf(const std::shared_ptr<Type>& other)
{
	return true;
}

bool AnyType::isSubtypeOf(const std::shared_ptr<Type>& other)
{
	return true;
}

bool AnyType::equalOrSubtypeOf(const std::shared_ptr<Type>& other)
{
	return true;
}

std::string AnyType::str() const
{
	return "(any)";
}

std::shared_ptr<Type> AnyType::copy()
{
	//we don't need to copy this
	return shared_from_this();
}

TypedObject::TypedObject(const std::string& name,
		const std::shared_ptr<Type>& type) :
				m_name(name),
				m_type(type),
				m_hash(0)
{
}

TypedObject::TypedObject(const std::shared_ptr<Type>& type) :
				m_type(type),
				m_hash(0)
{
}

TypedObject::TypedObject(const TypedObject& other) :
				m_name(other.m_name),
				m_type(other.m_type),
				m_hash(0)
{
}

TypedObject::~TypedObject()
{
}

const std::shared_ptr<Type>& TypedObject::getType() const
{
	return m_type;
}

//void TypedObject::setType(const std::shared_ptr<Type>& type)
//{
//	m_type = type;
//	m_hash = 0;
//}

const std::string& TypedObject::getName() const
{
	return m_name;
}

//void TypedObject::setName(const std::string& name)
//{
//	m_name = name;
//	m_hash = 0;
//}

bool TypedObject::isInstanceOf(const std::shared_ptr<Type>& type)
{
	return m_type->equalOrSubtypeOf(type);
}

bool TypedObject::operator ==(const TypedObject& rhs) const
		{
	return compare(rhs);
//	return hash() == rhs.hash();
}

bool TypedObject::compare(const TypedObject& rhs) const
		{
	return hash() == rhs.hash(); //t() == rhs.t() && m_name == rhs.m_name && *m_type == *rhs.m_type;
}

bool TypedObject::operator !=(const TypedObject& rhs) const
		{
	return !(*this == rhs);
}

size_t TypedObject::hash(bool hashAll) const
		{
	if (m_hash == 0)
	{
		std::size_t h = 0;
		hash_combine(h, t(), m_name, m_type->hash());
		m_hash = h;
	}
	return m_hash;
}

std::string TypedObject::str() const
{
	return m_name + " (" + m_type->str() + ")";
}

void TypedObject::instantiate(const std::shared_ptr<TypedObject> value)
{
	throw Exception("Don't call this method directly!");
}

std::shared_ptr<TypedObject> TypedObject::copy()
{
	return shared_from_this(); //we don't need to copy, because object will not change
	//return std::shared_ptr<TypedObject>(new TypedObject(m_name, m_type));
}

TypedObjectReturnValue::TypedObjectReturnValue(bool value) :
				TypedObject("return_value", DefaultTypes::objectType()),
				m_value(value)
{
}

bool TypedObjectReturnValue::isValue() const
{
	return m_value;
}

void TypedObjectReturnValue::setValue(bool value)
{
	m_value = value;
	m_hash = 0;
}

size_t TypedObjectReturnValue::hash(bool hashAll) const
		{
	if (m_hash == 0)
	{
		std::size_t h = 0;
		hash_combine(h, t(), m_name, m_value);
		m_hash = h;
	}

	return m_hash;
}

std::string TypedObjectReturnValue::str() const
{
	return m_value ? "True" : "False";
}

bool TypedObjectReturnValue::compare(const TypedObject& rhs) const
		{
	const TypedObjectReturnValue* p = dynamic_cast<const TypedObjectReturnValue*>(&rhs);
	if (!p)
		return false;
	else
		return hash() == rhs.hash(); //m_value == p->m_value;
}

std::shared_ptr<TypedObject> TypedObjectReturnValue::copy()
{
	return std::shared_ptr<TypedObject>(new TypedObjectReturnValue(m_value));
}

TypedNumber::TypedNumber(double number) :
				TypedObject(DefaultTypes::numberType()),
				m_number(number)
{
}

TypedNumber::TypedNumber(const TypedNumber& other) :
				TypedObject(other),
				m_number(other.m_number)
{
}

TypedNumber::~TypedNumber()
{
}

std::string TypedNumber::str() const
{
	return std::to_string(m_number) + " (type: " + m_type->str() + ")";
}

size_t TypedNumber::hash(bool hashAll) const
		{
	if (m_hash == 0)
	{
		std::size_t h = 0;
		hash_combine(h, t(), m_name, m_type->hash(), m_number);
		m_hash = h;
	}
	return m_hash;
}

bool TypedNumber::compare(const TypedObject& rhs) const
		{
	const TypedNumber* n = dynamic_cast<const TypedNumber*>(&rhs);
	if (!n)
	{
		return false;
	}
	else
	{
		return hash() == rhs.hash(); //m_name == rhs.getName() && *m_type == *rhs.getType() && m_number == n->m_number;
	}
}

double TypedNumber::getNumber() const
{
	return m_number;
}

std::shared_ptr<TypedObject> TypedNumber::copy()
{
	return std::shared_ptr<TypedObject>(new TypedNumber(m_number));
}

Parameter::Parameter(const std::string& name,
		const std::shared_ptr<Type>& type) :
				TypedObject(name, type),
				m_hashAll(0)
{
}

Parameter::Parameter(const Parameter& other) :
				TypedObject(other),
				m_instance(other.m_instance),
				m_hashAll(0)
{
}

Parameter::~Parameter()
{
}

void Parameter::instantiate(const std::shared_ptr<TypedObject> value)
{
	m_hashAll = 0;
	m_instance = value;
}

bool Parameter::isInstantiated()
{
	return m_instance.get() != NULL;
}

const std::shared_ptr<TypedObject> Parameter::getInstance()
{
	return m_instance;
}

std::string Parameter::str() const
{
	return m_name + " - " + m_type->str() + " (inst: " + (m_instance ? m_instance->str() : "no") + ")";
}

size_t Parameter::hash(bool hashAll) const
		{
	if (hashAll)
	{
		if (m_hashAll == 0)
		{
			std::size_t h = 0;
			hash_combine(h, t(), m_name, m_type->hash(), m_instance ? m_instance->hash() : 0);
			m_hashAll = h;
		}
		return m_hashAll;
	}
	else
	{
		return TypedObject::hash(hashAll);
	}
}

bool Parameter::compare(const TypedObject& rhs) const
		{
	const Parameter* p = dynamic_cast<const Parameter*>(&rhs);
	if (!p)
	{
		return false;
	}
	else
	{
//		LOG_INFO("compare: " << str() << " -> " << rhs.str() << " = " << (hash() == rhs.hash()));
		return hash() == rhs.hash(); //m_name == rhs.getName() && *m_type == *rhs.getType();// && m_instantiated == p->m_instantiated;
	}
}

std::shared_ptr<TypedObject> Parameter::copy()
{
	std::shared_ptr<TypedObject> param(new Parameter(m_name, m_type));
	if (m_instance)
	{
		param->instantiate(m_instance->copy());
	}
	return param;
}

std::shared_ptr<Type> DefaultTypes::objectType()
{
	static std::shared_ptr<Type> type(new Type("object"));
	return type;
}

std::shared_ptr<Type> DefaultTypes::numberType()
{
	static std::shared_ptr<Type> type(new Type("number"));
	return type;
}

std::shared_ptr<Type> DefaultTypes::booleanType()
{
	static std::shared_ptr<Type> type(new Type("boolean"));
	return type;
}

std::shared_ptr<AnyType> DefaultTypes::anyType()
{
	static std::shared_ptr<AnyType> type(new AnyType());
	return type;
}

std::shared_ptr<TypedObject> DefaultTypedObjects::trueObject()
{
	static std::shared_ptr<TypedObject> typedObject(new TypedObject("true", DefaultTypes::booleanType()));
	return typedObject;
}

std::shared_ptr<TypedObject> DefaultTypedObjects::falseObject()
{
	static std::shared_ptr<TypedObject> typedObject(new TypedObject("false", DefaultTypes::booleanType()));
	return typedObject;
}

std::shared_ptr<TypedObject> DefaultTypedObjects::__returnObjectTrue()
{
	static std::shared_ptr<TypedObject> typedObject(new TypedObjectReturnValue(true));
	return typedObject;
}

std::shared_ptr<TypedObject> DefaultTypedObjects::__returnObjectFalse()
{
	static std::shared_ptr<TypedObject> typedObject(new TypedObjectReturnValue(false));
	return typedObject;
}

std::shared_ptr<TypedObject> DefaultTypedObjects::unknownObject()
{
	static std::shared_ptr<TypedObject> typedObject(new TypedObject("unknown", DefaultTypes::anyType()));
	return typedObject;
}

std::shared_ptr<TypedObject> DefaultTypedObjects::undefinedObject()
{
	static std::shared_ptr<TypedObject> typedObject(new TypedObject("undefined", DefaultTypes::anyType()));
	return typedObject;
}

const std::vector<std::shared_ptr<Type>>& CompositeType::getTypes() const
{
	return m_types;
}

const std::shared_ptr<Type>& FunctionType::getType() const
{
	return m_type;
}

} /* namespace pddl */

