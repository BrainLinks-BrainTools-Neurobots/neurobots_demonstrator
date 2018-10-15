/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 27, 2017
 *      Author: kuhnerd
 * 	  Filename: types.h
 */

#ifndef H7E516D54_07F7_4C32_A461_81AEBBD02DD8
#define H7E516D54_07F7_4C32_A461_81AEBBD02DD8
#include <ostream>
#include <string>
#include <vector>
#include <typeinfo>

#include <goal_planner_gui/pddl/exception.h>
#include <goal_planner_gui/pddl/utils.h>
#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace pddl
{

namespace Types
{
enum T
{
	Type,
	FunctionType,
	CompositeType,
	ProxyType,
	AnyType
};
}

namespace TypedObjectTypes
{
enum T
{
	TypedObject,
	TypedObjectReturnValue,
	TypedNumber,
	Parameter
};
}

class Element;
class Scope;
class Token;
class TypeHasher;

class Type: public std::enable_shared_from_this<Type>
{
public:
	Type(const std::string& type);
	Type(const std::string& type,
			const std::vector<std::shared_ptr<Type>>& supertypes);
	Type();
	virtual ~Type();

	void getSupertypes(std::unordered_set<std::shared_ptr<Type>, TypeHasher>& supertypes);
	bool isDirectSupertype(const std::shared_ptr<Type>& t);

	virtual bool isSupertypeOf(const std::shared_ptr<Type>& other);
	virtual bool equalOrSupertypeOf(const std::shared_ptr<Type>& other);

	virtual bool isSubtypeOf(const std::shared_ptr<Type>& other);
	virtual bool equalOrSubtypeOf(const std::shared_ptr<Type>& other);

	virtual std::shared_ptr<Type> copy();

	bool isCompatible(const std::shared_ptr<Type>& other);

	virtual bool operator==(const Type& rhs) const;
	virtual bool operator<(const Type& rhs) const;
	virtual bool operator!=(const Type& rhs) const;
	const std::string& getName() const;

	virtual size_t hash() const;
	virtual std::string str() const;
	virtual inline Types::T t() const
	{
		return Types::Type;
	}

	static std::shared_ptr<Type> parse(std::shared_ptr<Element> it,
			std::unordered_map<std::string, std::shared_ptr<Type>> types,
			std::shared_ptr<Scope> scope = std::shared_ptr<Scope>());

	static void parseTypeList(std::vector<std::shared_ptr<Element> >::iterator itCurrent,
			std::vector<std::shared_ptr<Element> >::iterator itEnd,
			std::unordered_map<Token, Token>& types);

	std::vector<std::shared_ptr<Type>> m_supertypes;

protected:
	std::string m_name;
	mutable size_t m_hash;
};

/**
 * This class represents a PDDL type that is composed of other
 * types using the "(either a b c)" syntax.
 */
class CompositeType: public Type
{
public:
	CompositeType(const std::vector<std::shared_ptr<Type>>& types);
	virtual ~CompositeType();

	virtual bool isSupertypeOf(const std::shared_ptr<Type>& other);
	virtual bool equalOrSupertypeOf(const std::shared_ptr<Type>& other);

	virtual bool isSubtypeOf(const std::shared_ptr<Type>& other);
	virtual bool equalOrSubtypeOf(const std::shared_ptr<Type>& other);

	virtual std::shared_ptr<Type> copy();

	virtual bool operator==(const Type& rhs) const;
	const std::vector<std::shared_ptr<Type>>& getTypes() const;

	virtual size_t hash() const;
	virtual std::string str() const;
	virtual inline Types::T t() const
	{
		return Types::CompositeType;
	}

protected:
	std::vector<std::shared_ptr<Type>> m_types;
};

/**
 * This class represents a function type as specified by the
 * "(function x)" syntax. Function types are used to model modal
 * predicates/actions/axioms in MAPL.
 */
class FunctionType: public Type
{
public:
	FunctionType(const std::shared_ptr<Type>& type);
	virtual ~FunctionType();

	virtual bool isSupertypeOf(const std::shared_ptr<Type>& other);
	virtual bool equalOrSupertypeOf(const std::shared_ptr<Type>& other);

	virtual bool isSubtypeOf(const std::shared_ptr<Type>& other);
	virtual bool equalOrSubtypeOf(const std::shared_ptr<Type>& other);

	virtual std::shared_ptr<Type> copy();

	virtual bool operator==(const Type& rhs) const;
	const std::shared_ptr<Type>& getType() const;

	virtual size_t hash() const;
	virtual std::string str() const;
	virtual inline Types::T t() const
	{
		return Types::FunctionType;
	}

protected:
	std::shared_ptr<Type> m_type;
};

/**
 * This class represents a proxy type as specified by the "(typeof
 * ?x)" syntax. It is used to refer to types of function parameters
 *  which might not be known at the time of parsing.
 *
 *  A ProxyType created with ProxyType(param) (with param.type being a
 *  FunctionType) will behave like the base type of param when param is not
 *  instantiated. When param is instantiated with a FunctionTerm, it
 *  will behave like the type of this FunctionTerm's function.
 */
class Parameter;

class ProxyType: public Type
{
public:
	ProxyType(const std::shared_ptr<Parameter> param);
	virtual ~ProxyType();

	std::shared_ptr<Type> effectiveType() const;

	virtual bool isSupertypeOf(const std::shared_ptr<Type>& other);
	virtual bool equalOrSupertypeOf(const std::shared_ptr<Type>& other);

	virtual bool isSubtypeOf(const std::shared_ptr<Type>& other);
	virtual bool equalOrSubtypeOf(const std::shared_ptr<Type>& other);

	virtual std::shared_ptr<Type> copy();

	virtual bool operator==(const Type& rhs) const;

	virtual size_t hash() const;
	virtual std::string str() const;
	virtual inline Types::T t() const
	{
		return Types::ProxyType;
	}

	const std::shared_ptr<Parameter>& getParameter() const;

protected:
	std::shared_ptr<Parameter> m_parameter;
};

/**
 * This class is used to represent types that match all other
 * types. Used to model the "unknown" constant.
 */
class AnyType: public Type
{
public:
	AnyType(const std::string& name = "any");
	virtual ~AnyType();

	virtual bool isSupertypeOf(const std::shared_ptr<Type>& other);
	virtual bool equalOrSupertypeOf(const std::shared_ptr<Type>& other);

	virtual bool isSubtypeOf(const std::shared_ptr<Type>& other);
	virtual bool equalOrSubtypeOf(const std::shared_ptr<Type>& other);

	virtual std::shared_ptr<Type> copy();

	virtual std::string str() const;
	virtual inline Types::T t() const
	{
		return Types::AnyType;
	}

};

class DefaultTypes
{
public:
	static std::shared_ptr<Type> objectType();
	static std::shared_ptr<Type> numberType();
	static std::shared_ptr<Type> booleanType();
	static std::shared_ptr<AnyType> anyType();
};

/**
 * This class represents all constants or objects in a PDDL
 * domain.
 */
class TypedObject: public std::enable_shared_from_this<TypedObject>
{
public:
	TypedObject(const std::string& name,
			const std::shared_ptr<Type>& type);
	TypedObject(const TypedObject& other);
	TypedObject(const std::shared_ptr<Type>& type);
	virtual ~TypedObject();
	const std::string& getName() const;
	const std::shared_ptr<Type>& getType() const;
	//	void setName(const std::string& name);
//	void setType(const std::shared_ptr<Type>& type);
	bool isInstanceOf(const std::shared_ptr<Type>& type);
	bool operator==(const TypedObject& rhs) const;
	bool operator!=(const TypedObject& rhs) const;
	virtual void instantiate(const std::shared_ptr<TypedObject> value);

	virtual std::shared_ptr<TypedObject> copy();
	template<class T>
	std::shared_ptr<T> getCopy()
	{
		return std::static_pointer_cast<T>(copy());
	}

	//hashAll required to toggle instance hashing of parameter
	virtual size_t hash(bool hashAll = false) const;
	virtual std::string str() const;
	virtual inline TypedObjectTypes::T t() const
	{
		return TypedObjectTypes::TypedObject;
	}

	virtual bool compare(const TypedObject& rhs) const;

protected:
	std::string m_name;
	std::shared_ptr<Type> m_type;
	mutable size_t m_hash;
};

class TypedObjectReturnValue: public TypedObject
{
public:
	TypedObjectReturnValue(bool value);

	virtual inline TypedObjectTypes::T t() const
	{
		return TypedObjectTypes::TypedObjectReturnValue;
	}

	virtual size_t hash(bool hashAll = false) const;
	virtual std::shared_ptr<TypedObject> copy();

	virtual std::string str() const;

	virtual bool compare(const TypedObject& rhs) const;
	bool isValue() const;
	void setValue(bool value);

private:
	bool m_value;
};

/**
 * Specialized subclass of TypedObject to represent numeric
 * values. It will automatically have a type of t_number.
 */
class TypedNumber: public TypedObject
{
public:
	TypedNumber(double number);
	TypedNumber(const TypedNumber& other);
	virtual ~TypedNumber();

	virtual std::string str() const;
	virtual size_t hash(bool hashAll = false) const;
	virtual inline TypedObjectTypes::T t() const
	{
		return TypedObjectTypes::TypedNumber;
	}

	virtual bool compare(const TypedObject& rhs) const;
	virtual std::shared_ptr<TypedObject> copy();

	double getNumber() const;

private:
	double m_number;
};

class Parameter: public TypedObject
{
public:
	Parameter(const std::string& name,
			const std::shared_ptr<Type>& type);
	Parameter(const Parameter& other);
	virtual ~Parameter();
	virtual void instantiate(const std::shared_ptr<TypedObject> value);
	bool isInstantiated();
	const std::shared_ptr<TypedObject> getInstance();
	virtual std::string str() const;
	virtual size_t hash(bool hashAll = false) const;
	virtual std::shared_ptr<TypedObject> copy();

	virtual bool compare(const TypedObject& rhs) const;

	virtual inline TypedObjectTypes::T t() const
	{
		return TypedObjectTypes::Parameter;
	}

private:
	std::shared_ptr<TypedObject> m_instance;
	mutable size_t m_hashAll;
};

POINTER_DEF(Parameter);

class DefaultTypedObjects
{
public:
	static std::shared_ptr<TypedObject> __returnObjectTrue();
	static std::shared_ptr<TypedObject> __returnObjectFalse();
	static std::shared_ptr<TypedObject> trueObject();
	static std::shared_ptr<TypedObject> falseObject();
	static std::shared_ptr<TypedObject> unknownObject();
	static std::shared_ptr<TypedObject> undefinedObject();
};

inline Type mostSpecificType(const std::vector<Type>& types)
{
	throw NotImplementedException(FILE_AND_LINE);
}

STREAM_OPERATOR(Type);
STREAM_OPERATOR(TypedObject);

HASH_AND_COMPARISON_OPERATOR(Type);
HASH_AND_COMPARISON_OPERATOR(TypedObject);

typedef std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher> TypedObjectUnorderedSet;
typedef std::vector<std::shared_ptr<TypedObject>> TypedObjectVector;
typedef std::shared_ptr<TypedObject> TypedObjectPtr;

struct TypedObjectEqual
{
	bool operator()(std::shared_ptr<TypedObject> const &lhs,
			std::shared_ptr<TypedObject> const &rhs) const
			{
		LOG_INFO("called: " << lhs << " " << rhs << " " << (lhs == rhs));
		return lhs == rhs;
	}
};

//pair
class TypedObjectPairHasher
{
public:

	std::size_t operator()(const std::pair<pddl::TypedObjectPtr, pddl::TypedObjectPtr>& obj) const noexcept
	{
		size_t hash = 0;
		pddl::hash_combine(hash, obj.first->hash(), obj.second->hash());
		return hash;
	}
};
inline bool operator ==(std::pair<pddl::TypedObjectPtr, pddl::TypedObjectPtr> const& lhs,
		std::pair<pddl::TypedObjectPtr, pddl::TypedObjectPtr> const& rhs)
{
	return lhs.first == rhs.first && lhs.second == rhs.second;
}

//vector
class TypedObjectVectorHasher
{
public:

	std::size_t operator()(const std::vector<pddl::TypedObjectPtr>& obj) const noexcept
	{
		size_t hash = 0;
		for (auto& it : obj)
		{
			pddl::hash_combine(hash, it->hash());
		}
		return hash;
	}
};

class TypedObjectVectorWithInstanceHasher
{
public:

	std::size_t operator()(const std::vector<pddl::TypedObjectPtr>& obj) const noexcept
	{
		size_t hash = 0;
		for (auto& it : obj)
		{
			pddl::hash_combine(hash, it->hash(true));
		}
		return hash;
	}
};

inline bool operator ==(std::vector<pddl::TypedObjectPtr> const& lhs,
		std::vector<pddl::TypedObjectPtr> const& rhs)
{
	if (lhs.size() != rhs.size())
		return false;

	for (size_t i = 0; i < lhs.size(); ++i)
	{
		if (lhs[i] != rhs[i])
			return false;
	}

	return true;
}

//typedef std::unordered_set<std::shared_ptr<TypedObject>, TypedObjectHasher> TypedObjectUnorderedSet;
//typedef std::unordered_set<std::shared_ptr<pddl::Type>, TypeHasher> TypeUnorderedSet;

} /* namespace pddl */

namespace std
{

template<>
class hash<pddl::Types::T>
{
public:
	size_t operator()(const pddl::Types::T& t) const
			{
		return std::hash<int>()((int) t);
	}
};

template<>
class hash<pddl::TypedObjectTypes::T>
{
public:
	size_t operator()(const pddl::TypedObjectTypes::T& t) const
			{
		return std::hash<int>()((int) t);
	}
};

}

STD_SHARED_PTR_HASH(pddl::TypedObject);
STD_SHARED_PTR_HASH(pddl::Type);

#endif /* H7E516D54_07F7_4C32_A461_81AEBBD02DD8 */
