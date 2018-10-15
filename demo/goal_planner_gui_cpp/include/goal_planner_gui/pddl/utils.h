/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 27, 2017
 *      Author: kuhnerd
 * 	  Filename: utils.h
 */

#ifndef HDC257E2C_0581_4946_960F_BE5F83654764
#define HDC257E2C_0581_4946_960F_BE5F83654764

#include <boost/variant/variant.hpp>
#include <iostream>
#include <string.h>
#include <functional>
#include <unordered_set>
#include <vector>
#include <memory>
#include <tuple>
#include <unordered_map>

//output
#define ANSI_RESET   			"\033[0m"

#define ANSI_BLACK   			"\033[30m"
#define ANSI_RED     			"\033[31m"
#define ANSI_GREEN   			"\033[32m"
#define ANSI_YELLOW  			"\033[33m"
#define ANSI_BLUE    			"\033[34m"
#define ANSI_MAGENTA 			"\033[35m"
#define ANSI_CYAN    			"\033[36m"
#define ANSI_WHITE   			"\033[37m"

#define ANSI_BACK_BLACK   		"\033[40m"
#define ANSI_BACK_RED     		"\033[41m"
#define ANSI_BACK_GREEN   		"\033[42m"
#define ANSI_BACK_YELLOW  		"\033[43m"
#define ANSI_BACK_BLUE    		"\033[44m"
#define ANSI_BACK_MAGENTA 		"\033[45m"
#define ANSI_BACK_CYAN    		"\033[46m"
#define ANSI_BACK_WHITE   		"\033[47m"
#define ANSI_BACK_LIGHT_BLUE   	"\033[104m"

#define ANSI_BOLD				"\033[1m"
#define ANSI_ITALIC   			"\033[3m"

#define __FILENAME_ONLY__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LOG_FATAL(TEXT) std::cout << std::boolalpha << ANSI_BOLD << ANSI_BACK_RED << "[" << ANSI_WHITE << "F" \
			<< ANSI_WHITE << ", " << __FILENAME_ONLY__ << ", " << __LINE__ << "]:" << ANSI_RESET << " " << TEXT << std::endl;
#define LOG_ERROR(TEXT) std::cout << std::boolalpha << ANSI_BOLD << ANSI_BACK_RED << "[" << ANSI_WHITE << "E" \
			<< ANSI_WHITE << ", " << __FILENAME_ONLY__ << ", " << __LINE__ << "]:" << ANSI_RESET << " " << TEXT << std::endl;
#define LOG_WARNING(TEXT) std::cout << std::boolalpha << ANSI_BOLD << ANSI_BACK_YELLOW << "[" << ANSI_WHITE << "W" \
			<< ANSI_WHITE << ", " << __FILENAME_ONLY__ << ", " << __LINE__ << "]:" << ANSI_RESET << " " << TEXT << std::endl;
#define LOG_INFO(TEXT) std::cout << std::boolalpha << ANSI_BOLD << ANSI_BACK_LIGHT_BLUE << "[" << ANSI_WHITE << "I" \
			<< ANSI_WHITE << ", " << __FILENAME_ONLY__ << ", " << __LINE__ << "]:" << ANSI_RESET << " " << TEXT << std::endl;
#define LOG_DEBUG(TEXT) std::cout << std::boolalpha << ANSI_BOLD << ANSI_BACK_LIGHT_BLUE << "[" << ANSI_WHITE << "D" \
			<< ANSI_WHITE << ", " << __FILENAME_ONLY__ << ", " << __LINE__ << "]:" << ANSI_RESET << " " << TEXT << std::endl;
#define LOG_INFO_CONTAINER(container) for (auto& it: container) LOG_INFO("\t - " << it);
#define LOG_INFO_CONTAINER_STR(container) for (auto& it: container) LOG_INFO("\t - " << it->str());
#define LOG_INFO_CONTAINER_TEXT_STR(text, container) LOG_INFO(text); for (auto& it: container) LOG_INFO("\t - " << it->str());
#define LOG_INFO_MAP(container) for (auto& it: container) LOG_INFO("\t - " << it.first << " -> " << it.second);

//template<class T, class CMP = std::less<T>, class ALLOC = std::allocator<T> >
//std::unordered_set<T, CMP, ALLOC>& operator *(
//		std::unordered_set<T, CMP, ALLOC> &s1,
//		const std::unordered_set<T, CMP, ALLOC> &s2)
//{
//	auto iter1 = s1.begin();
//	for (auto iter2 = s2.begin(); iter1 != s1.end() && iter2 != s2.end();)
//	{
//		if (*iter1 < *iter2)
//			iter1 = s1.erase(iter1);
//		else
//		{
//			if (!(*iter2 < *iter1))
//				++iter1;
//			++iter2;
//		}
//	}
//	while (iter1 != s1.end())
//		iter1 = s1.erase(iter1);
//	return s1;
//}

template<class T, class CMP = std::less<T>, class ALLOC = std::allocator<T> >
std::unordered_set<T, CMP, ALLOC>& operator +(
		std::unordered_set<T, CMP, ALLOC> &s1,
		const std::unordered_set<T, CMP, ALLOC> &s2)
{
	s1.insert(s2.begin(), s2.end());
	return s1;
}

namespace helpers
{
template<class T>
class ProductVector
{
	struct Iterator
	{
		typename std::vector<T>::const_iterator begin;
		typename std::vector<T>::const_iterator end;
		typename std::vector<T>::const_iterator me;
	};
	typedef std::vector<Iterator> Vd;
	typedef std::vector<T> Vt;
	typedef std::vector<Vt> VVt;

public:
	//https://stackoverflow.com/questions/5279051/how-can-i-create-cartesian-product-of-vector-of-vectors
	static void compute(
			const std::vector<std::vector<T>>& in,
			std::vector<std::vector<T>>& out)

	{
		Vd vd;

		// Start all of the iterators at the beginning.
		for (typename VVt::const_iterator it = in.begin(); it != in.end(); ++it)
		{
			Iterator d = { (*it).begin(), (*it).end(), (*it).begin() };
			vd.push_back(d);
		}

		while (true)
		{

			// Construct your first product vector by pulling
			// out the element of each vector via the iterator.
			Vt result;
			for (typename Vd::const_iterator it = vd.begin(); it != vd.end(); it++)
			{
				result.push_back(*(it->me));
			}
			out.push_back(result);

			// Increment the rightmost one, and repeat.

			// When you reach the end, reset that one to the beginning and
			// increment the next-to-last one. You can get the "next-to-last"
			// iterator by pulling it out of the neighboring element in your
			// vector of iterators.
			for (typename Vd::iterator it = vd.begin();;)
			{
				// okay, I started at the left instead. sue me
				++(it->me);
				if (it->me == it->end)
				{
					if (it + 1 == vd.end())
					{
						// I'm the last digit, and I'm about to roll
						return;
					}
					else
					{
						// cascade
						it->me = it->begin;
						++it;
					}
				}
				else
				{
					// normal
					break;
				}
			}
		}
	}
};

template<class T, class Hasher>
class ProductVectorOfSets
{
	struct Iterator
	{
		typename std::unordered_set<T, Hasher>::const_iterator begin;
		typename std::unordered_set<T, Hasher>::const_iterator end;
		typename std::unordered_set<T, Hasher>::const_iterator me;
	};
	typedef std::vector<Iterator> Vd;
	typedef std::unordered_set<T, Hasher> Vt;
	typedef std::vector<Vt> VVt;

public:
	//https://stackoverflow.com/questions/5279051/how-can-i-create-cartesian-product-of-vector-of-vectors
	static void compute(
			const std::vector<std::unordered_set<T, Hasher>>& in,
			std::vector<std::unordered_set<T, Hasher>>& out)

	{
		Vd vd;

		// Start all of the iterators at the beginning.
		for (typename VVt::const_iterator it = in.begin(); it != in.end(); ++it)
		{
			Iterator d = { (*it).begin(), (*it).end(), (*it).begin() };
			vd.push_back(d);
		}

		while (true)
		{

			// Construct your first product vector by pulling
			// out the element of each vector via the iterator.
			Vt result;
			for (typename Vd::const_iterator it = vd.begin(); it != vd.end(); it++)
			{
				result.insert(*(it->me));
			}
			out.push_back(result);

			// Increment the rightmost one, and repeat.

			// When you reach the end, reset that one to the beginning and
			// increment the next-to-last one. You can get the "next-to-last"
			// iterator by pulling it out of the neighboring element in your
			// vector of iterators.
			for (typename Vd::iterator it = vd.begin();;)
			{
				// okay, I started at the left instead. sue me
				++(it->me);
				if (it->me == it->end)
				{
					if (it + 1 == vd.end())
					{
						// I'm the last digit, and I'm about to roll
						return;
					}
					else
					{
						// cascade
						it->me = it->begin;
						++it;
					}
				}
				else
				{
					// normal
					break;
				}
			}
		}
	}
};

template<class T>
class ProductVectorOfVectors
{
	struct Iterator
	{
		typename std::vector<T>::const_iterator begin;
		typename std::vector<T>::const_iterator end;
		typename std::vector<T>::const_iterator me;
	};
	typedef std::vector<Iterator> Vd;
	typedef std::vector<T> Vt;
	typedef std::vector<Vt> VVt;

public:
	//https://stackoverflow.com/questions/5279051/how-can-i-create-cartesian-product-of-vector-of-vectors
	static void compute(
			const std::vector<std::vector<T>>& in,
			std::vector<std::vector<T>>& out)

	{
		Vd vd;

		// Start all of the iterators at the beginning.
		for (typename VVt::const_iterator it = in.begin(); it != in.end(); ++it)
		{
			Iterator d = { (*it).begin(), (*it).end(), (*it).begin() };
			vd.push_back(d);
		}

		while (true)
		{

			// Construct your first product vector by pulling
			// out the element of each vector via the iterator.
			Vt result;
			for (typename Vd::const_iterator it = vd.begin(); it != vd.end(); it++)
			{
				result.push_back(*(it->me));
			}
			out.push_back(result);

			// Increment the rightmost one, and repeat.

			// When you reach the end, reset that one to the beginning and
			// increment the next-to-last one. You can get the "next-to-last"
			// iterator by pulling it out of the neighboring element in your
			// vector of iterators.
			for (typename Vd::iterator it = vd.begin();;)
			{
				// okay, I started at the left instead. sue me
				++(it->me);
				if (it->me == it->end)
				{
					if (it + 1 == vd.end())
					{
						// I'm the last digit, and I'm about to roll
						return;
					}
					else
					{
						// cascade
						it->me = it->begin;
						++it;
					}
				}
				else
				{
					// normal
					break;
				}
			}
		}
	}
};

} //namespace helpers end

namespace pddl
{

inline std::string stringConcat(const std::vector<std::string>& parts,
		const std::string& concat)
{
	std::string res;
	for (auto& p : parts)
	{
		res += p + concat;
	}
	if (!parts.empty())
	{
		for (size_t i = 0; i < concat.size(); ++i)
		{
			res.pop_back();
		}
	}
	return res;
}

struct DoubleDefaultZero
{
	double val = 0;
};

struct IntDefaultZero
{
	int val = 0;
};

template<typename T, typename C>
inline bool any(C& list,
		std::function<bool(T)> condition)
{
	for (auto& t : list)
		if (condition(t))
			return true;
	return false;
}

template<typename T, typename C>
inline bool all(C& list,
		std::function<bool(const T&)> condition)
{
	for (auto& t : list)
		if (!condition(t))
			return false;
	return true;
}

template<class T, class T2>
int index(const std::vector<T>& container,
		const T2& val)
{
	int i = 0;
	for (auto& it : container)
	{
		if (it == val)
			return i;
		++i;
	}
	return -1;
}

inline void hash_combine(std::size_t& seed)
{
}

template<typename T, typename ... Rest>
inline void hash_combine(std::size_t& seed,
		const T& v,
		Rest ... rest)
{
	std::hash<T> hasher;
	seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
	hash_combine(seed, rest...);
}

/** Intersection and union function for unordered containers which support a fast lookup function find()
 *  Return values are moved by move-semantics, for c++11/c++14 this is efficient, otherwise it results in a copy
 */

namespace unordered_helpers
{

/**
 * Computes the intersection set of in1 and in2
 * python: | on sets
 */
template<typename UnorderedIn1, typename UnorderedIn2,
		typename UnorderedOut = UnorderedIn1>
UnorderedOut makeIntersection(const UnorderedIn1 &in1,
		const UnorderedIn2 &in2)
{
	if (in2.size() < in1.size())
	{
		return makeIntersection<UnorderedIn2, UnorderedIn1, UnorderedOut>(in2, in1);
	}

	UnorderedOut out;
	auto e = in2.end();
	for (auto & v : in1)
	{
		if (in2.find(v) != e)
		{
			out.insert(v);
		}
	}
	return out;
}

/**
 * Computes the union set of in1 and in2
 * python: & on sets
 */
template<typename UnorderedIn1, typename UnorderedIn2,
		typename UnorderedOut = UnorderedIn1>
UnorderedOut makeUnion(const UnorderedIn1 &in1,
		const UnorderedIn2 &in2)
{
	UnorderedOut out;
	out.insert(in1.begin(), in1.end());
	out.insert(in2.begin(), in2.end());
	return out;
}

template<typename UnorderedIn1, typename UnorderedIn2,
		typename UnorderedOut = UnorderedIn1>
void makeUnionInline(UnorderedIn1 &in1,
		const UnorderedIn2 &in2)
{
	in1.insert(in2.begin(), in2.end());
}

/**
 * Computes a set containing all elements from a, which
 * are not in b
 * python: - on sets
 */
template<typename UnorderedIn>
UnorderedIn minus(const UnorderedIn &a,
		const UnorderedIn &b)
{
	UnorderedIn out;
	for (auto& it : a)
		if (b.find(it) == b.end())
			out.insert(it);
	return out;
}

/**
 * Checks if in1 is a subset of in2
 * python: < on sets
 */
template<typename UnorderedIn>
bool isSubset(const UnorderedIn &in1,
		const UnorderedIn &in2)
{
	if (in1.size() > in2.size())
	{
		return false;
	}

	for (auto& it1 : in1)
		if (in2.find(it1) == in2.end())
			return false;
	return true;
}

template<typename UnorderedIn1, typename UnorderedIn2>
void differenceUpdate(UnorderedIn1& in,
		UnorderedIn2& other)
{
	for (auto it = other.begin(); it != other.end(); ++it)
	{
		in.erase(*it);
	}
}

template<typename Key, typename Value, typename Hash = std::hash<Key>>
Key getDefault(std::unordered_map<Key, Value, Hash>& map,
		const Key& key,
		const Value& defaultValue)
{
	typename std::unordered_map<Key, Value, Hash>::const_iterator v = map.find(key);
	if (v != map.end())
	{
		return v->second;
	}
	else
	{
		return defaultValue;
	}
}

}

} /* namespace pddl */

#define isInstance(var, classname) std::dynamic_pointer_cast<classname>(var)

#define isInstanceCast(var, varnew, classname) const classname* varnew = dynamic_cast<const classname*>(var)
#define isInstanceSharedCast(var, varnew, classname) std::shared_ptr<classname> varnew = std::dynamic_pointer_cast<classname>(var)

#define staticPointerCast(var, varnew, classname) const classname* varnew = static_cast<const classname*>(var)
#define staticSharedPointerCast(var, varnew, classname) std::shared_ptr<classname> varnew = std::static_pointer_cast<classname>(var)
#define dynamicSharedPointerCast(var, varnew, classname) std::shared_ptr<classname> varnew = std::dynamic_pointer_cast<classname>(var)
#define isInstanceStaticSharedCast(var, varnew, classname) staticPointerCast(var, varnew, classname)

#define isInstanceSharedConstCast(var, varnew, classname) std::shared_ptr<const classname> varnew = std::const_pointer_cast<const classname>(var)

#define VECTOR_DEF(classname) typedef std::vector<std::shared_ptr<classname>> classname##Vector;
#define POINTER_DEF(classname) typedef std::shared_ptr<classname> classname##Ptr;

#define HASH_AND_COMPARISON_OPERATOR(classname)\
		class classname##Hasher\
		{\
		public:\
			std::size_t operator()(const std::shared_ptr<classname>& obj) const noexcept\
					{\
				if (!obj)\
					return 0;\
				else \
					return obj->hash();\
			}\
		};\
		inline bool operator == (std::shared_ptr<classname> const& lhs, std::shared_ptr<classname> const& rhs)\
		{\
		    return (!lhs && !rhs) || (lhs && rhs && (*lhs) == (*rhs));\
		}\
		inline bool operator != (std::shared_ptr<classname> const& lhs, std::shared_ptr<classname> const& rhs)\
		{\
		    return !(lhs == rhs);\
		}\
		typedef std::unordered_set<std::shared_ptr<classname>, classname##Hasher> classname##UnorderedSet;\
		VECTOR_DEF(classname)\
		POINTER_DEF(classname)

#define HASH_AND_COMPARISON_STRUCT(classname, hashFct, compareFct, hasherPrefix)\
		class hasherPrefix##Hasher\
		{\
		public:\
			std::size_t operator()(const std::shared_ptr<classname>& obj) const noexcept\
					{\
				if (!obj)\
					return 0;\
				else \
					return obj->hashFct();\
			}\
		};\
		class hasherPrefix##EqualTo\
		{\
		public:\
			std::size_t operator()(const std::shared_ptr<classname>& obj1, const std::shared_ptr<classname>& obj2) const noexcept\
					{\
				return (!obj1 && !obj2) || (obj1 && obj2 && obj1->compareFct(*obj2));\
			}\
		};\
		typedef std::unordered_set<std::shared_ptr<classname>, hasherPrefix##Hasher, hasherPrefix##EqualTo> hasherPrefix##UnorderedSet;\

//		struct classname##Equal\
//		{\
//		    bool operator()(std::shared_ptr<classname> const &lhs, std::shared_ptr<classname> const &rhs) const\
//		    {\
//		        return lhs == rhs;\
//		    }\
//		};

#define STD_SHARED_PTR_HASH(classname)\
		namespace std\
		{\
		    template<> struct hash<std::shared_ptr<classname>>\
		    {\
			size_t operator()(const std::shared_ptr<classname>& obj) const\
			    {\
		    		if (!obj)\
						return 0;\
					else \
						return obj->hash();\
			    }\
		    };\
		}

#define STREAM_OPERATOR(classname)\
		inline std::ostream& operator<<(std::ostream& stream,\
				const std::shared_ptr<classname>& val){\
					if (val)\
					{\
						stream << val->str();\
					}\
					else \
					{\
						stream << "__NULL_POINTER__";\
					}\
					return stream;\
				}\
		inline std::ostream& operator<<(std::ostream& stream,\
				const classname& val){\
					stream << val.str();\
					return stream;\
				}

#define CONTAINS(value, map) map.find(value) != map.end()
#define CONTAINSV(value, vec) std::find(vec.begin(), vec.end(), value) != vec.end()
#define COPY_VECTOR(vector_old, vector_new) for (auto& it_##vector_old: vector_old) vector_new.push_back(it_##vector_old->copy())
#define COPY_VECTOR_CAST(vector_old, vector_new, type) for (auto& it_##vector_old: vector_old) vector_new.push_back(it_##vector_old->getCopy<type>());
#define CONTAINS_NOT(value, map) map.find(value) == map.end()
#define CONTAINSV_NOT(value, vec) std::find(vec.begin(), vec.end(), value) == vec.end()
#define ASSERT(cond) if(!(cond)) { LOG_ERROR("ASSERTION in file " << __FILENAME_ONLY__ << " (" << __LINE__ << "): " << #cond << std::flush); exit(5); }
#define REMOVE_LAST_COMMA(string) if (!string.empty()) \
{\
	string.pop_back();\
	string.pop_back();\
}\

namespace std
{
template<> struct hash<std::tuple<std::string, std::string, std::string>>
{
	size_t operator()(const std::tuple<std::string, std::string, std::string>& obj) const
			{
		size_t h = 0;
		pddl::hash_combine(h, std::get<0>(obj), std::get<1>(obj), std::get<2>(obj));
		return h;
	}
};

template<> struct hash<std::tuple<std::string, std::string, boost::variant<std::string, bool>>>
{
	size_t operator()(const std::tuple<std::string, std::string, boost::variant<std::string, bool>>& obj) const
			{
		size_t h = 0;
		pddl::hash_combine(h, std::get<0>(obj), std::get<1>(obj), std::get<2>(obj));
		return h;
	}
};
}

template<class T>
std::ostream& operator <<(std::ostream& os,
		const std::vector<T>& v)
{
	os << "[";
	for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii)
	{
		os << " " << *ii;
	}
	os << "]";
	return os;
}

#endif /* HDC257E2C_0581_4946_960F_BE5F83654764 */
