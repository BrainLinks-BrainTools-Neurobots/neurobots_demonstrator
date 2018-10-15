/*
 * utils.h
 *
 *  Created on: Apr 29, 2010
 *      Author: arh
 */

#ifndef __TICSYNC_UTILS_H_
#define __TICSYNC_UTILS_H_

#include "TICSync/Core.h"
#include <algorithm>
#include <functional>

namespace TICSync
{

/**
 * A quick and dirty debugging class.
 */
template<class T>
struct printer : public std::unary_function<T, void>
{
  printer(std::ostream& out) : os(out)
  {
  }

  void operator()(T x)
  {
    os.setf(std::ios::fixed);
    os.precision(9);
    os << x << "\n";
  }

  std::ostream& os;
};

}



#endif /* __TICSYNC_UTILS_H_ */
