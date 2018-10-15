/*
 * TimestampFilter.h
 *
 *  Created on: Aug 30, 2010
 *      Author: arh
 */

#ifndef __TimestampFilter_h
#define __TimestampFilter_h

#include "TICSync/LinearOneWayClockSync.h"
#include "TICSync/Core.h"
#include "TICSync/TimingEnvelope.h"
#include "TICSync/TimestampMapper.h"
#include "TICSync/dataTypes.h"

namespace TICSync
{

/**
 * Deprecated class.  Simply hands everything over to the
 * LinearOneWayClockSync class
 */
template< class T >
class TimestampFilter : public LinearOneWayClockSync<T>
{
public:

  TimestampFilter()
  {
  }

  virtual ~TimestampFilter()
  {
  }


public:

  template< class TT >
  friend std::ostream
  & operator<<(std::ostream& os, const TimestampFilter< TT >& filt);
};

template<class T>
inline std::ostream& operator<<(std::ostream& os, const TimestampFilter<T>& filt)
{
  os << filt.m_env;
  return os;
}

} // TICSync


#endif /* __TimestampFilter_h */
