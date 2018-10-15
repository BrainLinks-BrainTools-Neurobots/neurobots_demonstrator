/*
 * Copyright (c) 2018 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Sep 19, 2018
 *      Author: kuhnerd
 * 	  Filename: std_boost.h
 */

#ifndef H1AFB93B7_53B7_4F39_8CD7_B65C2EF7BAAD
#define H1AFB93B7_53B7_4F39_8CD7_B65C2EF7BAAD

#include <boost/smart_ptr/shared_ptr.hpp>
#include <memory>

namespace {
    template<class SharedPointer> struct Holder {
        SharedPointer p;

        Holder(const SharedPointer &p) : p(p) {}
        Holder(const Holder &other) : p(other.p) {}
        Holder(Holder &&other) : p(std::move(other.p)) {}

        void operator () (...) { p.reset(); }
    };
}

template<class T> std::shared_ptr<T> to_std_ptr(const boost::shared_ptr<T> &p) {
    typedef Holder<std::shared_ptr<T>> H;
    if(H *h = boost::get_deleter<H, T>(p)) {
        return h->p;
    } else {
        return std::shared_ptr<T>(p.get(), Holder<boost::shared_ptr<T>>(p));
    }
}

template<class T> boost::shared_ptr<T> to_boost_ptr(const std::shared_ptr<T> &p){
    typedef Holder<boost::shared_ptr<T>> H;
    if(H * h = std::get_deleter<H, T>(p)) {
        return h->p;
    } else {
        return boost::shared_ptr<T>(p.get(), Holder<std::shared_ptr<T>>(p));
    }
}



#endif /* H1AFB93B7_53B7_4F39_8CD7_B65C2EF7BAAD */
