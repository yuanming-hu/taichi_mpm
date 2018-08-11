//#####################################################################
// Copyright (c) 2016, Mridul Aanjaneya
// Distributed under the FreeBSD license (see license.txt)
//#####################################################################
#ifndef __Poisson_Data__
#define __Poisson_Data__

namespace SPGrid{
template<class T,class T_FLAGS=uint32_t>
struct Poisson_Data
{
    typedef T Data_type;
    typedef T_FLAGS Flags_type;

    T_FLAGS flags;
    T ch0;
    T ch1;
    T ch2;
};
}
#endif
