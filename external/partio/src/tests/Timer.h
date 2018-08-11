/*
PARTIO SOFTWARE
Copyright 2010 Disney Enterprises, Inc. All rights reserved

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in
the documentation and/or other materials provided with the
distribution.

* The names "Disney", "Walt Disney Pictures", "Walt Disney Animation
Studios" or the names of its contributors may NOT be used to
endorse or promote products derived from this software without
specific prior written permission from Walt Disney Pictures.

Disclaimer: THIS SOFTWARE IS PROVIDED BY WALT DISNEY PICTURES AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE, NONINFRINGEMENT AND TITLE ARE DISCLAIMED.
IN NO EVENT SHALL WALT DISNEY PICTURES, THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND BASED ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
*/

#ifndef _timer_h_
#define _timer_h_

#ifndef PARTIO_WIN32

#include <sys/time.h>
#include <iostream>

struct Timer
{
    const char* name;
    struct timeval start;
    bool running;
    
    Timer(const char* name)
        :name(name)
    {
        gettimeofday(&start,0);
        running=true;
    }
    
    double Stop_Time()
    {
        struct timeval end;
        gettimeofday(&end,0);
        double seconds=(double)1e-6*(double)((end.tv_sec * 1000000 + end.tv_usec)
            - (start.tv_sec * 1000000 + start.tv_usec));
        std::cerr<<"Time for '"<<name<<"' was "<<seconds<<" s"<<std::endl;
        running=false;
        return seconds;
    }
    
    ~Timer()
    {
        if(running) Stop_Time();
    }
    
};

#else

#include <windows.h>
#include <string>

struct Timer
{
    const char* name;
    bool running;

    __int64 start;
    __int64 counts_per_sec;
    
    Timer(const char* name)
        :name(name)
    {
        QueryPerformanceFrequency((LARGE_INTEGER*)&counts_per_sec);
        QueryPerformanceCounter((LARGE_INTEGER*)&start);
        running=true;
    }
    
    double Stop_Time()
    {
        __int64 end;
        QueryPerformanceCounter((LARGE_INTEGER*)&end);
        double seconds=float(end-start)/float(counts_per_sec);
        std::cerr<<"Time for '"<<name<<"' was "<<seconds<<" s"<<std::endl;
        running=false;
        return seconds;
    }
    
    ~Timer()
    {
        if(running) Stop_Time();
    }
    
};


#endif

#endif
