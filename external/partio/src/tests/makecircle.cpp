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

#include <Partio.h>
#include <iostream>
#ifdef PARTIO_WIN32
#define M_PI (3.14159265359893238)
#endif

#include <cmath>

using namespace Partio;

int main(int argc,char *argv[])
{
    ParticlesDataMutable* p=create();
    ParticleAttribute positionAttr=p->addAttribute("position",VECTOR,3);
    ParticleAttribute normalAttr=p->addAttribute("normal",VECTOR,3);
    int n=30;
    for(int i=0;i<n;i++){
        int particle=p->addParticle();
        float* pos=p->dataWrite<float>(positionAttr,particle);
        float* norm=p->dataWrite<float>(normalAttr,particle);
        float theta=i*2*M_PI/(float)n;
        pos[2]=cos(theta);
        pos[0]=sin(theta);
        pos[1]=0;
        norm[0]=cos(theta);
        norm[2]=-sin(theta);
        norm[1]=0;
        
    }
    write("circle.00001.bgeo",*p);
    write("circle.00001.geo",*p);
    write("circle.00001.bin",*p);
    write("circle.00001.pdc",*p);
    write("circle.00001.pdb",*p);
    write("circle.00001.pda",*p);
    write("circle.00001.ptc",*p);
    write("circle.00001.rib",*p);
    write("circle.00001.mc",*p);

   
    p->release();
    return 0;
}
