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
#include <PartioIterator.h>
#include <iostream>

int main(int argc,char *argv[])
{

    Partio::ParticlesDataMutable* particles=Partio::createInterleave();

	particles->addParticles(10);

    Partio::ParticleAttribute position=particles->addAttribute("position",Partio::VECTOR,3);
    Partio::ParticleAttribute id=particles->addAttribute("id",Partio::INT,1);

    
    
    Partio::ParticlesDataMutable::iterator it=particles->begin();
    Partio::ParticleAccessor positionAccess(position);
    it.addAccessor(positionAccess);
    Partio::ParticleAccessor idAccess(id);
    it.addAccessor(idAccess);

    float x=0;
    int idCounter=0;
    for(;it!=particles->end();++it){
        Partio::Data<float,3>& P=positionAccess.data<Partio::Data<float,3> >(it);
        Partio::Data<int,1>& id=idAccess.data<Partio::Data<int,1> >(it);
        P[0]=x;P[1]=-x;P[2]=0;
        id[0]=idCounter;
        x+=1.;
        idCounter++;
    }
    
    Partio::write("test.bgeo",*particles);
    Partio::write("test.geo",*particles);
    

}
