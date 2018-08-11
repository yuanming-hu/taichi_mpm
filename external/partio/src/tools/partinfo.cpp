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
#include <iomanip>
#include <stdlib.h>

int main(int argc,char *argv[])
{
    if(argc<2){
        std::cerr<<"Usage is: "<<argv[0]<<" <filename> { particle indices to print full info } "<<std::endl;
        return 1;
    }
    Partio::ParticlesDataMutable* p=Partio::read(argv[1]);
    if(p){
        std::cout<<std::setiosflags(std::ios::left)<<"Number of particles:  "<<p->numParticles()<<std::endl;
        int numAttr=p->numAttributes();
        std::cout<<std::setw(12)<<"Type"<<std::setw(10)<<"Count"<<std::setw(30)<<"Name"<<std::endl;
        std::cout<<std::setw(12)<<"----"<<std::setw(10)<<"-----"<<std::setw(30)<<"----"<<std::endl;
        for(int i=0;i<numAttr;i++){
            Partio::ParticleAttribute attr;
            p->attributeInfo(i,attr);
            std::cout<<std::setw(12)<<Partio::TypeName(attr.type)
                     <<std::setw(10)<<attr.count
                     <<std::setw(30)<<attr.name<<std::endl;;
        }

        Partio::ParticleAttribute positionhandle;
        p->attributeInfo("position",positionhandle);
        if(argc==2){
            for(int i=0;i<std::min(10,p->numParticles());i++){
                const float* data=p->data<float>(positionhandle,i);;
                std::cout<<"particle "<<i<<" data "<<data[0]<<" "<<data[1]<<" "<<data[2]<<std::endl;
            }
        }else{
            for(int j=2;j<argc;j++){
                int particleIndex=atoi(argv[j]);
                std::cout<<"---------------------------"<<std::endl;
                std::cout<<"Particle "<<particleIndex<<":"<<std::endl;
                if(particleIndex>p->numParticles() || particleIndex<0){
                    std::cout<<"OUT OF RANGE"<<std::endl;
                }else{
                    for(int i=0;i<numAttr;i++){
                        Partio::ParticleAttribute attr;
                        p->attributeInfo(i,attr);
                        std::cout<<std::setw(10)<<Partio::TypeName(attr.type)<<" "<<std::setw(10)<<attr.name;
                        for(int ii=0;ii<attr.count;ii++){
                            if(attr.type==Partio::INDEXEDSTR){
                                int val=p->data<int>(attr,particleIndex)[ii];
                                std::cout<<" "<<val<<" '"<<p->indexedStrs(attr)[val]<<"'";
                            }else if(attr.type==Partio::INT) std::cout<<" "<<p->data<int>(attr,particleIndex)[ii];
                            else std::cout<<" "<<p->data<float>(attr,particleIndex)[ii];
                        }
                        std::cout<<std::endl;
                    }
                }
            }
        }

        p->release();
    }
    
    return 0;
}
