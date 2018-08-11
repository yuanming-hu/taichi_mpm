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
#include <typeinfo>
#include <cstdlib>

#include "Timer.h"

int main(int argc,char *argv[])
{
    Timer* timer=new Timer("make array");
    
    Partio::ParticlesDataMutable& foo=*Partio::create();
    int nParticles=10000000;
    foo.addParticles(nParticles);
    Partio::ParticleAttribute position=foo.addAttribute("position",Partio::VECTOR,3);
    Partio::ParticleAttribute radius=foo.addAttribute("radius",Partio::FLOAT,1);
    Partio::ParticleAttribute life=foo.addAttribute("life",Partio::FLOAT,2);
    delete timer;

    {

        Timer timer("write data");
        for(int i=0;i<nParticles;i++){
            float* pos=foo.dataWrite<float>(position,i);
            float* rad=foo.dataWrite<float>(radius,i);
            float* lifeVal=foo.dataWrite<float>(life,i);
            pos[0]=i;pos[1]=2*i;pos[2]=3*i;
            rad[0]=.1*i;
            lifeVal[0]=.01*i;
            lifeVal[1]=1.;
            //std::cerr<<"writing data "<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
        }
    }

    int i=0;
    Partio::ParticlesDataMutable::iterator it=foo.begin();
    Partio::ParticleAccessor Xacc(position);
    it.addAccessor(Xacc);
    for(;it!=foo.end();++it){
        Partio::Data<float,3>& X=Xacc.data<Partio::Data<float,3> >(it);
//        Partio::Data<float,3>& X=it.data<Partio::Data<float,3> >(position);
        //const Partio::Data<float,1>& r=it.data<Partio::Data<float,1> >(radius);
        //const Partio::Data<float,2>& l=it.data<Partio::Data<float,2> >(life);
        
//        std::cerr<<"write guy X="<<X<<std::endl; // " life="<<l<<" radius="<<r<<std::endl;
        X[1]+=.1*i;// 100.;
        i++;
    }
    std::vector<double> iteratorTimes,handTimes,rawTimes;
    for(int i=0;i<10;i++){

        {
            Timer timer("Access and sum with iterator");
            
            Partio::ParticlesData& fooc=foo;
            float sum=0;
            Partio::ParticlesDataMutable::const_iterator it=fooc.begin();
            Partio::ParticleAccessor Xacc(position);
            Partio::ParticleAccessor racc(radius);
            Partio::ParticleAccessor lacc(life);
            it.addAccessor(Xacc);
            it.addAccessor(racc);
            it.addAccessor(lacc);
            
            for(;it!=fooc.end();++it){
                const Partio::Data<float,3>& X=Xacc.data<Partio::Data<float,3> >(it);
                const Partio::Data<float,1>& r=racc.data<Partio::Data<float,1> >(it);
                const Partio::Data<float,2>& l=lacc.data<Partio::Data<float,2> >(it);
                
//                std::cerr<<"guy X="<<X[0]<<" "<<X[1]<<" "<<X[2]<<" life="<<l<<" radius="<<r<<std::endl;
                sum+= X[0]+r[0]+l[0];
            }
            std::cerr<<"sum is "<<sum<<std::endl;
            iteratorTimes.push_back( timer.Stop_Time());
        }
    
        {
            Timer timer("Access and sum by hand");
            
            Partio::ParticlesData& fooc=foo;
            float sum=0;
            for(uint64_t i=0;i<(uint64_t)fooc.numParticles();i++){
                const float* X=fooc.data<float>(position,i);
                const float* r=fooc.data<float>(radius,i);
                const float* l=fooc.data<float>(life,i);
                
//                std::cerr<<"guy X="<<X[0]<<" "<<X[1]<<" "<<X[2]<<" life="<<l<<" radius="<<r<<std::endl;
                sum+= X[0]+r[0]+l[0];
            }
            std::cerr<<"sum is "<<sum<<std::endl;
           
            handTimes.push_back( timer.Stop_Time());

        }

        // NOTE: this is using direct access and assuming ParticlesSimple (non-interleaved data)
        // This is not what you should ever do, and is not guaranteed to work under any stretch of the imagination.
        // I am only doing it here to see what I'm losing in iteration.
        {
            Timer timer("Access and sum raw");
            std::string fooType=typeid(foo).name();
            if(fooType.find("ParticlesSimple")==std::string::npos){
                std::cerr<<"You are using the wrong particle type for this test it must be ParticlesSimple"<<std::endl;
                exit(1);
            }
            Partio::ParticlesData& fooc=foo;
            int nParts=fooc.numParticles();
            float sum=0;
            const float* X=fooc.data<float>(position,0);
            const float* r=fooc.data<float>(radius,0);
            const float* l=fooc.data<float>(life,0);
            for(int i=0;i<nParts;i++){
                //std::cout<<i<<std::endl;
                //sum+= X[0]+X[3]+X[4];
                sum+= X[0]+r[0]+l[0];
                X+=3;
                r++;
                l+=2;
            }
            std::cerr<<"sum is "<<sum<<std::endl;
           
            rawTimes.push_back( timer.Stop_Time());

        }
    }

    double avgHand=0,avgIterator=0,raw=0;
    for(unsigned int i=0;i<handTimes.size();i++){
        avgHand+=handTimes[i];
        avgIterator+=iteratorTimes[i];
        raw+=rawTimes[i];
    }
    avgIterator/=handTimes.size();
    avgHand/=handTimes.size();
    raw/=handTimes.size();
    float megs=nParticles*20./float(1<<20);
    std::cerr<<megs<<" MB"<<std::endl;
    std::cerr<<"Iterator "<<avgIterator<<" s "<<megs/avgIterator<<" MB/s"<<std::endl;
    std::cerr<<"Hand "<<avgHand<<" s "<<megs/avgHand<<" MB/s"<<std::endl;
    std::cerr<<"Raw "<<raw<<" s "<<megs/raw<<" MB/s"<<std::endl;

    return 0;

}

