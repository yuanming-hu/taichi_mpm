

#include <Partio.h>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <cstdlib>

using namespace Partio;

void failure(const std::string& x)
{
    std::cerr<<"teststr failed "<<x<<std::endl;
    exit(1);
}

int main(int argc,char *argv[])
{
    {
        ParticlesDataMutable* p=create();
        ParticleAttribute posAttr=p->addAttribute("position",VECTOR,3);
        ParticleAttribute fileAttr=p->addAttribute("filename",INDEXEDSTR,1);
        ParticleAttribute stateAttr=p->addAttribute("state",INDEXEDSTR,1);
        
        int test1Token=p->registerIndexedStr(fileAttr,"test1");
        int test2Token=p->registerIndexedStr(fileAttr,"test2 with space");

        int aliveState=p->registerIndexedStr(stateAttr,"alive");
        int deadState=p->registerIndexedStr(stateAttr,"dead");
        int zombieState=p->registerIndexedStr(stateAttr,"zombie");
        int stateChoices[]={aliveState,deadState,zombieState};

        for(int i=0;i<10;i++){
            p->addParticle();
            float* f=p->dataWrite<float>(posAttr,i);
            int* tok=p->dataWrite<int>(fileAttr,i);
            int* state=p->dataWrite<int>(stateAttr,i);
            tok[0]=i%2==0 ? test1Token : test2Token;
            state[0]=stateChoices[i%3];
            f[0]=i;
            f[1]=0;
            f[2]=0;
        }


        Partio::write("foo.bgeo",*p);
        p->release();
    }

    {
        ParticlesDataMutable* p=Partio::read("foo.bgeo");
        if(!p) failure("failed to get foo.bgeo");
        ParticleAttribute posAttr,fileAttr,stateAttr;
        if(!p->attributeInfo("position",posAttr)) failure("couldn't get position");
        if(!p->attributeInfo("filename",fileAttr)) failure("couldn't get filnename");
        if(!p->attributeInfo("state",stateAttr)) failure("couldn't get state");
        

        // lookup the string codes in a way that is efficient to check later
        int alive=p->lookupIndexedStr(stateAttr,"alive");
        int dead=p->lookupIndexedStr(stateAttr,"dead");
        int zombie=p->lookupIndexedStr(stateAttr,"zombie");
        int states[]={alive,dead,zombie}; // every ith particle should be asigned these in order
        if(alive == -1 || dead == -1 || zombie == -1){ std::cerr<<"don't have tokens I expect!"<<std::endl;}

        for(int i=0;i<p->numParticles();i++){
            //const int* tok=p->data<int>(fileAttr,i);
            const int* state=p->data<int>(stateAttr,i);

            // Note this is not what you would like to do ideally because it uses strings a lot!
            //std::cerr<<"Particle index "<<i<<" str "<<tok[0]<<" "<<p->indexedStrs(fileAttr)[num]<<
            //    " state is "<<state[0]<<" "<<p->indexedStrs(stateAttr)[state[0]]<<std::endl;

            if(state[0] != states[i%3]) std::cerr<<"TEST FAILED EXPECTED STATE "<<states[i%3]<<" got "<<state[0]<<std::endl;
            if(state[0]==alive) std::cerr<<"Particle "<<i<<" is alive"<<std::endl;
        }

        p->release();
    }
    return 0;
}


