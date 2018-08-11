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

#include "../Partio.h"
#include "../core/ParticleHeaders.h"
#include "ZIP.h"

#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <memory>


namespace Partio
{

using namespace std;

template<ParticleAttributeType ETYPE>
void readGeoAttr(istream& f,const ParticleAttribute& attr,ParticleAccessor& accessor,ParticlesDataMutable::iterator& iterator)
{
    //cout<<"reading "<<TypeName(attr.type)<<endl;
    typedef typename ETYPE_TO_TYPE<ETYPE>::TYPE TYPE;
    TYPE* data=accessor.raw<TYPE>(iterator);
    for(int k=0;k<attr.count;k++){
        f>>data[k];
    }
}

void writeString(std::ostream& output,const char* s){
    const char* p=s;
    output<<"\"";
    while(*p != 0){
        if(*p=='\\' || *p=='"') output<<'\\';
        output<<*p;
        p++;
    }
    output<<"\"";
}

string scanString(istream& input)
{
    // TODO: this code does not check for buf overrun
    // TODO: this code doesn't properly check for FEOF condition
    char buf[4096];
    char *ptr=buf;
    char c;
    while(input.good()){
        input.get(c);
        if(!isspace(c)) break;
    }
    if(!input.good()) return "";

    if(c!='"'){
        while(input.good()){
            *ptr++=c;
            input.get(c);
            if(isspace(c)) break;
        }
    }else{
        while(input.good()){
            input.get(c);
            if(c=='\\'){
                input.get(c);
                *ptr++=c;
            }else if(c=='"'){
                break;
            }else{
                *ptr++=c;
            }
        }
    }
    *ptr++=0;
    return string(buf);
}

ParticlesDataMutable* readGEO(const char* filename,const bool headersOnly)
{
    auto_ptr<istream> input(Gzip_In(filename,ios::in));
    if(!*input){
        cerr<<"Partio: Can't open particle data file: "<<filename<<endl;
        return 0;
    }
    int NPoints=0, NPointAttrib=0;

    ParticlesDataMutable* simple=0;
    if(headersOnly) simple=new ParticleHeaders;
    else simple=create();

    // read NPoints and NPointAttrib
    string word;
    while(input->good()){
        *input>>word;
        if(word=="NPoints") *input>>NPoints;
        else if(word=="NPointAttrib"){*input>>NPointAttrib;break;}
    }
    // skip until PointAttrib
    while(input->good()){
        *input>>word;
        if(word=="PointAttrib") break;
    }
    // read attribute descriptions
    int attrInfoRead = 0;
    
    ParticleAttribute positionAttr=simple->addAttribute("position",VECTOR,3);
    ParticleAccessor positionAccessor(positionAttr);

    vector<ParticleAttribute> attrs;
    vector<ParticleAccessor> accessors;
    while (input->good() && attrInfoRead < NPointAttrib) {
        string attrName, attrType;
        int nvals = 0;
        *input >> attrName >> nvals >> attrType;
        if(attrType=="index"){
            cerr<<"Partio: attr '"<<attrName<<"' of type index (string) found, treating as integer"<<endl;
            int nIndices=0;
            *input>>nIndices;
            ParticleAttribute attribute=simple->addAttribute(attrName.c_str(),INDEXEDSTR,1);
            attrs.push_back(attribute);
            for(int j=0;j<nIndices;j++){
                string indexName;
                //*input>>indexName;
                indexName=scanString(*input);
                //std::cerr<<"Partio:    index "<<j<<" is "<<indexName<<std::endl;
                int id=simple->registerIndexedStr(attribute,indexName.c_str());
                if(id != j){
                    cerr<<"Partio: error on read, expected registeerIndexStr to return index "<<j<<" but got "<<id<<" for string "<<indexName<<endl;
                }
            }
            accessors.push_back(ParticleAccessor(attrs.back()));
            attrInfoRead++;
            
        }else{
            for (int i=0;i<nvals;i++) {
                float defval;
                *input>>defval;
            }
            ParticleAttributeType type;
            // TODO: fix for other attribute types
            if(attrType=="float") type=FLOAT;
            else if(attrType=="vector") type=VECTOR;
            else if(attrType=="int") type=INT;
            else{
                cerr<<"Partio: unknown attribute "<<attrType<<" type... aborting"<<endl;
                type=NONE;
            }
            attrs.push_back(simple->addAttribute(attrName.c_str(),type,nvals));
            accessors.push_back(ParticleAccessor(attrs.back()));
            attrInfoRead++;
        }
    }

    simple->addParticles(NPoints);

    ParticlesDataMutable::iterator iterator=simple->begin();
    iterator.addAccessor(positionAccessor);
    for(size_t i=0;i<accessors.size();i++) iterator.addAccessor(accessors[i]);

    if(headersOnly) return simple; // escape before we try to touch data
    
    
    float fval;
    // TODO: fix
    for(ParticlesDataMutable::iterator end=simple->end();iterator!=end  && input->good();++iterator){
        float* posInternal=positionAccessor.raw<float>(iterator); 
        for(int i=0;i<3;i++) *input>>posInternal[i];
        *input>>fval;
        //cout<<"saw "<<posInternal[0]<<" "<<posInternal[1]<<" "<<posInternal[2]<<endl;
        
        // skip open paren
        char paren = 0;
        *input>> paren;
        if (paren != '(') break;
        
        // read additional attribute values
        for (unsigned int i=0;i<attrs.size();i++){
            switch(attrs[i].type){
                case NONE: assert(false);break;
                case FLOAT: readGeoAttr<FLOAT>(*input,attrs[i],accessors[i],iterator);break;
                case VECTOR: readGeoAttr<VECTOR>(*input,attrs[i],accessors[i],iterator);break;
                case INT: readGeoAttr<INT>(*input,attrs[i],accessors[i],iterator);break;
                case INDEXEDSTR: readGeoAttr<INDEXEDSTR>(*input,attrs[i],accessors[i],iterator);break;
            }
        }
        // skip closing parenthes
        *input >> paren;
        if (paren != ')') break;
    }
    return simple;
}


template<class T>
void writeType(ostream& output,const ParticlesData& p,const ParticleAttribute& attrib,
    const ParticleAccessor& accessor,const ParticlesData::const_iterator& iterator)
{
    const T* data=accessor.raw<T>(iterator);
    for(int i=0;i<attrib.count;i++){
        if(i>0) output<<" ";
        output<<data[i];
    }
}

bool writeGEO(const char* filename,const ParticlesData& p,const bool compressed)
{
    auto_ptr<ostream> output(
        compressed ? 
        Gzip_Out(filename,ios::out)
        :new ofstream(filename,ios::out));

    *output<<"PGEOMETRY V5"<<endl;
    *output<<"NPoints "<<p.numParticles()<<" NPrims "<<1<<endl;
    *output<<"NPointGroups "<<0<<" NPrimGroups "<<0<<endl;
    *output<<"NPointAttrib "<<p.numAttributes()-1<<" NVertexAttrib "<<0<<" NPrimAttrib 1 NAttrib 0"<<endl;

    if(p.numAttributes()-1>0) *output<<"PointAttrib"<<endl;
    ParticleAttribute positionHandle;
    ParticleAccessor positionAccessor(positionHandle);
    bool foundPosition=false;
    vector<ParticleAttribute> handles;
    vector<ParticleAccessor> accessors;
    for(int i=0;i<p.numAttributes();i++){
        ParticleAttribute attrib;
        p.attributeInfo(i,attrib);
        if(attrib.name=="position"){
            positionHandle=attrib;
            positionAccessor=ParticleAccessor(positionHandle);
            foundPosition=true;
        }else{
            handles.push_back(attrib);
            accessors.push_back(ParticleAccessor(handles.back()));
            string typestring;
            if(attrib.type!=INDEXEDSTR){
                switch(attrib.type){
                    case NONE: assert(false);typestring="int";break;
                    case VECTOR: typestring="vector";break;
                    case FLOAT: typestring="float";break;
                    case INT: typestring="int";break;
                    default: break;
                }
                *output<<attrib.name<<" "<<attrib.count<<" "<<typestring;
                for(int k=0;k<attrib.count;k++) *output<<" "<<0;
            }else{
                typestring="index";
                const vector<string>& indexes=p.indexedStrs(attrib);
                *output<<attrib.name<<" "<<attrib.count<<" "<<typestring<<" "<<indexes.size();
                for(size_t k=0;k<indexes.size();k++){
                    *output<<" ";
                    writeString(*output,indexes[k].c_str());
                }
            }
        }
        *output<<endl;
    }
    if(!foundPosition){
        cerr<<"Partio: didn't find attr 'position' while trying to write GEO"<<endl;
        return false;
    }

    ParticlesData::const_iterator iterator=p.begin();
    iterator.addAccessor(positionAccessor);
    for(size_t i=0;i<accessors.size();i++) iterator.addAccessor(accessors[i]);

    for(ParticlesData::const_iterator end=p.end();iterator!=end;++iterator){
        const Data<float,3>& point=positionAccessor.data<Data<float,3> >(iterator);
        *output<<point[0]<<" "<<point[1]<<" "<<point[2]<<" 1";
        if(handles.size()) *output<<" (";
        for(unsigned int aindex=0;aindex<handles.size();aindex++){
            if(aindex>0) *output<<"\t";
            ParticleAttribute& handle=handles[aindex];
            ParticleAccessor& accessor=accessors[aindex];
            switch(handle.type){
                case NONE: assert(false);break;
                case FLOAT: writeType<ETYPE_TO_TYPE<FLOAT>::TYPE>(*output,p,handle,accessor,iterator);break;
                case INT: writeType<ETYPE_TO_TYPE<INT>::TYPE>(*output,p,handle,accessor,iterator);break;
                case VECTOR: writeType<ETYPE_TO_TYPE<VECTOR>::TYPE>(*output,p,handle,accessor,iterator);break;
                case INDEXEDSTR: writeType<ETYPE_TO_TYPE<INDEXEDSTR>::TYPE>(*output,p,handle,accessor,iterator);break;
            }
        }        
        if(handles.size()) *output<<")";
        *output<<endl;
    }

    *output<<"PrimitiveAttrib"<<endl;
    *output<<"generator 1 index 1 papi"<<endl;
    *output<<"Part "<<p.numParticles();
    for(int i=0;i<p.numParticles();i++)
        *output<<" "<<i;
    *output<<" [0]\nbeginExtra"<<endl;
    *output<<"endExtra"<<endl;

    // success
    return true;
}

}
