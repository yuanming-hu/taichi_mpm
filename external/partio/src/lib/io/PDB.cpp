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

This code is partially based on the Gifts/readpdb directory of Autodesk Maya
*/

#include "../Partio.h"
#include "../core/ParticleHeaders.h"
namespace PDB{
#include "pdb.h"
}
#include "PartioEndian.h"
#include "ZIP.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <memory>
#include <string.h>
namespace Partio
{

using namespace std;

template<int bits> struct PDB_POLICY;
template<> struct PDB_POLICY<32>
{
    typedef PDB::PDB_Header32 HEADER;
    typedef PDB::PDBdata32 DATA;
    typedef PDB::Channel32 CHANNEL;
    typedef PDB::Channel_Data32 CHANNEL_DATA;
    typedef PDB::Channel_io_Header CHANNEL_IO;
};
template<> struct PDB_POLICY<64>
{
    typedef PDB::PDB_Header HEADER;
    typedef PDB::PDBdata DATA;
    typedef PDB::Channel CHANNEL;
    typedef PDB::Channel_Data CHANNEL_DATA;
    typedef PDB::Channel_io_Header CHANNEL_IO;
};

string GetString(istream& input,bool& error)
{
    //cerr<<"enter"<<endl;
    const char terminator='\0';
    // TODO: make this check for FEOF condition! and also more efficient
    char c=' ';
    string s="";
    error=true;
    while (input)
    {
        input.read(&c,sizeof(char));
        if(c == terminator){error=false;break;}
        s += c;
    }
    return s;
}


template<int bits> ParticlesDataMutable* readPDBHelper(const char* filename,const bool headersOnly)
{

    auto_ptr<istream> input(Gzip_In(filename,ios::in|ios::binary));
    if(!*input){
        cerr<<"Partio: Unable to open file "<<filename<<endl;
        return 0;
    }

    // Use simple particle since we don't have optimized storage.
    ParticlesDataMutable* simple=0;
    if(headersOnly) simple=new ParticleHeaders;
    else simple=create();

    // Read header and add as many particles as found
    typename PDB_POLICY<bits>::HEADER header;

    input->read((char*)&header,sizeof(typename PDB_POLICY<bits>::HEADER));
    if(header.magic != PDB_MAGIC){
        cerr<<"Partio: failed to get PDB magic"<<endl;
        return 0;
    }

    simple->addParticles(header.data_size);
    
    for(unsigned int i=0;i<header.num_data;i++){
        typename PDB_POLICY<bits>::CHANNEL_IO channelIOHeader;
        input->read((char*)&channelIOHeader,sizeof(channelIOHeader));
        typename PDB_POLICY<bits>::CHANNEL channelHeader;
        input->read((char*)&channelHeader,sizeof(channelHeader));
        bool error;
        string name=GetString(*input,error);
        if(error){
            simple->release();
            return 0;
        }

        typename PDB_POLICY<bits>::CHANNEL_DATA channelData;
        input->read((char*)&channelData,sizeof(channelData));

        
        ParticleAttributeType type;
        switch(channelHeader.type){
            case PDB_VECTOR: type=VECTOR;break;
            case PDB_REAL: type=FLOAT;break;
            case PDB_LONG: type=INT;break;
            default: type=NONE;break;
        }
        int size=header.data_size*channelData.datasize;

        // Read data or skip if we haven't found appropriate type handle
        if(type==NONE){
            char buf[1024];
            int toSkip=size;
            while(toSkip>0){
                input->read(buf,min(toSkip,1024));
                toSkip-=1024;
            }
            cerr<<"Partio: Attribute '"<<name<<"' cannot map type"<<endl;
        }else{
            int count=channelData.datasize/TypeSize(type);
            ParticleAttribute attrHandle=simple->addAttribute(name.c_str(),type,count);
            if(headersOnly){
                char buf[1024];
                int toSkip=size;
                while(toSkip>0){
                    input->read(buf,min(toSkip,1024));
                    toSkip-=1024;
                }
            }else{
                Partio::ParticlesDataMutable::iterator it=simple->begin();
                Partio::ParticleAccessor accessor(attrHandle);
                it.addAccessor(accessor);

                for(Partio::ParticlesDataMutable::iterator end=simple->end();it!=end;++it){
                    input->read(accessor.raw<char>(it),sizeof(float)*attrHandle.count);
                }
            }
        }
    }
    return simple;
}

template<int bits>
bool writePDBHelper(const char* filename,const ParticlesData& p,const bool compressed)
{
    auto_ptr<ostream> output(
        compressed ? 
        Gzip_Out(filename,ios::out|ios::binary)
        :new ofstream(filename,ios::out|ios::binary));

    if(!*output){
        cerr<<"Partio Unable to open file "<<filename<<endl;
        return false;
    }

    typename PDB_POLICY<bits>::HEADER h32;
    memset(&h32,0,sizeof(typename PDB_POLICY<bits>::HEADER));
    h32.magic=PDB_MAGIC;
    h32.swap=1;
    h32.version=1.0;
    h32.time=0.0;
    h32.data_size=p.numParticles();
    h32.num_data=p.numAttributes();
    for(int k=0;k<32;k++) h32.padding[k]=0;
    h32.data=0;
    output->write((char*)&h32,sizeof(typename PDB_POLICY<bits>::HEADER));
    
    for(int attrIndex=0;attrIndex<p.numAttributes();attrIndex++){
        ParticleAttribute attr;
        p.attributeInfo(attrIndex,attr);

        typename PDB_POLICY<bits>::CHANNEL_IO cio;
        typename PDB_POLICY<bits>::CHANNEL channel;
        typename PDB_POLICY<bits>::CHANNEL_DATA data_header;
        memset(&cio,0,sizeof(typename PDB_POLICY<bits>::CHANNEL_IO));
        memset(&channel,0,sizeof(typename PDB_POLICY<bits>::CHANNEL));
        memset(&data_header,0,sizeof(typename PDB_POLICY<bits>::CHANNEL_DATA));

        cio.magic=0;
        cio.swap=1;
        cio.encoding=0;
        cio.type=0;
        output->write((char*)&cio,sizeof(typename PDB_POLICY<bits>::CHANNEL_IO));
 
        // TODO: assert cproper count!
        channel.name=0;
        switch(attr.type){
            case INDEXEDSTR:channel.type=PDB_LONG;break;
            case INT:channel.type=PDB_LONG;break;
            case FLOAT:channel.type=PDB_REAL;break;
            case VECTOR:channel.type=PDB_VECTOR;break;
            default: assert(false);
        }
        channel.size=0;
        channel.active_start=0;
        channel.active_end=h32.data_size-1;
        channel.hide=0;
        channel.disconnect=0;
        channel.data=0;
        channel.link=0;
        channel.next=0;
        output->write((char*)&channel,sizeof(channel));
        output->write(attr.name.c_str(),attr.name.length()*sizeof(char)+1);
        data_header.type=channel.type;
        data_header.datasize=attr.count*sizeof(float);
        data_header.blocksize=p.numParticles();
        data_header.num_blocks=1;
        data_header.block=0;
        output->write((char*)&data_header,sizeof(data_header));

        Partio::ParticlesData::const_iterator it=p.begin();
        Partio::ParticleAccessor accessor(attr);
        it.addAccessor(accessor);

        for(Partio::ParticlesData::const_iterator end=p.end();it!=end;++it){
            output->write(accessor.raw<char>(it),sizeof(float)*attr.count);
        }
    }
    return true;
}

ParticlesDataMutable* readPDB32(const char* filename,const bool headersOnly)
{return readPDBHelper<32>(filename,headersOnly);}

ParticlesDataMutable* readPDB64(const char* filename,const bool headersOnly)
{return readPDBHelper<64>(filename,headersOnly);}

bool writePDB32(const char* filename,const ParticlesData& p,const bool compressed)
{return writePDBHelper<32>(filename,p,compressed);}

bool writePDB64(const char* filename,const ParticlesData& p,const bool compressed)
{return writePDBHelper<64>(filename,p,compressed);}

ParticlesDataMutable* readPDB(const char* filename,const bool headersOnly)
{
    auto_ptr<istream> input(Gzip_In(filename,ios::in|ios::binary));
    if(!*input){
        cerr<<"Partio: Unable to open file "<<filename<<endl;
        return 0;
    }
    // Read header and add as many particles as found
    PDB_POLICY<64>::HEADER header;
    input->read((char*)&header,sizeof(header));
    if(header.magic != PDB_MAGIC){
        cerr<<"Partio: failed to get PDB magic"<<endl;
        return 0;
    }
    // Now read a channel io ... and see if the the swap is zero or one and encoding is zero. If so then we probably god a good thing
    // if not, then we assume 64 bit.
    PDB_POLICY<64>::CHANNEL_IO channelIOHeader;
    input->read((char*)&channelIOHeader,sizeof(channelIOHeader));
    //cout<<"we got channel io as "<<int(channelIOHeader.type)<<" swap is "<<channelIOHeader.swap<<endl;
    if(channelIOHeader.type > 5  || channelIOHeader.type < 0 || (channelIOHeader.swap != 1 && channelIOHeader.swap != 0)){
        return readPDBHelper<32>(filename,headersOnly);
    }else{
        return readPDBHelper<64>(filename,headersOnly);
    }
}

bool writePDB(const char* filename,const ParticlesData& p,const bool compressed)
{return writePDBHelper<32>(filename,p,compressed);}

}
