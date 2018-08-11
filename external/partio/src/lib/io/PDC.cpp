/*
PARTIO SOFTWARE
Copyright (c) 2011 Disney Enterprises, Inc. and Contributors,  All rights reserved

 keypress events  also added the PTS file format  (all need cleanup)
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

Format Contributed by github user: Jinkuen
Modifications from: github user: redpawfx (redpawFX@gmail.com)  and Luma Pictures  2011

*/

#include "../Partio.h"
#include "../core/ParticleHeaders.h"
#include "PartioEndian.h"
#include "ZIP.h"

#include <iostream>
#include <fstream>
#include <string>
#include <memory>

namespace Partio{

using namespace std;

static const int PDC_MAGIC = (((((' '<<8)|'C')<<8)|'D')<<8)|'P'; // " CDP"

typedef struct{
    int magic;
    int version;
    int bitorder;
    int tmp1;
    int tmp2;
    int numParticles;
    int numAttrs;
} PDC_HEADER;

string readName(istream& input){
    int nameLen = 0;
    read<BIGEND>(input, nameLen);
    char* name = new char[nameLen];
    input.read(name, nameLen);
    string result(name, name+nameLen);
    delete [] name;
    return result;
}

ParticlesDataMutable* readPDC(const char* filename, const bool headersOnly){

    auto_ptr<istream> input(Gzip_In(filename,std::ios::in|std::ios::binary));
    if(!*input){
        std::cerr << "Partio: Unable to open file " << filename << std::endl;
        return 0;
    }

    PDC_HEADER header;
    input->read((char*)&header, sizeof(header));
    if(PDC_MAGIC != header.magic){
        std::cerr << "Partio: Magic number '" << header.magic << "' of '" << filename << "' doesn't match pdc magic '" << PDC_MAGIC << "'" << std::endl;
        return 0;
    }

    BIGEND::swap(header.numParticles);
    BIGEND::swap(header.numAttrs);

    ParticlesDataMutable* simple = headersOnly ? new ParticleHeaders: create();
    simple->addParticles(header.numParticles);

    for(int attrIndex = 0; attrIndex < header.numAttrs; attrIndex++){
        // add attribute
        ParticleAttribute attr;
        string attrName = readName(*input);
        int type;
        read<BIGEND>(*input, type);
        if(type == 3){
            attr = simple->addAttribute(attrName.c_str(), FLOAT, 1);
        }
        else if(type == 5){
            attr = simple->addAttribute(attrName.c_str(), VECTOR, 3);
        }

        // if headersOnly, skip
        if(headersOnly){
            input->seekg((int)input->tellg() + header.numParticles*sizeof(double)*attr.count);
            continue;
        }
        else{
            double tmp[3];
            for(int partIndex = 0; partIndex < simple->numParticles(); partIndex++){
                for(int dim = 0; dim < attr.count; dim++){
                    read<BIGEND>(*input, tmp[dim]);
                    simple->dataWrite<float>(attr, partIndex)[dim] = (float)tmp[dim];
                }
            }
        }
    }

    return simple;
}

bool writePDC(const char* filename,const ParticlesData& p,const bool compressed){
    auto_ptr<ostream> output(
        compressed ?
        Gzip_Out(filename,ios::out|ios::binary)
        :new std::ofstream(filename,ios::out|ios::binary));

    if(!*output){
        cerr << "Partio Unable to open file " << filename << endl;
        return false;
    }

    // write .pdc header
    write<LITEND>(*output, PDC_MAGIC);
    write<BIGEND>(*output, (int)1); // version
    write<BIGEND>(*output, (int)1); // bitorder
    write<BIGEND>(*output, (int)0); // tmp1
    write<BIGEND>(*output, (int)0); // tmp2
    write<BIGEND>(*output, (int)p.numParticles());
    write<BIGEND>(*output, (int)p.numAttributes());

    for(int attrIndex = 0; attrIndex < p.numAttributes(); attrIndex++){
        ParticleAttribute attr;
        p.attributeInfo(attrIndex,attr);

        // write attribute name
        write<BIGEND>(*output, (int)attr.name.length());
        output->write(attr.name.c_str(), (int)attr.name.length());

        // write type
        int count = 1; // FLOAT
        if(attr.type == VECTOR){
            count = 3;
        }
        write<BIGEND>(*output, (int)(count+2));

        // write data
        for(int partIndex = 0; partIndex < p.numParticles(); partIndex++){
            const float* data = p.data<float>(attr, partIndex);
            for(int dim = 0; dim < count; dim++){
                write<BIGEND>(*output, (double)data[dim]);
            }
        }
    }
    return true;
}

}// end of namespace Partio
