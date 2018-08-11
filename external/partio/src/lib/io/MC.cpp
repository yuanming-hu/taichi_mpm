/*
PARTIO SOFTWARE
Copyright (c) 2011 Disney Enterprises, Inc. and Contributors,  All rights reserved

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
#include "PartioEndian.h" // read/write big-endian file
#include "ZIP.h" // for zip file

#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <memory>

namespace Partio
{
//#define PartioBIG_ENDIAN

using namespace std;

// TODO: convert this to use iterators like the rest of the readers/writers

std::string GetString(std::istream& input, unsigned int size){
    char* tmp = new char [size];
    input.read(tmp, size);
    std::string result(tmp);

    // fix read tag error (ex: DBLA --> DBLAi, why !!)
    if(result.size() > size){
        result.resize(size);
    }

    delete [] tmp;
    return result;
}

typedef struct{
    std::string name;
    std::string type;
    unsigned int numParticles;
    unsigned int blocksize;
} Attribute_Header;


bool ReadAttrHeader(std::istream& input, Attribute_Header& attribute){
    char tag[4];
    input.read(tag, 4); // CHNM

    int chnmSize;
    read<BIGEND>(input, chnmSize);
    if(chnmSize%4 > 0){
        chnmSize = chnmSize - chnmSize%4 + 4;
    }
    attribute.name = GetString(input, chnmSize);
    attribute.name = attribute.name.substr(attribute.name.find_first_of("_")+1);

    input.read(tag, 4); // SIZE
    int dummy;
    read<BIGEND>(input, dummy); // 4

    read<BIGEND>(input, attribute.numParticles);

    attribute.type = GetString(input, 4);

    read<BIGEND>(input, attribute.blocksize);

    return true;
}

int CharArrayLen(char** charArray){
    int i = 0;
    if(charArray != false){
        while(charArray[i] != '\0'){
            i++;
        }
    }
    return i;
}

bool IsStringInCharArray(std::string target, char** list){
    //std::cout << "Is " << target << " in ";
    for(int i = 0; i < CharArrayLen(list); i++){
        //std::cout << std::string(list[i]) << " ";
        if(target == std::string(list[i])){
            //std::cout << "? (YES)" << std::endl;
            return true;
        }
    }
    //std::cout << "? (NO)" << std::endl;
    return false;
}

static const int MC_MAGIC = ((((('F'<<8)|'O')<<8)|'R')<<8)|'4';
static const int HEADER_SIZE = 56;

ParticlesDataMutable* readMC(const char* filename, const bool headersOnly){

    std::auto_ptr<std::istream> input(Gzip_In(filename,std::ios::in|std::ios::binary));
    if(!*input){
        std::cerr << "Partio: Unable to open file " << filename << std::endl;
        return 0;
    }

    int magic;
    read<BIGEND>(*input, magic);
    if(MC_MAGIC != magic){
        std::cerr << "Partio: Magic number '" << magic << "' of '" << filename << "' doesn't match mc magic '" << MC_MAGIC << "'" << std::endl;
        return 0;
    }

    int headerSize;
    read<BIGEND>(*input, headerSize);

    int dummy; // tmp1, tmp2, num1, tmp3, tmp4, num2, num3, tmp5, num4, num5, blockTag
    for(int i = 0; i < 10; i++){
        read<BIGEND>(*input, dummy);
        //std::cout << dummy << std::endl;
    }

    char tag[4];
    input->read(tag, 4); // FOR4

    int blockSize;
    read<BIGEND>(*input, blockSize);

    // Allocate a simple particle with the appropriate number of points
    ParticlesDataMutable* simple=0;
    if(headersOnly){
        simple = new ParticleHeaders;
    }
    else{
        simple=create();
    }

    int numParticles = 0;
    input->read(tag, 4); // MYCH
    while(((int)input->tellg()-HEADER_SIZE) < blockSize){
        Attribute_Header attrHeader;
        ReadAttrHeader(*input, attrHeader);

        if(attrHeader.name == std::string("id")){
            numParticles = attrHeader.numParticles;
        }

        if(attrHeader.blocksize/sizeof(double) == 1){ // for who ?
            input->seekg((int)input->tellg() + attrHeader.blocksize);
            continue;
        }
#if 0 // TODO: if we ever put back attributes re-enable this
        if(attributes && (IsStringInCharArray(attrHeader.name, attributes)==false)){
            input->seekg((int)input->tellg() + attrHeader.blocksize);
            continue;
        }
#endif

        if(attrHeader.type == std::string("FVCA")){
            input->seekg((int)input->tellg() + attrHeader.blocksize);
            simple->addAttribute(attrHeader.name.c_str(), VECTOR, 3);
        }
        else if(attrHeader.type == std::string("DBLA")){
			input->seekg((int)input->tellg() + attrHeader.blocksize);
			if(attrHeader.name == "id"){
				simple->addAttribute(attrHeader.name.c_str(), INT, 1);
			}
            else{
				simple->addAttribute(attrHeader.name.c_str(), FLOAT, 1);
			}
        }
        else
		{
            input->seekg((int)input->tellg() + attrHeader.blocksize);
            std::cerr << "Partio: Attribute '" << attrHeader.name << " " << attrHeader.type << "' cannot map type" << std::endl;
        }
    }
    simple->addParticles(numParticles);

    // If all we care about is headers, then return.--
    if(headersOnly){
        return simple;
    }
    //cout << "==============================================================" << endl;
    input->seekg(HEADER_SIZE);
    input->read(tag, 4); // MYCH
    while((int)input->tellg()-HEADER_SIZE < blockSize){
        Attribute_Header attrHeader;
        ReadAttrHeader(*input, attrHeader);

        if(attrHeader.blocksize/sizeof(double) == 1){ // for who ?
            input->seekg((int)input->tellg() + attrHeader.blocksize);
            continue;
        }

        ParticleAttribute attrHandle;
        if(simple->attributeInfo(attrHeader.name.c_str(), attrHandle) == false){
            input->seekg((int)input->tellg() + attrHeader.blocksize);
            continue;
        }

        Partio::ParticlesDataMutable::iterator it = simple->begin();
        Partio::ParticleAccessor accessor(attrHandle);
        it.addAccessor(accessor);

		//std::cout << attrHeader.name << std::endl;
        if (attrHeader.type == std::string("DBLA")){
			if  (attrHeader.name == "id")
			{
				for (int i = 0; i < simple->numParticles(); i++)
				{
                double tmp;
                read<BIGEND>(*input, tmp);
                int* data = simple->dataWrite<int>(attrHandle, i);
                data[0] = (int)tmp;
				}
			}
			else{
				for (int i = 0; i < simple->numParticles(); i++){
					double tmp;
					read<BIGEND>(*input, tmp);
					float* data = simple->dataWrite<float>(attrHandle, i);
					data[0] = (float)tmp;
				}
			}
        }
        else if(attrHeader.type == std::string("FVCA")){
            for(Partio::ParticlesDataMutable::iterator end = simple->end(); it != end; ++it){
                input->read(accessor.raw<char>(it), sizeof(float)*attrHandle.count);
            }
            it = simple->begin();
            for(Partio::ParticlesDataMutable::iterator end = simple->end(); it != end; ++it){
                float* data = accessor.raw<float>(it);
                for(int i = 0; i < attrHandle.count; i++){
                    BIGEND::swap(data[i]);
                    //data[k]=buffer[attrOffsets[attrIndex]+k];
                    //data[i] = endianSwap<float>(data[i]);
                }
            }


        }
        /*
        if(0){
            std::cout << typeName << std::endl;
            std::cerr << "Partio: " << filename << " had unknown attribute spec " << typeName << " " << name << std::endl;
            simple->release();
            return 0;
        }*/

    }
    return simple;

}
/*
bool dgMc::open(string filePath){
    timeval t1, t2;

    ReadStr(file, 4); // MYCH

    while(((int)file.tellg()-8-40-8) < header.blockSize){
        ReadStr(file, 4); // CHNM

        int chnmSize = ReadInt(file);
        if(chnmSize%4 > 0){
            chnmSize = chnmSize - chnmSize%4 + 4;
        }
        string attrname = ReadChar(file, chnmSize);
        attrname = attrname.substr(attrname.find_first_of("_")+1);
        //cout << attrname << endl;

        ReadStr(file, 4); // SIZE
        ReadInt(file);

        int arrayLength = ReadInt(file);
        string format = ReadStr(file, 4); // DBLA or FVCA
        int bufferLength = ReadInt(file);

        string DBLA("DBLA");
        //cout << format << " " << arrayLength << " " << bufferLength << endl;
        if(format == string("DBLA")){
            ReadDoubleArray(file, (int)file.tellg(), &_doubleArray[attrname], arrayLength);
            //file.seekg((int)file.tellg() + bufferLength);
        }
        else{ // FVCA
            ReadVectorArray(file, (int)file.tellg(), &_vectorArray[attrname], arrayLength);
            if(attrname == string("position")){
                //ReadVectorArray(file, (int)file.tellg(), &_vectorArray[attrname], arrayLength);
                //file.seekg((int)file.tellg() + bufferLength);
            }
            else{

                //file.seekg((int)file.tellg() + bufferLength);
            }
            //return true;
        }

        if(attrname == string("id")){
            _numParticles = arrayLength;
        }
    }
    return true;
}

int main(){
    cout << "go" << endl;

    timeval t1, t2;
    double elapsedTime;
    gettimeofday(&t1, NULL);

    for(int i = 0; i < 1; i++){
        dgMc mc;
        mc.open(string("/dept/rdworks/jinkuen/testfile/mc/real_nParticleShape1Frame47.mc"));
        cout << mc.numParticles() << " ";
        for(int i = 0; i < mc.count(); i++){
            cout << mc.list(dgParticle::All)[i] << " ";
        }
        mc.clear();
    }
    cout << endl;

    gettimeofday(&t2, NULL);
    elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
    elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
    cout << elapsedTime << " ms.\n";

    return 1;
}

*/

}
