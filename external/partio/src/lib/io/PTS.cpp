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

Format Contributed by github user: redpawfx (redpawFX@gmail.com)  and Luma Pictures  2011
Some code for this format  was helped along  by referring to an implementation by
*/
#include "../Partio.h"
#include "../core/ParticleHeaders.h"
#include "ZIP.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cassert>
#include <memory>
#include <cstring>

namespace Partio
{

using namespace std;

// TODO: convert this to use iterators like the rest of the readers/writers

ParticlesDataMutable* readPTS(const char* filename,const bool headersOnly)
{
    auto_ptr<istream> input(Gzip_In(filename,ios::in|ios::binary));
    if (!*input)
    {
        cerr<<"Partio: Can't open particle data file: "<<filename<<endl;
        return 0;
    }

    ParticlesDataMutable* simple=0;
    if (headersOnly) simple=new ParticleHeaders;
    else simple=create();

    // read NPoints and NPointAttrib
    string word;

    vector<string> attrNames;
    vector<ParticleAttribute> attrs;

	/// we're going to fake the ID attribute since there's no such thing in PTS Files
	attrNames.push_back((string)"id");
	attrs.push_back(simple->addAttribute(attrNames[0].c_str(),Partio::INT,1));

    /// FORMAT  is  up to 8 elements  space delimited { posX posY posZ  remission quality  red green blue }
    // since there's no header data to parse we're just going to hard code this

    /* Start from the beginning */
	input->seekg(0,ios::beg);

    /* Determine amount of values per line */
    char line[1024];
    /* Jump over first line. */
	input->getline(line,1024);
    input->getline(line,1024);
    int valcount = 0;
    char * pch = strtok( line, "\t " );
    while ( pch )
    {
        if ( *pch != 0 && *pch != '\n' )
		{
            valcount++;
        }
        pch = strtok( NULL, "\t " );
    }


    switch ( valcount )
	{
		case 3:  // position only
		{
			attrNames.push_back((string)"position");
			attrs.push_back(simple->addAttribute(attrNames[1].c_str(),Partio::VECTOR,3));
		}
		break;
		case 4:  // position and  remission
		{
			attrNames.push_back((string)"position");
			attrNames.push_back((string)"remission");
			attrs.push_back(simple->addAttribute(attrNames[1].c_str(),Partio::VECTOR,3));
			attrs.push_back(simple->addAttribute(attrNames[2].c_str(),Partio::FLOAT,1));
		}
		break;
		case 6: //  position and RGB
		{
			attrNames.push_back((string)"position");
			attrNames.push_back((string)"pointColor");
			attrs.push_back(simple->addAttribute(attrNames[1].c_str(),Partio::VECTOR,3));
			attrs.push_back(simple->addAttribute(attrNames[2].c_str(),Partio::VECTOR,3));
		}
		break;
		case 7: // position remission and  RGB
		{
			attrNames.push_back((string)"position");
			attrNames.push_back((string)"remission");
			attrNames.push_back((string)"pointColor");
			attrs.push_back(simple->addAttribute(attrNames[1].c_str(),Partio::VECTOR,3));
			attrs.push_back(simple->addAttribute(attrNames[2].c_str(),Partio::FLOAT,1));
			attrs.push_back(simple->addAttribute(attrNames[3].c_str(),Partio::VECTOR,3));
		}
		break;
		case 8: // everything
		{
			attrNames.push_back((string)"position");
			attrNames.push_back((string)"remission");
			attrNames.push_back((string)"quality");
			attrNames.push_back((string)"pointColor");
			attrs.push_back(simple->addAttribute(attrNames[1].c_str(),Partio::VECTOR,3));
			attrs.push_back(simple->addAttribute(attrNames[2].c_str(),Partio::FLOAT,1));
			attrs.push_back(simple->addAttribute(attrNames[3].c_str(),Partio::INT,1));
			attrs.push_back(simple->addAttribute(attrNames[4].c_str(),Partio::VECTOR,3));
		}
		break;
		default:
		{
			return 0;
		}
		break;

    }

    input->seekg(0,ios::beg);

    unsigned int num=0;
    simple->addParticles(num);
    if (headersOnly) return simple; // escape before we try to touch data

    if (input->good())
    { // garbage count at top of  file
		char junk[1024];
        input->getline(junk,1024);
    }

    // Read actual particle data
    if (!input->good()) {
        simple->release();
        return 0;
    }

    // we have to read line by line, because data is not  clean and consistent so we skip any lines that dont' conform

    for (unsigned int particleIndex=0;input->good();)
    {
        string token = "";
        char line[1024];
        input->getline(line, 1024);

        stringstream ss(line);

        float lineData[8];
        int i = 0;

        while (ss >> token)
        {
            stringstream foo(token);
            float x;
            foo >> x;
			lineData[i] = x;
            i++;
        }

        if (i == valcount)
        {
            simple->addParticle();
            for (unsigned int attrIndex=0;attrIndex<attrs.size();attrIndex++)
            {
                if (attrs[attrIndex].type==Partio::INT )
                {
					int* data=simple->dataWrite<int>(attrs[attrIndex],particleIndex);
					if (attrs[attrIndex].name == "id")
					{
						data[0]=particleIndex;
					}
					else
					{
						data[0] = (int)lineData[4];
					}


                }
                else if (attrs[attrIndex].type==Partio::FLOAT || attrs[attrIndex].type==Partio::VECTOR)
                {
                    float* data=simple->dataWrite<float>(attrs[attrIndex],particleIndex);

					if (attrs[attrIndex].name == "pointColor")
					{
						// 8 bit color  conversion
						data[0]=lineData[4]/255;
						data[1]=lineData[5]/255;
						data[2]=lineData[6]/255;
					}
					else if (attrs[attrIndex].name == "position")
					{
						// position, we flip y/z
						data[0]=lineData[0];
						data[1]=lineData[2];
						data[2]=lineData[1];
					}
					else if (attrs[attrIndex].name == "remission")
					{
						data[0] = lineData[3];
					}

                }
            }
            particleIndex++;
        }
    }
    return simple;
}

/// THIS DOESENT WORK YET>>
/*
bool writePTS(const char* filename,const ParticlesData& p,const bool compressed)
{
    auto_ptr<ostream> output(
        compressed ?
        Gzip_Out(filename,ios::out|ios::binary)
        :new ofstream(filename,ios::out|ios::binary));

    *output<<"ATTRIBUTES"<<endl;

    vector<ParticleAttribute> attrs;
    for (int aIndex=0;aIndex<p.numAttributes();aIndex++){
        attrs.push_back(ParticleAttribute());
        p.attributeInfo(aIndex,attrs[aIndex]);
        *output<<" "<<attrs[aIndex].name;
    }
    *output<<endl;

    // TODO: assert right count
    *output<<"TYPES"<<endl;
    for (int aIndex=0;aIndex<p.numAttributes();aIndex++){
        switch(attrs[aIndex].type){
            case FLOAT: *output<<" R";break;
            case VECTOR: *output<<" V";break;
            case INDEXEDSTR:
            case INT: *output<<" I";break;
            case NONE: assert(false); break; // TODO: more graceful
        }
    }
    *output<<endl;

    *output<<"NUMBER_OF_PARTICLES: "<<p.numParticles()<<endl;
    *output<<"BEGIN DATA"<<endl;

    for(int particleIndex=0;particleIndex<p.numParticles();particleIndex++){
        for(unsigned int attrIndex=0;attrIndex<attrs.size();attrIndex++){
            if(attrs[attrIndex].type==Partio::INT || attrs[attrIndex].type==Partio::INDEXEDSTR){
                const int* data=p.data<int>(attrs[attrIndex],particleIndex);
                for(int count=0;count<attrs[attrIndex].count;count++)
                    *output<<data[count]<<" ";
            }else if(attrs[attrIndex].type==Partio::FLOAT || attrs[attrIndex].type==Partio::VECTOR){
                const float* data=p.data<float>(attrs[attrIndex],particleIndex);
                for(int count=0;count<attrs[attrIndex].count;count++)
                    *output<<data[count]<<" ";
            }
        }
        *output<<endl;
    }
    return true;

}
*/

}

