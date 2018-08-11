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

Format Contributed by github user: K240
Some Code for this format  was helped along  by referring to an implementation by  Bo Schwartzstein and ThinkBox Software   THANKS!
Modifications from: github user: redpawfx (redpawFX@gmail.com)  and Luma Pictures  2011


*/
#include "../Partio.h"
#include "PartioEndian.h"
#include "../core/ParticleHeaders.h"
#include <string.h>


//#define USE_ILMHALF    // use Ilm's Half library
#define AUTO_CASES    // auto upcase ie:position => Position

#ifdef USE_ILMHALF
#include <half.h>
#endif

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <zlib.h>

namespace Partio{

#define OUT_BUFSIZE		(4096)

typedef struct FileHeadder {
    unsigned char	magic[8];
    unsigned int	headersize;
    unsigned char	signature[32];
    unsigned int	version;
    unsigned long long numParticles;
} PRT_File_Headder;

typedef struct Channel {
    unsigned char	name[32];
    unsigned int	type;
    unsigned int	arity;
    unsigned int	offset;
} Channel;

static unsigned char magic[8] = {192, 'P', 'R', 'T', '\r', '\n', 26, '\n'};
static unsigned char signature[32] = {
    0x45,0x78,0x74,0x65,0x6e,0x73,0x69,0x62,0x6c,0x65,0x20,0x50,0x61,0x72,0x74,0x69,
    0x63,0x6c,0x65,0x20,0x46,0x6f,0x72,0x6d,0x61,0x74,0x00,0x00,0x00,0x00,0x00,0x00
    };

#ifndef USE_ILMHALF
union f_ui {
    unsigned int ui;
    float f;
};

static f_ui half2float[65536] = {
#include "half2float.h"
};
#endif

static bool read_buffer(std::istream& is, z_stream& z, char* in_buf, void* p, size_t size) {
    z.next_out=(Bytef*)p;
    z.avail_out=(uInt)size;

    while (z.avail_out) {
        if ( z.avail_in == 0 ) {
            if (!is.eof()) {
                z.next_in = (Bytef*)in_buf;
                is.read((char*)z.next_in, OUT_BUFSIZE);
                if (is.bad()) {
                    std::cerr<<"read error "<<std::endl;;
                    return false;
                }
                z.avail_in = (uInt)is.gcount();
            }
        }
        int ret = inflate( &z, Z_BLOCK  );
        if ( ret != Z_OK && ret != Z_STREAM_END ) {
            std::cerr<<"Zlib error "<<z.msg<<std::endl;;
            return false;
        }
    }
    return true;
}

static bool write_buffer(std::ostream& os, z_stream& z, char* out_buf, void* p, size_t size, bool flush) {
    z.next_in=(Bytef*)p;
    z.avail_in=(uInt)size;
    while (z.avail_in!=0 || flush) {
	z.next_out = (Bytef*)out_buf;
	z.avail_out = OUT_BUFSIZE;
	int ret=deflate(&z,flush?Z_FINISH:Z_NO_FLUSH);
	if (!(ret!=Z_BUF_ERROR && ret!=Z_STREAM_ERROR)) {
            std::cerr<<"Zlib error "<<z.msg<<std::endl;;
            return false;
        }
        int	generated_output=(int)(z.next_out-(Bytef*)out_buf);
        os.write((char*)out_buf,generated_output);
        if (ret==Z_STREAM_END) break;
    }
    return true;
}



ParticlesDataMutable* readPRT(const char* filename,const bool headersOnly)
{
    std::auto_ptr<std::istream> input(new std::ifstream(filename,std::ios::in|std::ios::binary));
    if (!*input) {
        std::cerr<<"Partio: Unable to open file "<<filename<<std::endl;
        return 0;
    }

    // Use simple particle since we don't have optimized storage.
    ParticlesDataMutable* simple=0;
    if (headersOnly) simple=new ParticleHeaders;
    else simple=create();

    FileHeadder header;
    input->read((char*)&header,sizeof(FileHeadder));

    if (memcmp(header.magic, magic, sizeof(magic))) {
        std::cerr<<"Partio: failed to get PRT magic"<<std::endl;
        return 0;
    }
    int reserve=0;
    int channels=0;
    int channelsize=0;
    read<LITEND>(*input,reserve);		// reserved
    read<LITEND>(*input,channels);		// number of channel
    read<LITEND>(*input,channelsize);	// size of channel

    simple->addParticles((const int)header.numParticles);

    std::vector<Channel> chans;
    std::vector<ParticleAttribute> attrs;

    for (int i=0; i<channels; i++) {
        Channel ch;
        input->read((char*)&ch, sizeof(Channel));
        ParticleAttributeType type=NONE;
        switch (ch.type) {
        case 0:	// int16
        case 1:	// int32
        case 2:	// int64
            type = INT;
            break;
        case 3:	// float16
        case 4:	// float32
        case 5:	// float64
            if (ch.arity == 3)
                type = VECTOR;
            else
                type = FLOAT;
            break;
        case 6:	// uint16
        case 7:	// uint32
        case 8:	// uint64
            type = INT;
            break;
        case 9:	// int8
        case 10:// uint8
            type = INT;
            break;
        }
        if (type != NONE) {
#ifdef AUTO_CASES
            if (ch.name[0] >= 'A' && ch.name[0] <= 'Z') {
                ch.name[0] += 0x20;
            }
#endif
            std::string name((char*)ch.name);
            ParticleAttribute attrHandle=simple->addAttribute(name.c_str(),type,ch.arity);
            chans.push_back(ch);
            attrs.push_back(attrHandle);
        }
    }

    if (headersOnly) return simple;

    z_stream z;
    z.zalloc = Z_NULL;z.zfree = Z_NULL;z.opaque = Z_NULL;
    if (inflateInit( &z ) != Z_OK) {
        std::cerr<<"Zlib inflateInit error"<<std::endl;
        return 0;
    }

    char in_buf[OUT_BUFSIZE];
    z.next_in = 0;
    z.avail_in = 0;

    for (unsigned int particleIndex=0;particleIndex<(unsigned int )simple->numParticles();particleIndex++) {
        for (unsigned int attrIndex=0;attrIndex<attrs.size();attrIndex++) {
            if (attrs[attrIndex].type==Partio::INT) {
                int* data=simple->dataWrite<int>(attrs[attrIndex],particleIndex);
                for (int count=0;count<attrs[attrIndex].count;count++) {
                    int ival = 0;
                    switch (chans[attrIndex].type) {
                    case 0:	// int16
                        {
                            short val;
                            read_buffer(*input, z, (char*)in_buf, &val, sizeof(short));
                            ival = (int)val;
                        }
                        break;
                    case 1:	// int32
                        {
                            read_buffer(*input, z, (char*)in_buf, &ival, sizeof(int));
                        }
                        break;
                    case 2:	// int64
                        {
                            long long val;
                            read_buffer(*input, z, (char*)in_buf, &val, sizeof(long long));
                            ival = (int)val;
                        }
                        break;
                    case 6:	// uint16
                        {
                            unsigned short val;
                            read_buffer(*input, z, (char*)in_buf, &val, sizeof(unsigned short));
                            ival = (int)val;
                        }
                        break;
                    case 7:	// uint32
                        {
                            unsigned int val;
                            read_buffer(*input, z, (char*)in_buf,  &val, sizeof(unsigned int));
                            ival = (int)val;
                        }
                        break;
                    case 8:	// uint64
                        {
                            unsigned long long val;
                            read_buffer(*input, z, (char*)in_buf, &val, sizeof(unsigned long long));
                            ival = (int)val;
                        }
                        break;
                    case 9:	// int8
                        {
                            char val;
                            read_buffer(*input, z, (char*)in_buf, &val, sizeof(char));
                            ival = (int)val;
                        }
                        break;
                    case 10:// uint8
                        {
                            unsigned char val;
                            read_buffer(*input, z, (char*)in_buf, &val, sizeof(unsigned char));
                            ival = (int)val;
                        }
                        break;
                    }
                    data[count]=ival;
                }
            }else if (attrs[attrIndex].type==Partio::FLOAT || attrs[attrIndex].type==Partio::VECTOR) {
                float* data=simple->dataWrite<float>(attrs[attrIndex],particleIndex);
                for (int count=0;count<attrs[attrIndex].count;count++) {
                    float fval;
                    switch (chans[attrIndex].type) {
                    case 3:	// float16
                        {
#ifdef USE_ILMHALF
                            half val;
                            read_buffer(*input, z, (char*)in_buf, &val, sizeof(half));
                            fval = (float)(val);
#else
                            unsigned short val;
                            read_buffer(*input, z, (char*)in_buf, &val, sizeof(val));
                            fval = half2float[val].f;
#endif
                        }
                        break;
                    case 4:	// float32
                        {
                            read_buffer(*input, z, (char*)in_buf, &fval, sizeof(float));
                        }
                        break;
                    case 5:	// float64
                        {
                            double val;
                            read_buffer(*input, z, (char*)in_buf, &val, sizeof(double));
                            fval = (float)val;
                        }
                        break;
                    }
                    data[count]=fval;
                }
            }
		}
	}
    if (inflateEnd( &z ) != Z_OK) {
        std::cerr<<"Zlib inflateEnd error"<<std::endl;
        return 0;
    }

    // success
    return simple;
}

bool writePRT(const char* filename,const ParticlesData& p,const bool /*compressed*/)
{
	/// Krakatoa pukes on 0 particle files for some reason so don't export at all....
    int numParts = p.numParticles();
    if (numParts)
    {
        std::auto_ptr<std::ostream> output(
        new std::ofstream(filename,std::ios::out|std::ios::binary));

        if (!*output) {
            std::cerr<<"Partio Unable to open file "<<filename<<std::endl;
            return false;
        }

        FileHeadder header;
        memcpy(header.magic, magic, sizeof(magic));
        memcpy(header.signature, signature, sizeof(signature));
        header.headersize = 0x38;
        header.version = 1;
        header.numParticles = p.numParticles();
        int reserve = 4;
        output->write((char*)&header,sizeof(FileHeadder));
        write<LITEND>(*output, reserve);
        write<LITEND>(*output, (int)p.numAttributes());
            reserve = 0x2c;
        write<LITEND>(*output, reserve);

        std::vector<ParticleAttribute> attrs;
        int offset = 0;
        for (int i=0;i<p.numAttributes();i++) {
            ParticleAttribute attr;
            p.attributeInfo(i,attr);
            Channel ch;
            memset(&ch, 0, sizeof(Channel));
                    memcpy((char*)ch.name, attr.name.c_str(), attr.name.size());
            ch.offset = offset;
            switch (attr.type) {
            case FLOAT: ch.type=4; ch.arity=attr.count; offset += sizeof(float)*attr.count; break;
            case INT: ch.type=1; ch.arity=attr.count; offset += sizeof(int)*attr.count; break;
            case VECTOR: ch.type=4; ch.arity=attr.count; offset += sizeof(float)*attr.count; break;
			case INDEXEDSTR:; break;
            case NONE:;break;
            }
            if (ch.arity) {
    #ifdef AUTO_CASES
                if (ch.name[0] >= 'a' && ch.name[0] <= 'z') {
                    ch.name[0] -= 0x20;
                }
    #endif
                output->write((char*)&ch,sizeof(Channel));
                attrs.push_back(attr);
            }
        }

        z_stream z;
        z.zalloc = Z_NULL;z.zfree = Z_NULL;z.opaque = Z_NULL;
        if (deflateInit( &z, Z_DEFAULT_COMPRESSION ) != Z_OK) {
            std::cerr<<"Zlib deflateInit error"<<std::endl;
            return false;
        }

        char out_buf[OUT_BUFSIZE+10];
        for (int particleIndex=0;particleIndex<p.numParticles();particleIndex++) {
            for (unsigned int attrIndex=0;attrIndex<attrs.size();attrIndex++) {
                if (attrs[attrIndex].type==Partio::INT) {
                    const int* data=p.data<int>(attrs[attrIndex],particleIndex);
                    if (!write_buffer(*output, z, (char*)out_buf, (void*)data, sizeof(int)*attrs[attrIndex].count, false))
                        return false;
                } else if (attrs[attrIndex].type==Partio::FLOAT || attrs[attrIndex].type==Partio::VECTOR) {
                    const float* data=p.data<float>(attrs[attrIndex],particleIndex);
                    if (!write_buffer(*output, z, (char*)out_buf, (void*)data, sizeof(int)*attrs[attrIndex].count, false))
                        return false;
                }
            }
        }
        write_buffer(*output, z, (char*)out_buf, 0, 0, true);
        if (deflateEnd( &z ) != Z_OK) {
            std::cerr<<"Zlib deflateEnd error"<<std::endl;
            return false;
        }
        // success
    }// end if numParticles > 0
    return true;
}

}

