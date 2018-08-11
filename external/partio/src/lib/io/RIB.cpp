/*
* Copyright (c) 2010 - 2011 Sebastien Noury <sebastien@soja.org>
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*
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

bool writeRIB(const char* filename, const ParticlesData& p, const bool compressed)
{
    auto_ptr<ostream> output(
        compressed ? Gzip_Out(filename, ios::out | ios::binary)
        : new ofstream(filename, ios::out | ios::binary));

    ParticleAttribute dummy;
    bool foundP     = p.attributeInfo("position",  dummy) || p.attributeInfo("P",     dummy);
    bool foundP2    = p.attributeInfo("position2", dummy) || p.attributeInfo("P2",    dummy);
    bool foundWidth = p.attributeInfo("radius",    dummy) || p.attributeInfo("width", dummy) || p.attributeInfo("radiusPP", dummy);

    if (!foundP)
    {
        cerr << "Partio: failed to find attr 'position' or 'P' for RIB output" << endl;
        return false;
    }

    if (!foundWidth)
        cerr << "Partio: failed to find attr 'width','radius', or 'radiusPP' for RIB output, using constantwidth = 1" << endl;

    *output << "version 3.04" << endl;

    if (foundP2)
        *output << "GeometricApproximation \"motionfactor\" 1.0" << endl;

    *output << "AttributeBegin" << endl;
    *output << "  ResourceBegin" << endl;
    *output << "    Attribute \"identifier\" \"name\" [\"|partioParticle1|partioParticleShape1\"]" << endl;

    int numPasses = foundP2 ? 2 : 1;

    if (foundP2)
        *output << "    MotionBegin [0.0 1.0]" << endl;

    for (int passIndex = 0; passIndex < numPasses; ++passIndex)
    {
        *output << (foundP2 ? "  " : "") << "    Points ";

        for (int attrIndex = 0; attrIndex < p.numAttributes(); ++attrIndex)
        {
            ParticleAttribute attr;
            p.attributeInfo(attrIndex, attr);

            if ((passIndex == 0 && (attr.name == "P2" || attr.name == "position2")) ||
                    (passIndex == 1 && (attr.name == "P"  || attr.name == "position"))) continue;

            string attrname = (attr.name == "position" || attr.name == "position2" || attr.name == "P" || attr.name == "P2")
                              ? "P"
                              : (attr.name == "radius" ? "width" : attr.name);

            *output << "\"" << attrname << "\" [ ";
			int spaceCount = 0;
            switch (attr.type)
            {
            case Partio::FLOAT:
            case Partio::VECTOR:
			{
                for (int particleIndex = 0; particleIndex < p.numParticles(); ++particleIndex)
                {
                    const float *data = p.data<float>(attr, particleIndex);
                    for (int count = 0; count < attr.count; ++count)
					{
                        *output << data[count] << " ";
						spaceCount ++;
					}
                    if (spaceCount > 12)
                    {
                        *output << "\n    ";
                        spaceCount = 0;
                    }
                }
                break;
			}
            case Partio::INT:
			{
                for (int particleIndex = 0; particleIndex < p.numParticles(); ++particleIndex)
                {
                    const int *data = p.data<int>(attr, particleIndex);
                    for (int count = 0; count < attr.count; ++count)
					{
                        *output << data[count] << " ";
						spaceCount ++;
					}
					if (spaceCount > 12)
                    {
                        *output << "\n    ";
                        spaceCount = 0;
                    }
                }
                break;
			}
            case Partio::NONE:
            default:
                break;
            }

            *output << "]\n    ";
        }

        if (!foundWidth)
            *output << "\"constantwidth\" [1.0]";

        *output << endl;
    }

    if (foundP2)
        *output << "    MotionEnd" << endl;

    *output << "  ResourceEnd" << endl;
    *output << "AttributeEnd" << endl;

    return true;
}
}
