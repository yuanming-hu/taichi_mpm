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
Some code for this format  was helped along  by referring to an implementation by  Digital Cinema Arts THANKS!
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

static const long BIN_MAGIC = 0xFABADA;

typedef struct{
    int verificationCode;
    char fluidName[250] ;
    short version;
    float scaleScene;
    int fluidType;
    float elapsedSimulationTime;
    int frameNumber;
    int framePerSecond;
    int numParticles;
    float radius;
    float pressure[3];
    float speed[3];
    float temperature[3];
    float emitterPosition[3];
    float emitterRotation[3];
    float emitterScale[3];
} BIN_HEADER;


ParticlesDataMutable* readBIN(const char* filename, const bool headersOnly){

    auto_ptr<istream> input(new ifstream(filename,ios::in|ios::binary));

    if(!*input){
        cerr << "Partio: Unable to open file " << filename << endl;
        return 0;
    }

    BIN_HEADER header;
    input->read((char*)&header, sizeof(header));

	/*
	// DEBUG HEADER INFO
    cout << "header.verify_code = " << hex << header.verificationCode << endl;
    cout << "header.fluid_name = " << dec << header.fluidName << endl;
    cout << "header.version = " << header.version << endl;
    cout << "header.scene_scale = " << header.scaleScene << endl;
    cout << "header.fluid_type = " << header.fluidType << endl;
    cout << "header.elapsed_time = " << header.elapsedSimulationTime << endl;
    cout << "header.frame_number = " << header.frameNumber << endl;
    cout << "header.fps = " << header.framePerSecond << endl;
    cout << "header.num_particles = " << header.numParticles << endl;
    cout << "header.radius = " << header.radius << endl;
    cout << "header.pressure = " << header.pressure[0] << "\t"
        << header.pressure[1] << "\t"  << header.pressure[2] << endl;
    cout << "header.speed = " << header.speed[0] << "\t"
        << header.speed[1] << "\t" << header.speed[2] << endl;
    cout << "header.temperature = " << header.temperature[0] << "\t"
        << header.temperature[1] << "\t" << header.temperature[2] << endl;
    cout << "header.emitter_pos = " << header.emitterPosition[0] << "\t"
        << header.emitterPosition[1] << "\t" << header.emitterPosition[2] << endl;
    cout << "header.emitter_rot = " << header.emitterRotation[0] << "\t"
    << header.emitterRotation[1] << "\t" << header.emitterRotation[2] << endl;
    cout << "header.emitter_scale = " << header.emitterScale[0] << "\t"
    << header.emitterScale[1] << "\t" << header.emitterScale[2] << endl;
	*/

    if(BIN_MAGIC != header.verificationCode){
        cerr << "Partio: Magic number '" << hex<<  header.verificationCode << "' of '" << filename << "' doesn't match BIN magic '" << BIN_MAGIC << "'" << endl;
        return 0;
    }


    ParticlesDataMutable* simple = headersOnly ? new ParticleHeaders: create();
    simple->addParticles(header.numParticles);

    ParticleAttribute posAttr;
    posAttr = simple->addAttribute("position",  VECTOR, 3);
    ParticleAttribute veloAttr;
    veloAttr = simple->addAttribute("velocity", VECTOR, 3);
    ParticleAttribute forceAttr;
    forceAttr = simple->addAttribute("force", VECTOR, 3);
    ParticleAttribute vortAttr;
    vortAttr = simple->addAttribute("vorticity", VECTOR, 3);
    ParticleAttribute normAttr;
    normAttr = simple->addAttribute("normal", VECTOR, 3);
    ParticleAttribute neighborsAttr;
    neighborsAttr = simple->addAttribute("neighbors", FLOAT, 1);
    ParticleAttribute uvwAttr;
    uvwAttr = simple->addAttribute("uvw", VECTOR, 3);
    ParticleAttribute ageAttr;
    ageAttr = simple->addAttribute("age", FLOAT, 1);
    ParticleAttribute isoTimeAttr;
    isoTimeAttr = simple->addAttribute("isolationTime", FLOAT, 1);
    ParticleAttribute viscosityAttr;
    viscosityAttr = simple->addAttribute("viscosity", FLOAT, 1);
    ParticleAttribute densityAttr;
    densityAttr = simple->addAttribute("density", FLOAT, 1);
    ParticleAttribute pressureAttr;
    pressureAttr = simple->addAttribute("pressure", FLOAT, 1);
    ParticleAttribute massAttr;
    massAttr = simple->addAttribute("mass", FLOAT, 1);
    ParticleAttribute tempAttr;
    tempAttr = simple->addAttribute("temperature", FLOAT, 1);
    ParticleAttribute pidAttr;
    pidAttr = simple->addAttribute("id", INT, 1);

    // if headersOnly, skip
    //if(headersOnly)
    //{
     //   input->seekg((int)input->tellg() + header.numParticles*sizeof(double)*attr.count);
    //    continue;
    //}
    //else
    //{
        for(int partIndex = 0; partIndex < simple->numParticles(); partIndex++)
        {

            float position[3] = {0.0,0.0,0.0};
            float velocity[3] = {0.0,0.0,0.0};
            float force[3] = {0.0,0.0,0.0};
            float vorticity[3] = {0.0,0.0,0.0};
            float normal[3] = {0.0,0.0,0.0};
            int neighbors = 0;
            float uvw[3] = {0.0,0.0,0.0};
            short infoBits = 7;
            float age = 0.0;
            float isolationTime = 1.0;
            float viscosity = 1.0;
            float density = 1.0;
            float pressure = 1.0;
            float mass = 1.0;
            float temperature = 1.0;
            int pid = 0;

            input->read ((char *) &position[0], sizeof(float));
                simple->dataWrite<float>(posAttr, partIndex)[0] = (float)position[0];
            input->read ((char *) &position[1], sizeof(float));
                simple->dataWrite<float>(posAttr, partIndex)[1] = (float)position[1];
            input->read ((char *) &position[2], sizeof(float));
                simple->dataWrite<float>(posAttr, partIndex)[2] = (float)position[2];


            input->read ((char *) &velocity[0], sizeof(float));
                simple->dataWrite<float>(veloAttr, partIndex)[0] = (float)velocity[0];
            input->read ((char *) &velocity[1], sizeof(float));
                simple->dataWrite<float>(veloAttr, partIndex)[1] = (float)velocity[1];
            input->read ((char *) &velocity[2], sizeof(float));
                simple->dataWrite<float>(veloAttr, partIndex)[2] = (float)velocity[2];

            input->read ((char *) &force[0], sizeof(float));
                simple->dataWrite<float>(forceAttr, partIndex)[0] = (float)force[0];
            input->read ((char *) &force[1], sizeof(float));
                simple->dataWrite<float>(forceAttr, partIndex)[1] = (float)force[1];
            input->read ((char *) &force[2], sizeof(float));
                simple->dataWrite<float>(forceAttr, partIndex)[2] = (float)force[2];

            input->read ((char *) &vorticity[0], sizeof(float));
                simple->dataWrite<float>(vortAttr, partIndex)[0] = (float)vorticity[0];
            input->read ((char *) &vorticity[1], sizeof(float));
                simple->dataWrite<float>(vortAttr, partIndex)[1] = (float)vorticity[1];
            input->read ((char *) &vorticity[2], sizeof(float));
                simple->dataWrite<float>(vortAttr, partIndex)[2] = (float)vorticity[2];

            input->read ((char *) &normal[0], sizeof(float));
                simple->dataWrite<float>(normAttr, partIndex)[0] = (float)normal[0];
            input->read ((char *) &normal[1], sizeof(float));
                simple->dataWrite<float>(normAttr, partIndex)[1] = (float)normal[1];
            input->read ((char *) &normal[2], sizeof(float));
                simple->dataWrite<float>(normAttr, partIndex)[2] = (float)normal[2];


            input->read ((char *) &neighbors, sizeof (int));
                simple->dataWrite<float>(neighborsAttr, partIndex)[0] = (float)neighbors;

            input->read ((char *) &uvw[0], sizeof(float));
                simple->dataWrite<float>(uvwAttr, partIndex)[0] = (float)position[0];
            input->read ((char *) &uvw[1], sizeof(float));
                simple->dataWrite<float>(uvwAttr, partIndex)[1] = (float)position[1];
            input->read ((char *) &uvw[2], sizeof(float));
                simple->dataWrite<float>(uvwAttr, partIndex)[2] = (float)position[2];

            input->read ((char *) &infoBits, sizeof(infoBits));
            // don't  do anything with this..
            input->read ((char *) &age, sizeof(age));
                simple->dataWrite<float>(ageAttr, partIndex)[0] = (float)age;
            input->read ((char *) &isolationTime, sizeof(isolationTime));
                simple->dataWrite<float>(isoTimeAttr, partIndex)[0] = (float)isolationTime;
            input->read ((char *) &viscosity, sizeof(viscosity));
                simple->dataWrite<float>(viscosityAttr, partIndex)[0] = (float)viscosity;
            input->read ((char *) &density, sizeof(density));
                simple->dataWrite<float>(densityAttr, partIndex)[0] = (float)density;
            input->read ((char *) &pressure, sizeof(pressure));
                simple->dataWrite<float>(pressureAttr, partIndex)[0] = (float)pressure;
            input->read ((char *) &mass, sizeof(mass));
                simple->dataWrite<float>(massAttr, partIndex)[0] = (float)mass;
            input->read ((char *) &temperature, sizeof(temperature));
                simple->dataWrite<float>(tempAttr, partIndex)[0] = (float)temperature;
            input->read ((char *) &pid, sizeof(pid));
                simple->dataWrite<int>(pidAttr, partIndex)[0] = (int)pid;


        //}



    }

    return simple;
}

bool writeBIN(const char* filename,const ParticlesData& p,const bool /*compressed*/)
{

    auto_ptr<ostream> output(
    new ofstream(filename,ios::out|ios::binary));

    if (!*output) {
        cerr<<"Partio Unable to open file "<<filename<<endl;
        return false;
    }

    BIN_HEADER  header;

    header.verificationCode =  BIN_MAGIC;
    for(int i = 0; i < 250; i++)
    {header.fluidName[i] = 0;}
    string str = "partioExport_01";
    str.copy(header.fluidName,15,0);  //  fluid name
    header.framePerSecond = 24; // frames per second
    header.scaleScene = 1.0; // scene scale
    header.fluidType = 9; // fluid type
    header.version =  11; // version (11 is most current)
    header.frameNumber =  1; // frame number
    header.elapsedSimulationTime = 0.0416666; //   time elapsed (in seconds)
    header.numParticles = p.numParticles(); // number of particles
    header.radius = 0.1; // radius of emitter
    header.pressure[0] = 1.0; // max, min, and avg pressure
    header.pressure[1] = 1.0;
    header.pressure[2] = 1.0;
    header.speed[0] = 3.0;
    header.speed[1] = 2.0;
    header.speed[2] = 1.0; // max, min, and avg speed
    header.temperature[0] = 3.0;
    header.temperature[1] = 2.0;
    header.temperature[2] = 1.0; // max, min, and avg temperature
    header.emitterPosition[0] = 0.0;
    header.emitterPosition[1] = 0.0;
    header.emitterPosition[2] = 0.0; //emitter position
    header.emitterRotation[0] = 0.0;
    header.emitterRotation[1] = 0.0;
    header.emitterRotation[2] = 0.0; //emitter position
    header.emitterScale[0] = 0.0;
    header.emitterScale[1] = 0.0;
    header.emitterScale[2] = 0.0; //emitter position



    // write .bin header
    output->write ((const char *) &header.verificationCode, sizeof (int));
    output->write ((const char *) &header.fluidName, 250);
    output->write ((const char *) &header.version, sizeof (short int));
    output->write ((const char *) &header.scaleScene, sizeof (float));
    output->write ((const char *) &header.fluidType, sizeof (int));
    output->write ((const char *) &header.elapsedSimulationTime, sizeof (float));
    output->write ((const char *) &header.frameNumber, sizeof (int));
    output->write ((const char *) &header.framePerSecond, sizeof (int));
    output->write ((const char *) &header.numParticles, sizeof (int));
    output->write ((const char *) &header.radius, sizeof (float));

    for(int i=0; i <=2; i++)
                output->write ((const char *) &header.pressure[i], sizeof(float));
    for(int i=0; i<= 2; i++)
                output->write ((const char *) &header.speed[i], sizeof(float));
    for(int i=0; i<= 2; i++)
                output->write ((const char *) &header.temperature[i], sizeof(float));

    for(int i=0; i<= 2; i++)
                output->write ((const char *) &header.emitterPosition[i], sizeof(float));
    for(int i=0; i <= 2; i++)
                output->write ((const char *) &header.emitterRotation[i], sizeof(float));
    for(int i=0; i <= 2; i++)
                output->write ((const char *) &header.emitterScale[i], sizeof(float));

    for (int particles = 0; particles < p.numParticles(); particles++)
    {
        cout <<  particles << endl;
        // set defaults for stuff that is not exported...
        float position[3] = {0.0,0.0,0.0};
        float velocity[3] = {0.0,0.0,0.0};
        float force[3] = {0.0,0.0,0.0};
        float vorticity[3] = {0.0,0.0,0.0};
        float normal[3] = {0.0,0.0,0.0};
        int neighbors = 0;
        float uvw[3] = {0.0,0.0,0.0};
        short infoBits = 7;
        float age = 0.0;
        float isolationTime = 1.0;
        float viscosity = 1.0;
        float density = 1.0;
        float pressure = 1.0;
        float mass = 1.0;
        float temperature = 1.0;
        int pid = particles;

        // now run thru  the exported attrs  and  replace values for things that we do have

        for(int attrIndex = 0; attrIndex < p.numAttributes(); attrIndex++)
        {
            ParticleAttribute attr;
            p.attributeInfo(attrIndex,attr);

            cout << attr.name << endl;
            if (attr.name ==  "position")
            {
                const float* data = p.data<float>(attr, particles);
                position[0] = (double)data[0];
                position[1] = (double)data[1];
                position[2] = (double)data[2];
            }

            else if (attr.name == "velocity")
            {
                const float* data = p.data<float>(attr, particles);
                velocity[0] = (double)data[0];
                velocity[1] = (double)data[1];
                velocity[2] = (double)data[2];
            }
            else if (attr.name == "force")
            {
                const float* data = p.data<float>(attr, particles);
                force[0] = (double)data[0];
                force[1] = (double)data[1];
                force[2] = (double)data[2];
            }
            else if (attr.name == "vorticity")
            {
                const float* data = p.data<float>(attr, particles);
                vorticity[0] = (double)data[0];
                vorticity[1] = (double)data[1];
                vorticity[2] = (double)data[2];
            }
            else if (attr.name == "normal")
            {
                const float* data = p.data<float>(attr, particles);
                normal[0] = (double)data[0];
                normal[1] = (double)data[1];
                normal[2] = (double)data[2];
            }
            else if (attr.name == "neighbors")
            {
                const float* data = p.data<float>(attr, particles);
                neighbors= (int)data[0];
            }
            else if (attr.name == "uvw")
            {
                const float* data = p.data<float>(attr, particles);
                uvw[0] = (double)data[0];
                uvw[1] = (double)data[1];
                uvw[2] = (double)data[2];
            }
            else if (attr.name == "age")
            {
                const float* data = p.data<float>(attr, particles);
                age= (double)data[0];
            }
            else if (attr.name == "isolationTime")
            {
                const float* data = p.data<float>(attr, particles);
                isolationTime= (double)data[0];
            }
            else if (attr.name == "viscosity")
            {
                const float* data = p.data<float>(attr, particles);
                viscosity= (double)data[0];
            }
            else if (attr.name == "density")
            {
                const float* data = p.data<float>(attr, particles);
                density= (double)data[0];
            }
            else if (attr.name == "pressure")
            {
                const float* data = p.data<float>(attr, particles);
                pressure= (double)data[0];
            }
            else if (attr.name == "mass")
            {
                const float* data = p.data<float>(attr, particles);
                mass= (double)data[0];
            }
            else if (attr.name == "temperature")
            {
                const float* data = p.data<float>(attr, particles);
                temperature= (double)data[0];
            }
            else if (attr.name == "id")
            {
                const float* data = p.data<float>(attr, particles);
                pid= (int)data[0];
            }

            else
            {
                cout << "Attribute found that  we don't support yet" << endl;
            }
        }

        output->write ((const char *) &position[0], sizeof (float));
        output->write ((const char *) &position[1], sizeof (float));
        output->write ((const char *) &position[2], sizeof (float));

        output->write ((const char *) &velocity[0], sizeof (float));
        output->write ((const char *) &velocity[1], sizeof (float));
        output->write ((const char *) &velocity[2], sizeof (float));

        output->write ((const char *) &force[0], sizeof (float));
        output->write ((const char *) &force[1], sizeof (float));
        output->write ((const char *) &force[2], sizeof (float));

        output->write ((const char *) &vorticity[0], sizeof (float));
        output->write ((const char *) &vorticity[1], sizeof (float));
        output->write ((const char *) &vorticity[2], sizeof (float));

        output->write ((const char *) &normal[0], sizeof (float));
        output->write ((const char *) &normal[1], sizeof (float));
        output->write ((const char *) &normal[2], sizeof (float));

        output->write ((const char *) &neighbors, sizeof (neighbors));

        output->write ((const char *) &uvw[0], sizeof (float));
        output->write ((const char *) &uvw[1], sizeof (float));
        output->write ((const char *) &uvw[2], sizeof (float));

        output->write ((const char *) &infoBits, sizeof (infoBits));
        output->write ((const char *) &age, sizeof (age));
        output->write ((const char *) &isolationTime, sizeof (isolationTime));
        output->write ((const char *) &viscosity, sizeof (viscosity));
        output->write ((const char *) &density, sizeof (density));
        output->write ((const char *) &pressure, sizeof (pressure));
        output->write ((const char *) &mass, sizeof (mass));
        output->write ((const char *) &temperature, sizeof (temperature));
        output->write ((const char *) &pid, sizeof (pid));

    }

    // future proofing, TODO: fully support additional  PP attributes

    return true;

}

}// end of namespace Partio
