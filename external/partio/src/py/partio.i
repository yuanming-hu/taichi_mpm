/**
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


%module partio
%include "std_string.i"


%{
#include <Partio.h>
namespace Partio{
typedef uint64_t ParticleIndex;
}

using namespace Partio;


// This is for passing fixed sized arrays
struct fixedFloatArray
{
    int count;
    Partio::ParticleAttributeType type; // should be float or VECTOR
    union{
        float f[16];
        int i[16];
    };
};

%}

// Particle Types
enum ParticleAttributeType {NONE=0,VECTOR=1,FLOAT=2,INT=3,INDEXEDSTR=4};


%feature("docstring","A handle for operating on attribbutes of a particle set");
class ParticleAttribute
{
public:
    %feature("docstring","Type of the particle data (VECTOR,INT,FLOAT)");
    ParticleAttributeType type;

    %feature("docstring","Number of primitives (int's or float's)");
    int count;

    %feature("docstring","Attribute name");
    std::string name;

    // internal use
    //int attributeIndex; 
};


%typemap(in) uint64_t {
	$1 = (uint64_t) PyLong_AsLong($input);
}
%typemap(out) uint64_t {
	$result = PyLong_FromLong(
            (long)$1);
}
%apply uint64_t { ParticleIndex };

%typemap(in) fixedFloatArray
{
    int i;
    if(!PySequence_Check($input)){
        PyErr_SetString(PyExc_TypeError,"Expecting a sequence");
        return NULL;
    }
    int size=PyObject_Length($input);
    $1.count=size;
    for(i=0;i<size;i++){
        PyObject* o=PySequence_GetItem($input,i);
        if(!PyFloat_Check(o)){
            Py_XDECREF(o);
            PyErr_SetString(PyExc_ValueError,"Expecting a sequence of floats");
            return NULL;
        }
        $1.f[i]=PyFloat_AsDouble(o);
        Py_DECREF(o);
    } 
}

%typemap(out) fixedFloatArray
{
    PyObject* tuple=PyTuple_New($1.count);
    for(int k=0;k<$1.count;k++)
        PyTuple_SetItem(tuple,k,PyFloat_FromDouble($1.f[k]));
    $result=tuple;
}

%typemap(freearg) float* floatTupleIn
{
    delete [] $1;
}

%unrefobject ParticlesInfo "$this->release();"
%feature("autodoc");
%feature("docstring","A set of particles with associated data attributes.");
class ParticlesInfo
{
public:
    %feature("autodoc");
    %feature("docstring","Returns the number of particles in the set");
    virtual int numParticles() const=0;

    %feature("autodoc");
    %feature("docstring","Returns the number of particles in the set");
    virtual int numAttributes() const=0;
};



%unrefobject ParticlesData "$this->release();"
%feature("autodoc");
%feature("docstring","A reader for a set of particles.");
class ParticlesData:public ParticlesInfo
{
  public:
    %feature("autodoc");
    %feature("docstring","Looks up a given indexed string given the index, returns -1 if not found");
    int lookupIndexedStr(const ParticleAttribute& attribute,const char* str) const=0;

};

%unrefobject ParticlesDataMutable "$this->release();"
%feature("autodoc");
%feature("docstring","A writer for a set of particles.");
class ParticlesDataMutable:public ParticlesData
{
public:

    %feature("autodoc");
    %feature("docstring","Registers a string in the particular attribute");
    virtual int registerIndexedStr(const ParticleAttribute& attribute,const char* str)=0;


    %feature("autodoc");
    %feature("docstring","Prepares data for N nearest neighbor searches using the\n"
       "attribute in the file with name 'position'");
    virtual void sort()=0;

    %feature("autodoc");
    %feature("docstring","Adds a new attribute of given name, type and count. If type is\n"
        "partio.VECTOR, then count must be 3");
    virtual ParticleAttribute addAttribute(const char* attribute,ParticleAttributeType type,
        const int count)=0;

    %feature("autodoc");
    %feature("docstring","Adds a new particle and returns the index");
    virtual ParticleIndex addParticle()=0;

    %feature("autodoc");
    %feature("docstring","Adds count particles and returns the offset to the first one");
    virtual ParticleIterator<false> addParticles(const int count)=0;
};



%extend ParticlesData {
    %feature("autodoc");
    %feature("docstring","Searches for the N nearest points to the center location\n"
        "or as many as can be found within maxRadius distance.");
    PyObject* findNPoints(fixedFloatArray center,int nPoints,float maxRadius)
    {
        // make sure we are close
        if(center.count!=3){
            fprintf(stderr,"Need center to be a 3 tuple of floats\n");
            return NULL;
        }
        // find nearest neighbors
        std::vector<ParticleIndex> points;
        std::vector<float> pointDistancesSquared;
        $self->findNPoints(center.f,nPoints,maxRadius,points,pointDistancesSquared);
        
        // build the python return type
        PyObject* list=PyList_New(points.size());
        for(unsigned int i=0;i<points.size();i++){
            PyObject* tuple=Py_BuildValue("(if)",points[i],pointDistancesSquared[i]);
            PyList_SetItem(list,i,tuple); // tuple reference is stolen, so no decref needed
        }
        return list;
    }

    %feature("autodoc");
    %feature("docstring","Returns the indices of all points within the bounding\n"
        "box defined by the two cube corners bboxMin and bboxMax");
    PyObject* findPoints(fixedFloatArray bboxMin,fixedFloatArray bboxMax)
    {    
        // make sure we are close
        if(bboxMin.count!=3 || bboxMax.count!=3){
            fprintf(stderr,"Need bboxMin and bboxMax to be a 3 tuple of floats\n");
            return NULL;
        }
        // find nearest neighbors
        std::vector<ParticleIndex> points;
        $self->findPoints(bboxMin.f,bboxMax.f,points);
        
        // build the python return type
        PyObject* list=PyList_New(points.size());
        for(unsigned int i=0;i<points.size();i++){
            PyList_SetItem(list,i,PyInt_FromLong(points[i])); // tuple reference is stolen, so no decref needed
        }
        return list;
    }

    %feature("autodoc");
    %feature("docstring","Gets attribute data for particleIndex'th particle");
    PyObject* get(const ParticleAttribute& attr,const ParticleIndex particleIndex)
    {
        PyObject* tuple=PyTuple_New(attr.count);
        if(attr.type==Partio::INT){
            const int* p=$self->data<int>(attr,particleIndex);
            for(int k=0;k<attr.count;k++) PyTuple_SetItem(tuple,k,PyInt_FromLong(p[k]));
        }else{
            const float* p=$self->data<float>(attr,particleIndex);
            for(int k=0;k<attr.count;k++) PyTuple_SetItem(tuple,k,PyFloat_FromDouble(p[k]));
        }
        return tuple;
    }


    %feature("autodoc");
    %feature("docstring","Gets a list of all indexed strings for the given attribute handle");
    PyObject* indexedStrs(const ParticleAttribute& attr) const
    {
        const std::vector<std::string>& indexes=self->indexedStrs(attr);
        PyObject* list=PyList_New(indexes.size());
        for(int k=0;k<indexes.size();k++) PyList_SetItem(list,k,PyString_FromString(indexes[k].c_str()));
        return list;
    }
}

%extend ParticlesDataMutable {

    %feature("autodoc");
    %feature("docstring","Sets data on a given attribute for a single particle.\n"
        "Data must be specified as tuple.");
    PyObject* set(const ParticleAttribute& attr,const uint64_t particleIndex,PyObject* tuple)
    {
        if(!PySequence_Check(tuple)){
            PyErr_SetString(PyExc_TypeError,"Expecting a sequence");
            return NULL;
        }
        int size=PyObject_Length(tuple);
        if(size!=attr.count){
            PyErr_SetString(PyExc_ValueError,"Invalid number of parameters");
            return NULL;
        }
        
        if(attr.type==Partio::INT){
            int* p=$self->dataWrite<int>(attr,particleIndex);
            for(int i=0;i<size;i++){
                PyObject* o=PySequence_GetItem(tuple,i);
                if(PyInt_Check(o)){
                    p[i]=PyInt_AsLong(o);
                }else{
                    Py_XDECREF(o);
                    PyErr_SetString(PyExc_ValueError,"Expecting a sequence of ints");
                    return NULL;
                }
                Py_XDECREF(o);
            }
        }else{
            float* p=$self->dataWrite<float>(attr,particleIndex);
            for(int i=0;i<size;i++){
                PyObject* o=PySequence_GetItem(tuple,i);
                //fprintf(stderr,"checking %d\n",i);
                if(PyFloat_Check(o)){
                    p[i]=PyFloat_AsDouble(o);
                }else if(PyInt_Check(o)){
                    p[i]=(float)PyInt_AsLong(o);
                }else{
                    Py_XDECREF(o);
                    PyErr_SetString(PyExc_ValueError,"Expecting a sequence of floats or ints");
                    return NULL;
                }
                Py_XDECREF(o);
            }
        }

        Py_INCREF(Py_None);
        return Py_None;
    }


}

%extend ParticlesInfo {

    %feature("autodoc");
    %feature("docstring","Searches for and returns the attribute handle for a named attribute");
    %newobject attributeInfo;
    ParticleAttribute* attributeInfo(const char* name)
    {
        ParticleAttribute a;
        bool valid=$self->attributeInfo(name,a);
        if(valid) return new ParticleAttribute(a);
        else return 0;
    }

    %feature("autodoc");
    %feature("docstring","Returns the attribute handle by index");
    %newobject attributeInfo;
    ParticleAttribute* attributeInfo(const int index)
    {
        if(index<0 || index>=$self->numAttributes()){
            PyErr_SetString(PyExc_IndexError,"Invalid attribute index");
            return NULL;
        }
        ParticleAttribute a;
        bool valid=$self->attributeInfo(index,a);
        if(valid) return new ParticleAttribute(a);
        else return 0;
    }
}

%feature("autodoc");
%feature("docstring","Create an empty particle array");
%newobject create;
ParticlesDataMutable* create();

%feature("autodoc");
%feature("docstring","Reads a particle set from disk");
%newobject read;
ParticlesDataMutable* read(const char* filename);

%feature("autodoc");
%feature("docstring","Reads a particle set headers from disk");
%newobject readHeaders;
ParticlesInfo* readHeaders(const char* filename);

%feature("autodoc");
%feature("docstring","Writes a particle set to disk");
void write(const char* filename,const ParticlesData&,const bool=false);

%feature("autodoc");
%feature("docstring","Print a summary of particle file");
void print(const ParticlesData* particles);
