CXX=g++
CXXFLAGS=-g -I/opt/local/include  -fPIC
LINKFLAGS=
OBJS=ParticleSimple.o Particle.o DEBUG_PRINT.o  \
	reader/PDB.o reader/GEO.o reader/BGEO.o



all: lib partinfo test
lib: ${OBJS}
	${CXX} ${LINKFLAGS} -shared -o libpapi.so ${OBJS}
partinfo:  lib partinfo.cpp
	${CXX} ${LINKFLAGS} -o partinfo libpapi.so partinfo.cpp
test: lib test.cpp
	${CXX} ${LINKFLAGS} -o test libpapi.so test.cpp

ParticleSimple.o: ParticleSimple.cpp ParticleSimple.h Particle.h
Particle.o: Particle.cpp Particle.h 
test.o: test.cpp ParticleSimple.h Particle.h
DEBUG_PRINT.o: DEBUG_PRINT.h DEBUG_PRINT.cpp
reader/GEO.o: reader/GEO.cpp Particle.h ParticleSimple.h \
  reader/../Particle.h
reader/BGEO.o: reader/BGEO.cpp Particle.h ParticleSimple.h \
  reader/../Particle.h reader/endian.h
reader/PDB.o: reader/PDB.cpp Particle.h ParticleSimple.h \
  reader/../Particle.h reader/pdb.h
partinfo.o: partinfo.cpp Particle.h
partconv.o: partconf.cpp Particle.h