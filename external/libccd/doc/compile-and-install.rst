Compile And Install
====================

libccd contains several mechanisms how to compile and install it.
Using a simple Makefile, using autotools and using CMake.


1. Using Makefile
------------------
Directory ``src/`` contains Makefile that should contain everything needed for compilation and installation:

.. code-block:: bash

    $ cd src/
    $ make
    $ make install

Library libccd is by default compiled in double precision of floating point
numbers - you can change this by options ``USE_SINGLE``/``USE_DOUBLE``, i.e.:

.. code-block:: bash

    $ make USE_SINGLE=yes

will compile library in single precision.
Installation directory can be changed by options ``PREFIX``, ``INCLUDEDIR``
and ``LIBDIR``. 
For more info type '``make help``'.


2. Using Autotools
-------------------
libccd also contains support for autotools:
Generate configure script etc.:

.. code-block:: bash

    $ ./bootstrap

Create new ``build/`` directory:

.. code-block:: bash

    $ mkdir build && cd build

Run configure script:

.. code-block:: bash

    $ ../configure

Run make and make install:

.. code-block:: bash

    $ make && make install

configure script can change the way libccd is compiled and installed, most
significant option is ``--enable-double-precision`` which enables double
precision (single is default in this case).

3. Using CMake
---------------
TODO
