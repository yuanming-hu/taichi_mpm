Example of Usage
=================

1. GJK - Intersection Test
---------------------------
This section describes how to use **libccd** for testing if two convex objects intersects (i.e., 'yes/no' test) using Gilbert-Johnson-Keerthi (GJK) algorithm.

Procedure is very simple (and similar to the usage of the rest of the
library):

#. Include ``<ccd/ccd.h>`` file.
#. Implement support function for specific shapes. Support function is
   function that returns furthest point from object (shape) in specified
   direction.
#. Set up ``ccd_t`` structure.
#. Run ``ccdGJKIntersect()`` function on desired objects.


Here is a skeleton of simple program:

.. code-block:: c

    #include <ccd/ccd.h>
    #include <ccd/quat.h> // for work with quaternions

    /** Support function for box */
    void support(const void *obj, const ccd_vec3_t *dir,
                 ccd_vec3_t *vec)
    {
        // assume that obj_t is user-defined structure that holds info about
        // object (in this case box: x, y, z, pos, quat - dimensions of box,
        // position and rotation)
        obj_t *obj = (obj_t *)_obj;
        ccd_vec3_t dir;
        ccd_quat_t qinv;

        // apply rotation on direction vector
        ccdVec3Copy(&dir, _dir);
        ccdQuatInvert2(&qinv, &obj->quat);
        ccdQuatRotVec(&dir, &qinv);

        // compute support point in specified direction
        ccdVec3Set(v, ccdSign(ccdVec3X(&dir)) * box->x * CCD_REAL(0.5),
                      ccdSign(ccdVec3Y(&dir)) * box->y * CCD_REAL(0.5),
                      ccdSign(ccdVec3Z(&dir)) * box->z * CCD_REAL(0.5));

        // transform support point according to position and rotation of object
        ccdQuatRotVec(v, &obj->quat);
        ccdVec3Add(v, &obj->pos);
    }


    int main(int argc, char *argv[])
    {
        ...

        ccd_t ccd;
        CCD_INIT(&ccd); // initialize ccd_t struct
    
        // set up ccd_t struct
        ccd.support1       = support; // support function for first object
        ccd.support2       = support; // support function for second object
        ccd.max_iterations = 100;     // maximal number of iterations

        int intersect = ccdGJKIntersect(obj1, obj2, &ccd);
        // now intersect holds true if obj1 and obj2 intersect, false otherwise
    }




2. GJK + EPA - Penetration Of Two Objects
------------------------------------------

If you want to obtain also penetration info about two intersection objects
``ccdGJKPenetration()`` function can be used. 

Procedure is almost the same as for the previous case:

.. code-block:: c

    #include <ccd/ccd.h>
    #include <ccd/quat.h> // for work with quaternions

    /** Support function is same as in previous case */

    int main(int argc, char *argv[])
    {
        ...
        ccd_t ccd;
        CCD_INIT(&ccd); // initialize ccd_t struct

        // set up ccd_t struct
        ccd.support1       = support; // support function for first object
        ccd.support2       = support; // support function for second object
        ccd.max_iterations = 100;     // maximal number of iterations
        ccd.epa_tolerance  = 0.0001;  // maximal tolerance fro EPA part

        ccd_real_t depth;
        ccd_vec3_t dir, pos;
        int intersect = ccdGJKPenetration(obj1, obj2, &ccd, &depth, &dir, &pos);
        // now intersect holds true if obj1 and obj2 intersect, false otherwise
        // in depth, dir and pos is stored penetration depth, direction of
        // separation vector and position in global coordinate system
    }


3. MPR - Intersection Test
---------------------------

**libccd** also provides *MPR* - Minkowski Portal Refinement algorithm that
can be used for testing if two objects intersects.

Procedure is similar to the one used for GJK algorithm. Support function is
the same but also function that returns a center (or any point near center)
of a given object must be implemented:

.. code-block:: c

    #include <ccd/ccd.h>
    #include <ccd/quat.h> // for work with quaternions

    /** Support function is same as in previous case */

    /** Center function - returns center of object */
    void center(const void *_obj, ccd_vec3_t *center)
    {
        obj_t *obj = (obj_t *)_obj;
        ccdVec3Copy(center, &obj->pos);
    }

    int main(int argc, char *argv[])
    {
        ...
        ccd_t ccd;
        CCD_INIT(&ccd); // initialize ccd_t struct

        // set up ccd_t struct
        ccd.support1       = support; // support function for first object
        ccd.support2       = support; // support function for second object
        ccd.center1        = center;  // center function for first object
        ccd.center2        = center;  // center function for second object
        ccd.mpr_tolerance  = 0.0001;  // maximal tolerance

        int intersect = ccdMPRIntersect(obj1, obj2, &ccd);
        // now intersect holds true if obj1 and obj2 intersect, false otherwise
    }


4. MPR - Penetration Of Two Objects
------------------------------------

Using MPR algorithm for obtaining penetration info about two intersection
objects is equally easy as in the previous case instead but
``ccdMPRPenetration()`` function is used:

.. code-block:: c

    #include <ccd/ccd.h>
    #include <ccd/quat.h> // for work with quaternions

    /** Support function is same as in previous case */
    /** Center function is same as in prevous case */

    int main(int argc, char *argv[])
    {
        ...
        ccd_t ccd;
        CCD_INIT(&ccd); // initialize ccd_t struct

        // set up ccd_t struct
        ccd.support1       = support; // support function for first object
        ccd.support2       = support; // support function for second object
        ccd.center1        = center;  // center function for first object
        ccd.center2        = center;  // center function for second object
        ccd.mpr_tolerance  = 0.0001;  // maximal tolerance

        ccd_real_t depth;
        ccd_vec3_t dir, pos;
        int intersect = ccdMPRPenetration(obj1, obj2, &ccd, &depth, &dir, &pos);
        // now intersect holds true if obj1 and obj2 intersect, false otherwise
        // in depth, dir and pos is stored penetration depth, direction of
        // separation vector and position in global coordinate system
    }

