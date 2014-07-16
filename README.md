**ik - Jonathan's Inverse Kinematics Library**
---------
---------

**Overview**
--------

This library is a bunch of stuff ripped out of my private library (jtil) for the use of doing gradient-descent based inverse kinematics.  It is VERY bare-bones, but provides a rough starting point for doing custom IK (using non-linear unconstrained optimization).  I put it together for use internally in our research group, but is up here in case anyone finds any of it useful.

**Compilation**
---------------

Building ik uses Visual Studio 2012 on Windows and there are no dependencies for the main library.   If you want to run the example you will need freeglut, however the pre-compiled x64 binaries are in this repo (so the test project should just build and run).  The code is also cross platform (and compiles using gcc on Mac OS X), but I have not included any CMake files because I got sick of supporting them.

**Running**
---------------

If you build the test project (in ik.sln) then a simple glut window will show an animated 2-link joint.  For your own applications I would start from here.

**Style**
---------

This project follows the Google C++ style conventions: 

<http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml>
