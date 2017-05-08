Qhull can be called as an external application, but the documentation tells that this is wasteful for multiple calls.
Qhull has an experimental C++ interface that does not cover all functionality of Qhull,
for example it only has a method runQhull but misses runQhull3D to construct 3 dimensional convex hulls.
However the missing method can be added to the interface as described on stackoverflow (see below).
This method has beed added to Qhull.h and Qhull.cpp.



References:
http://stackoverflow.com/questions/19530731/qhull-library-c-interface

http://www.qhull.org/html/qh-code.htm#reentrant

Example of calling qhull from C++
https://github.com/qhull/qhull/blob/master/src/user_eg3/user_eg3_r.cpp

