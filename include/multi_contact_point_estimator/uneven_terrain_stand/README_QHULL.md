Install libqhull 6 (2012.1-4, trusty):

apt-get install libqhull-dev=2012.1-4
apt-get install libqhull-doc=2012.1-4
apt-get install libqhull6=2012.1-4

Nothing else needs to be done.



For documentation purposes, this is how the shared libraries and include folders were added in the CMakeLists.txt:

To find the shared library libqhull.so or directly libqhull.so.6 do this:
ldconfig -p | grep libqhull
e.g. /usr/lib/x86_64-linux-gnu/libqhull.so.6

Include this library in the CMakeLists target_link_libraries.

The header files need to be included as well. Find them with
locate libqhull | grep include/libqhull
e.g. /usr/include/libqhullcpp

Put this path into the CMakeLists include_directories.



