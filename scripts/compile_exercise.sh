#!/bin/sh

cd /RoboticsAcademy/exercises/$@
cd cpp_lib
mkdir build
cd build/
cmake ..
make
chmod 777 *.so
mv *.so ../../cpp_template/libs/
cd ..
rm -r build/
# Copy the headers to the cpp_template directory
cp -r include/ ../cpp_template/libs/

