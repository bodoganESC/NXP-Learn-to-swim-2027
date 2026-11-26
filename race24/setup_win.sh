#!/bin/bash
# run in Git bash, start bash use cd to change directory to where the script is and launch ./setup_win.sh
echo -e "${PATH//:/\\n}" | grep -i Python
echo -e "Make sure at least one valid python binary path is listed above!"
echo -e "Make sure sim24*.zip file in the current folder and press ENTER to continue!"
read # wait for ENTER to be pressed

unzip sim24*.zip \
&& cd sim24 \
&& python -m venv --upgrade-deps venv \
&& source ./venv/Scripts/activate \
&& pip install pygame PyOpenGL numpy opencv-python \
&& cd pixy2sim \
&& python -m pip install . --verbose \
&& cd .. \
&& python carsim.py

# above script compiles module from sources using visual studio compiler (again Git bash is the terminal to use...)
# alternatively only the binary of the module is needed, it can be obtained with a mingw compiler
# https://github.com/skeeto/w64devkit/releases (includes mingw + other utilities for compiling C++ under windows)
# https://github.com/skeeto/w64devkit/releases/download/v1.21.0/w64devkit-1.21.0.zip (the version that I tested, just needs to be unpacked and add the bin folder to PATH, as follows:)

# NOTE: the g++ line below needs to be updated so that -L/c/Program\ Files/Python/libs points to the actual libs of the system installation...

# unzip w64devkit*.zip \
# && PATH=$PATH:~/Downloads/w64devkit/bin/ \
# && unzip sim24*.zip \
# && cd sim24 \
# && python -m venv --upgrade-deps venv \
# && source ./venv/Scripts/activate \
# && pip install pygame PyOpenGL numpy opencv-python pybind11 \
# && cp -r ./venv/Lib/site-packages/pybind11/include/pybind11 ./venv/Include/pybind11 \
# && cd pixy2sim \
# && g++ -fmax-errors=1 -Ofast -Wall pixy.cpp -fPIC -shared -DMS_WIN64 fw.cpp misc.cpp -o pixy2sim.pyd `python -m pybind11 --includes` -L/c/Program\ Files/Python/libs -lPython39 \
# && cp pixy2sim.pyd ../ \
# && cd .. \
# && python carsim.py
