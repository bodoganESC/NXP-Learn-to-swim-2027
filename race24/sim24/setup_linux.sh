#!/bin/bash
# tested on Linux Mint 21.2 Victoria (boot from usb stick and reconfigured software sources for national high-throughput servers, no install!)
echo Download the sim24 folder from drive \(it gets automatically packed as zip\)
echo put it the current folder and press ENTER to setup the environment \(you will be asked for root password to install python3-dev, python3-venv and build-essential\)
read # pausing and waiting for ENTER
unzip sim24*.zip \
&& cd sim24 \
&& sudo apt install python3-dev python3-venv build-essential \
&& python3 -m venv --upgrade-deps venv \
&& source ./venv/bin/activate \
&& cd pixy2sim \
&& python -m pip install . --verbose \
&& cd .. \
&& pip3 install pygame PyOpenGL numpy opencv-python \
&& echo -e "\n\n\nSetup completed OK!\n\n\n" \
&& python3 carsim.py \
&& deactivate \
&& echo -e "\n\n\nSetup test completed. Start PyCharm, open the sim24 folder, requirements should already be met!\n"



# the folowing one line
# && python -m pip install . --verbose \
# replaced the following 2 lines in order to comply with Python specification, but compiling the module in the g++ -Ofast may be beneficial...
#&& g++ -Ofast pixy.cpp -fPIC -shared fw.cpp misc.cpp -o pixy2sim.so -I/usr/include/python`python3 --version | grep -Po "3\.\d+"` \
#&& python3 setup.py install \
