#!/bin/bash

if 0; then # activate by manually setting the 0 to 1 for non standard setup...
    # to compile just the module with the g++ it is enough to have g++ toolchain (mingw on windows or build-essential on linux):
    cd pixy2sim && g++ -Ofast pixy.cpp -fPIC -shared fw.cpp misc.cpp -o pixy2sim`python3-config --extension-suffix` -I/usr/include/python`python3 --version | grep -Po "3\.\d+"` `python3 -m pybind11 --includes`  \
    && cp *.pyd ../ # copy the pyd file in the sim24 working folder
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # tested on Linux Mint 21.2 Victoria (boot from usb stick and reconfigured software sources for national high-throughput servers, no install!)
    sudo apt install python3-dev python3-venv build-essential \
    && python3 -m venv --upgrade-deps venv \
    && source ./venv/bin/activate \
    && cd pixy2sim \
    && python -m pip install . --verbose \
    && cd .. \
    && pip3 install pygame PyOpenGL numpy opencv-python \
    && echo -e "\n\n\nSetup completed OK! Runnning once to test...\n\n\n" \
    && python3 carsim.py \
    && deactivate \
    && chmod +x *.sh \
    && echo -e "\n\n\nSetup test completed. Start PyCharm, open the sim24 folder, requirements should already be met!\n"

elif [[ "$OSTYPE" == "msys" ]]; then
    # run in Git bash, start bash use cd to change directory to where the script is and launch ./setup.sh
    which python # display full path to python executable    
    echo -e "${PATH//:/\\n}" | grep -i Python # display system paths that contain Python word
    echo -e "Make sure at least one valid python binary path is listed above and press ENTER to continue or CTRL+C to abort!"
    read # wait for ENTER to be pressed

    python -m venv --upgrade-deps venv \
    && source ./venv/Scripts/activate \
    && cd pixy2sim \
    && python -m pip install . --verbose \
    && cd .. \
    && python -m pip install pygame PyOpenGL numpy opencv-python \
    && echo -e "\n\n\nSetup completed OK! Runnning once to test...\n\n\n" \
    && python carsim.py \
    && deactivate \
    && echo -e "\n\n\nSetup test completed. Start PyCharm, open the sim24 folder, requirements should already be met!\n"

else
	echo -e "Unsupported OSTYPE: $OSTYPE; please consider editing the script to add support based on the existing supported OSTYPES!"
fi

echo -e "\nOptionally use the run.sh script to launch without PyCharm or other IDE\n"



