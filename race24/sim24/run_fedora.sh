#!/bin/bash
# cross-platform script for running the simulation without IDE dependencies 
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
	. ./venv/bin/activate && LIBGL_ALWAYS_SOFTWARE=1 PYOPENGL_PLATFORM=glx SDL_VIDEODRIVER=x11 python3 carsim.py
	deactivate
elif [[ "$OSTYPE" == "msys" ]]; then
	. ./venv/Scripts/activate && python carsim.py
    deactivate
else
	echo -e "Unsupported OSTYPE: $OSTYPE; please consider editing the script to add support based on the existing supported OSTYPEs!"
fi
