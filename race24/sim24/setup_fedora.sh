#!/bin/bash
# Adapted for Fedora Linux (Wayland/X11) with NVIDIA GPU

echo "Starting sim24 environment setup for Fedora Linux + NVIDIA..."
echo "You will be asked for your sudo password to install gcc-c++, python3-devel, and OpenGL libraries."

# Install C++ compiler, Python headers, and OpenGL/GLU development libraries
sudo dnf install -y gcc-c++ python3-devel mesa-libGL-devel mesa-libGLU-devel \
&& python3 -m venv --upgrade-deps venv \
&& source ./venv/bin/activate \
&& pip install --upgrade pip setuptools wheel \
&& cd pixy2sim \
&& python -m pip install . --verbose \
&& cd .. \
&& pip install pygame PyOpenGL numpy opencv-python \
&& echo -e "\n\n\nSetup completed OK! Launching simulator test...\n\n\n" \
&& SDL_VIDEODRIVER=x11 python3 carsim.py \
&& deactivate \
&& echo -e "\n\n\nSetup test completed. Start PyCharm/VSCode, open the sim24 folder, requirements should already be met!\n"