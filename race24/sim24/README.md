# README #

## Install ##

* Checked on Linux (preffered Ubuntu, Mint or other Debian distribution) on live image and normal install.
* Checked on Windows x64, Python3.9+ with Visual Studio 2019 or higher, script executed from Git Bash
* To install, the easiest way, start setup.sh after unpacking the archive
    * download and extract the sim24 folder in an appropriate place (with enough rights to read/write)
    * open a terminal (on Linux) or Git Bash (on Windows) and change directory to where the setup.sh file got unpacked and then run: `bash setup.sh`
    * the script will detect the operating system type ang may give some directions case in which it asks to confirm using ENTER
    * it is likely that the password for running as root will be asked for installing build-essentials, python3-dev and python3-venv (on Linux)
    * if all goes well, the simulation is started automatically, press ESC to leave it
    * use PyCharm to open the sim24 folder, everything should work well
    * if something goes wrong, try to trace and fix the problem by analyzing the script output, running the commands manually, one by one and finally updating the script
    * please report the encountered issues
* KNOWN ISSUES
    * seems that Virtual Box virtual machines DO NOT properly support graphical acceleration so are unusable
    * on recent Ubuntu distributions with nVidia GPU when calling glVertexPointer(3, GL_FLOAT, 0, vs) the error "OpenGL.error.Error: Attempt to retrieve context when no valid context" might appear; it seems that disabling the Wayland support in the display manager might help: consider adding the line  "WaylandEnable=false" in /etc/gdm3/custom.conf and reboot (issue encoutered at least 3 times, thanks for solution to Rares C.)
    * on some nVidia hardware, under Windows, each track segment gets doubled on diplay, producing impossible track configurations (thanks for the report to Rares C.); it seems this happens when calling the display lists for track segments from the display list for the entire track; support for avoiding this issue was added and needs to manually set to True the variable `OPENGL_DISPLAYLIST_BUGFIX` which by default is set to False in the carsim.py script. (side note, on the same system under recent Ubuntu it was needed to disable Wayland support in the display manager) 

## Usages ##

* Simulates track runs using a DriverDizzy implementation, or if reconfigured to DriverBlind, demonstrates running 5 laps using a precalculated aproximative lane centered trajectory
* Multiple track perspective
* Timewarp - pressing 0-9 keys alter run speed (0-fastest, 1-attempt realtime, 2-9 times slower, space - pause)
	* slow-motion with improved frame rate, while simulation still runs *almost* accurately)
	* acceleration of rendering to focus only on rendering first car and just while moving
* Autonomous driving simulation by extending the Driver/DriverBlind classes
	* observe method - receives snapshot of track (@60Hz)
	* control method - called by simulation engine on each physical simulation step to update course references (@1kHz)
* Camera calibration routine testing
* Manual simulation quality adjustment (by editing lines with if 1: and if 0:) of display information detail (less info, more speed)
    * enable antialias by increasing multisample value for more realistic images, also slower simulation speed
    * reduce display perspectives (hide proessed pixy2 images and top-view perspective)
    * windowed/fullscreen/non-windowed mode
    * initial window position changing in script
    * change lighting conditions (turn on/off environment lights, move them arround change their properties, ...)
    * ...
    
## HowTo ##

* Make sure all dependencies are met
* Use run.sh (from terminal opened in the sim24 folder) to start the simulation
* Use keyboard as instructed
* Adjust camera configuration dictionary or uncomment/use the load_pixy_prm_file_to_dict function call to load a .prm config file
* Adjust simulation parameters in main (Change refresh rate, Track design...)
* Adjust other parameters by following object hierarchy (Camera placement, VirtualCar steering and traction options)
* Adjust/Replace the DriverDizzy subclass to improve rule compliance, performance, robustness...

## More Info on building C++ Python Extensions on Windows with msvc compiler ##

https://learn.microsoft.com/en-us/visualstudio/python/working-with-c-cpp-python-in-visual-studio?view=vs-2022#use-pybind11 

## More Info on Pixy2 hardware and api ##

* Hardware info
https://pixycam.com/2021/05/20/introducing-pixy-2-1/
* PixyMon advanced settings
https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:line_expert
* Line tracking quick start
https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:line_quickstart
* Line tracking API
https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:line_api
* Pixy2 Firmware, API and Software - original source code
https://github.com/charmedlabs/pixy2

## Other stuff ##

* Quaternions
https://quaternions.online/
* PyCharm as AppImage
https://github.com/felipefacundes/pycharm-appimage
* Wheel Encoders and more Buggy3 info
https://notblackmagic.com/projects/wheel-encoder/
https://notblackmagic.com/bitsnpieces/mr-buggy3/
* PX4 Autopilot
https://notblackmagic.com/snippets/px4-autopilot/
