
To use the GUI you need to define the location where to find fast-downward.
To do so, please add your directory to the corresponding variable in CMakeLists.txt.
You can safely commit such changes. If you need to modify anything other than the path
in the config.ini file, you have to do the changes in config.ini.in and rebuild your catkin
workspace to force CMake to rebuild the config.ini file.

