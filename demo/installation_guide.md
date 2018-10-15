# Neurobots Demonstrator Installation

Install required packages wstool and eigen3:

`sudo apt-get install python-wstool libeigen3-dev networkx`

Required non-standard ROS modules:

`sudo apt-get install ros-[indigo,jade,...]-tf`

`sudo apt-get install ros-[indigo,jade,...]-tf-conversions`

`sudo apt-get install ros-[indigo,jade,...]-image-transport`

`sudo apt-get install ros-[indigo,jade,...]-cv-bridge`

For the GUI:

`sudo apt-get install python-networkx qt5-default python-pyqt5`

If python-pyqt5 is not available for your Linux distribution have a look at  [this PyQt5 for Python 2.7 Guide.](PyQt5_manual_install_instructions.md). Important: On Ubuntu 16.04 you need to install

`sudo apt-get install qml-module-qtquick-controls`

If you use ROS Kinetic, the following library is also required:

`sudo apt-get install libopencv-dev`

Create your workspace:

`mkdir -p ~/my-ws/src`

Copy the contents of neurobots_demo.rosinstall into a file `~/my-ws/src/.rosinstall`

Fetch the code:

`cd ~/my-ws/src`

`wstool update`

Install the dependencies:

`cd ~/my-ws`

`sudo rosdep init` # only if never run before

`rosdep install --from-paths src --ignore-src`

Build:

`cd ~/my-ws`

`catkin_make`

In order to run the GUI, you need a planner, e.g. [Fast Downward](http://www.fast-downward.org)
The planner requires the following packages:

`sudo apt-get install cmake g++ g++-multilib mercurial make python`

The planner code is obtained by

`hg clone http://hg.fast-downward.org DIRNAME`

where `DIRNAME` is the name of the planner directory.
You have to update the file `~/my-ws/src/demo/goal_planner_gui/src/config.ini` accordingly
(under `[base_planner]` you have to alter `planner_dir=DIRNAME`) to the Fast Downward directory specified before.
Please do not commit your changes for the file and ignore it in git with
`git update-index --assume-unchanged config.ini`

You also need the PyPDDL repository from Moritz, which is available here.
Create a new folder for PyPDDL somewhere, then clone the repository

`git clone git@mlgitlab.informatik.uni-freiburg.de:neurobots/pypddl.git PYPDDLDIR`

You have add this directory to your `PYTHONPATH`. The easiest way to do this is to open
`~/.bashrc` with your favourite editor, and append the PYPDDLDIR to your PYTHONPATH 

`export PYTHONPATH=PYPDDLDIR:${PYTHONPATH}`

Now you can start the database with 

`roslaunch neurobots_launch database.launch`

and the goal planner GUI with

`roslaunch goal_planner_gui goal_planner_gui.launch`