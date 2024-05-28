.. RT2-1 Documentation documentation master file, created by
   sphinx-quickstart on Sun May 26 19:21:42 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to RT2_Assignment's documentation!
===========================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:
     

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


RT2_Assignment's documentation
**************************************
This repository contains ROS (Robot Operating System) nodes and services for controlling and monitoring a robot's movements. The system allows users to set targets for the robot, cancel ongoing actions, and obtain information about the robot's current state, target positions, distances, and average speed.

Prerequisites
*************************
ROS installed on your system (ROS Installation Instructions)
Catkin workspace configured

Getting Started
*************************
    Clone the repository into your catkin workspace:

    git clone https://github.com/OuassimMilous/RT2-1-Documentation

Build the workspace:
*************************
    cd catkin_ws
    catkin_make
    source devel/setup.bash
    roslaunch robot_control robot_control.launch



RT2_Assignment Nodes
===========================

NodeA:
*************************

.. automodule:: scripts.nodeA
   :members:
   :undoc-members:
   :show-inheritance:
   
 

NodeB:
*************************

.. automodule:: scripts.nodeB
   :members:
   :undoc-members:
   :show-inheritance:

   

NodeC:
*************************

.. automodule:: scripts.nodeC
   :members:
   :undoc-members:
   :show-inheritance:
