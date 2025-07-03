# **ADAMSim: PyBullet-Based Simulation Environment for Research on Domestic Mobile Manipulator Robots**

<p align="center">
  <a href="https://mobile-robots-group-uc3m.github.io/AdamSim/">
    <img src="installation/Images/LogoADAMSim.png" height="200"/>
  </a>
</p>

This repository introduces ADAMSim, a PyBullet-based simulation environment tailored for Ambidextrous Domestic Autonomous Manipulator (ADAM), developed to support research in navigation, manipulation, and learning for domestic robotics. The simulator accurately replicates the structure and behavior of the physical robot, enabling robust sim-to-real and real-to-sim algorithm transfer. ADAMSim follows a modular design, including navigation, arm and hand kinematics, perception, and ROS communication. This architecture allows synchronized operation between the real robot and its digital twin. Several example applications were developed, ranging from vision and grasping tasks to navigation and teleoperation, including experiments running both simulated and real robots simultaneously. Its open-source and flexible design makes ADAMSim a powerful tool for safe and reproducible algorithm development and experimentation in household robotics. The platform is also intended to support future research in indoor mapping, advanced manipulation learning, and educational projects, serving as a test bed.

# Website
On our website, you can access video examples as well as images that showcase some of the functionalities of this simulator.
For more information, visit our website: [ADAMSim webpage](https://mobile-robots-group-uc3m.github.io/AdamSim/)
# Installation
To be used on your device, follow the installation steps below.

**Requierements:**
- There is a `requirements.txt` file with all the elements neccesary for the correct instalation.

> [!CAUTION]  
> ADAMSim has been developed in Ubuntu. Some of the PyKDL-based functions use a proprietary ROS1 parser so they do not work on Windows. In case you want to use it on Windows without WSL, the functions that depend on PyKDL or URDF Parser must be commented out. In future versions, we will be looking for full cross-platform integration..


## Install miniconda (highly-recommended)
It is highly recommended to install all the dependencies on a new virtual environment. For more information check the conda documentation for [installation](https://conda.io/projects/conda/en/latest/user-guide/install/index.html) and [environment management](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html). For creating the environment use the following commands on the terminal.

```bash
conda create -n ADAMSim python=3.8
conda activate ADAMSim
```

## Install repository
Clone the repository in your system.
```bash
git clone https://github.com/Mobile-Robots-Group-UC3M/AdamSim.git
```
Then enter the directory and install all the requierements:
```bash
cd installation
pip install -r requirements.txt
```
After that, you have to install the package in editable mode, so you can modify the code and see the changes without reinstalling it.
```bash
cd ..
pip install -e .
```
Then you are ready to work with ADAMSim 



## **Installation of third party programs**
In order to make use of pyKDL the pip install does not work properly, therefore third party packages need to be implemented. For the correct operation it is necessary to install using the packages described in the following [web page](https://anaconda.org/conda-forge/python-orocos-kdl).

```bash
conda install conda-forge::python-orocos-kdl
conda install conda-forge/label/cf202003::python-orocos-kdl
```

Additionally, in case you want to use the robotic hands with the ADAM robot in the real environment, it is necessary to add the Inspire Hands package to your workspace, compile it and install the following ROS1 package:

```bash
sudo apt-get install ros-<your distro>-serial
```
## **TO DO**
* Add documentation of the functions
* Add more examples
* Add more functionalities
* Change pyKDL with other kinematics library that works on Windows and Linux

# Citation
If you use this code, please quote our works :blush:

In progress :construction_worker:

## Acknowledgement
This work was supported by Advanced Mobile dual-arm manipulator for Elderly People Attendance (AMME) (PID2022-139227OB-I00), funded by Ministerio de Ciencia e Innovacion.

This work has been developed in the [Mobile Robotics Group](https://github.com/Mobile-Robots-Group-UC3M) from RoboticsLab, at University Carlos III de Madrid.
