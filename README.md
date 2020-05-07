# CTH Yumi

This is a package for controlling ABB's collaborative robot YuMi. It was written in conjunction with the Bachelor Thesis "EENX-15-20-11 - Collaborative and autonomous robots, COBOTS" at Chalmers Institute of Technology in 2020. 

## Installation Guide
This guide is meant for students with no prior experience in using ROS(other than basic tutorials found on the ROS website) or YuMi.
The main package which will be used to simulate YuMi is this one :
https://github.com/kth-ros-pkg/yumi
However, installation guidelines do not seem to be completely up to date, so here is an alternate method(which includes the installation of the CTH package aswell).


Install Ros(Melodic) and follow basic user tutorials to get an understanding of how the operative system works.


The Industrial Core packages and ABB packages have to be installed. Source these to your main ROS installation.
You can install them with the following commands
``` sudo apt-get install ros-melodic-industrial-core
sudo apt-get install ros-melodic-abb 
``` 


Then, use the command ```rospack list-names``` to check that the following packages are installed:

abb_driver
abb_irb2400_moveit_config
abb_irb2400_moveit_plugins
abb_irb2400_support
abb_irb4400_support
abb_irb5400_support
abb_irb6600_support
abb_irb6640_moveit_config
abb_irb6640_support
abb_resources
industrial_deprecated
industrial_msgs
industrial_robot_client
industrial_robot_simulator
industrial_trajectory_filters


Create the YuMi workspace:

```bash
mkdir -p yumi_depends_ws/src
cd yumi_depends_ws/src
catkin_init_workspace
cd ..
catkin_make
echo "export YUMI_WS='$(pwd)'" >> ~/.bashrc
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
```

Clone the Yumi Workspace

```bash
bash
cd $YUMI_WS/src
git clone https://github.com/EENX15-20-11-COBOTS/yumi.git
git clone https://github.com/DVNO911/cth_yumi.git
git clone https://github.com/EENX15-20-11-COBOTS/generic_control_toolbox.git
git clone https://github.com/EENX15-20-11-COBOTS/robot_kinematic_services.git

```
You are now sourcing repositories forked from Diogo Almeida's project. These contain some minor modifications that we found necessary to build the workspace. For more updated versions of the packages please check upstream in each repository.

Run rosdep to resolve dependencies

```bash
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

Build(This might fail, if it does proceed to next step)

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```


Check that you are sourcing the correct branch

```bash
cd $YUMI_WS
rm -rf build/ devel/
cd src/
rm -rf yumi
git clone https://github.com/EENX15-20-11-COBOTS/yumi.git
cd yumi
git checkout origin/$ROS_DISTRO 
cd $YUMI_WS
catkin_make -DCMAKE_BUILD_TYPE=Release
```

Catkin should successfully build now. You might run into some additional problems but if you manage to build the package then the worst is over :)


## Contributing

This package is currently only meant to be developed by students at Chalmers.

## Authors and Acknowledgments

David Wannheden\
Charles Strömblad\
Simon Mattson Tenser\
Ahmad Wahba\
William West\
Nick Norizadeh

Special thanks to Yiannis Karayiannidis for guidance in this project.

## License

Missing
