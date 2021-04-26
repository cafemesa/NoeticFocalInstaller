# Install ROS Noetic
## Enable universe, multiverse and restricted repositories
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo add-apt-repository restricted
sudo apt update

## Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

## Set up your keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

## Installation
sudo apt update
sudo apt-get install ros-kinetic-desktop-full -y

## Environment setup
rossource="source /opt/ros/kinetic/setup.bash"
echo "$rossource" >> ~/.bashrc
eval $rossource

## Dependencies for building packages
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential ros-kinetic-catkin ros-kinetic-rospack python-wstool openjdk-8-jdk -y
sudo rosdep init
rosdep update

## Create a ROS Workspace
mkdir -p ~/rosjava/src
wstool init -j4 ~/rosjava/src https://raw.githubusercontent.com/rosjava/rosjava/kinetic/rosjava.rosinstall
cd ~/rosjava
rosdep update
rosdep install --from-paths src -i -y
catkin_make
rossource2="source ~/rosjava/devel/setup.bash"
echo "$rossource2" >> ~/.bashrc
eval $rossource

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
rossource3="source ~/catkin_ws/devel/setup.bash"
echo "$rossource3" >> ~/.bashrc
eval $rossource

cd ~/catkin_ws/src
catkin_create_rosjava_pkg social_robotics
cd ~/catkin_ws
catkin_make
cd ~/catkin_ws/src/social_robotics
catkin_create_rosjava_project app1
cd ~/catkin_ws
catkin_make

## Setup Display variables
echo "export LIBGL_ALWAYS_INDIRECT=" >> ~/.bashrc
echo "export DISPLAY=\"`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0\"" >> ~/.bashrc
echo "export DISPLAY=\"`sed -n 's/nameserver //p' /etc/resolv.conf`:0\"" >> ~/.bashrc
echo "export DISPLAY=$(ip route|awk '/^default/{print $3}'):0.0" >> ~/.bashrc
eval $rossource



