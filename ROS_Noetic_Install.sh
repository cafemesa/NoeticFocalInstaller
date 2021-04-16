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
sudo apt install ros-noetic-desktop-full -y

## Environment setup
rossource="source /opt/ros/noetic/setup.bash"
echo "$rossource" >> ~/.bashrc
eval $rossource

## Dependencies for building packages
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update

## Create a ROS Workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
rossource2="~/catkin_ws/src/devel/setup.bash"
echo "$rossource2" >> ~/.bashrc
echo "cd ~/catkin_ws" >> ~/.bashrc

## Setup Display variables
echo "export LIBGL_ALWAYS_INDIRECT=" >> ~/.bashrc
echo "export DISPLAY=\"`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0\"" >> ~/.bashrc
echo "export DISPLAY=\"`sed -n 's/nameserver //p' /etc/resolv.conf`:0\"" >> ~/.bashrc
echo "export DISPLAY=$(ip route|awk '/^default/{print $3}'):0.0" >> ~/.bashrc
eval $rossource



