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
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo rosdep init
rosdep update

## Create a ROS Workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
rossource2="source ~/catkin_ws/devel/setup.bash"
echo "$rossource2" >> ~/.bashrc
echo "cd ~/catkin_ws" >> ~/.bashrc
eval $rossource

# Go to src folder
cd ~/catkin_ws/src/

# Install GIT
sudo apt-get install git -y
sudo apt-get install openjdk-8-jdk -y

# Turtlebot Packages
git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_msgs.git
git clone https://github.com/turtlebot/turtlebot_apps.git
git clone https://github.com/turtlebot/turtlebot_simulator
git clone --branch indigo https://github.com/turtlebot/turtlebot_viz.git

# Kobuki Packages
git clone https://github.com/yujinrobot/kobuki_msgs.git
git clone https://github.com/yujinrobot/kobuki_desktop.git
git clone --branch melodic https://github.com/yujinrobot/kobuki.git
git clone https://github.com/yujinrobot/kobuki_core.git
git clone https://github.com/ros-perception/depthimage_to_laserscan.git
git clone https://github.com/ros-perception/slam_gmapping.git

# Velodyne driver and simulator
git clone https://github.com/ros-drivers/velodyne.git
git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git

# yocs_controllers
git clone https://github.com/yujinrobot/yocs_msgs.git
git clone https://github.com/yujinrobot/yujin_ocs.git

# Clone tutorial files
git clone https://github.com/cafemesa/PFF.git
git clone https://github.com/cafemesa/PFF_PeD.git
git clone https://github.com/cafemesa/Project_ss2021.git
git clone https://github.com/cafemesa/PFF_SeM.git

# Clone ROS JAVA
git clone https://github.com/cafemesa/rosjava_build_tools.git

# ar_track_alvar_msgs
git clone --branch melodic-devel https://github.com/ros-perception/ar_track_alvar.git
mv ar_track_alvar/ar_track_alvar_msgs ./
rm -rf ar_track_alvar

# Dependencies
sudo apt-get install ros-noetic-openslam-gmapping -y
sudo apt-get install ros-noetic-ecl-build  -y
sudo apt-get install ros-noetic-ecl-geometry -y
sudo apt-get install ros-noetic-ecl-threads -y
sudo apt-get install ros-noetic-ecl-devices -y
sudo apt-get install ros-noetic-ecl-sigslots -y
sudo apt-get install ros-noetic-ecl-command-line -y
sudo apt-get install ros-noetic-ecl-streams -y
sudo apt-get install ros-noetic-joy -y
sudo apt-get install ros-noetic-base-local-planner -y
sudo apt-get install ros-noetic-move-base -y
sudo apt-get install ros-noetic-kobuki-driver -y
sudo apt-get install libusb-dev -y
sudo apt-get install libftdi-dev -y
sudo apt-get install ros-noetic-amcl -y
sudo apt-get install ros-noetic-map-server -y
sudo apt-get install pyqt5-dev-tools -y
sudo apt-get install gedit -y

sudo apt install python2 -y
sudo apt-get install curl
curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
sudo python2 get-pip.py
python -m pip install pyyaml
python -m pip install rospkg

sudo apt-get install libqt5core5a
sudo strip --remove-section=.note.ABI-tag /lib/x86_64-linux-gnu/libQt5Core.so.5

cd ~/catkin_ws/
rosdep install --from-path src -i -y -r
catkin_make
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 10

## Setup Display variables
echo "export LIBGL_ALWAYS_INDIRECT=" >> ~/.bashrc
echo "export DISPLAY=\"`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0\"" >> ~/.bashrc
echo "export DISPLAY=\"`sed -n 's/nameserver //p' /etc/resolv.conf`:0\"" >> ~/.bashrc
echo "export DISPLAY=$(ip route|awk '/^default/{print $3}'):0.0" >> ~/.bashrc

chmod +x ~/catkin_ws/src/rosjava_build_tools/src/rosjava_build_tools/gradle/gradlew

rossource3="source ~/.bashrc"
eval $rossource3
