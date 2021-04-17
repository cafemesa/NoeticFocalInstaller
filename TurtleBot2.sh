sudo apt-get install git -y


#Turtlebot Packages
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

# yocs_controllers
git clone https://github.com/yujinrobot/yocs_msgs.git
git clone https://github.com/yujinrobot/yujin_ocs.git

# ar_track_alvar_msgs
git clone --branch melodic-devel https://github.com/ros-perception/ar_track_alvar.git
mv ar_track_alvar/ar_track_alvar_msgs ./
rm -rf ar_track_alvar

#Dependencies
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

sudo apt-get install python -y
sudo apt-get install curl
curl https://bootstrap.pypa.io/2.7/get-pip.py --output get-pip.py
python get-pip.py
python -m pip install pyyaml
python -m pip install rospkg

sudo apt-get install libqt5core5a
sudo strip --remove-section=.note.ABI-tag /lib/x86_64-linux-gnu/libQt5Core.so.5

cd ..
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

## Grphics variables
#export LIBGL_ALWAYS_INDIRECT=
#export DISPLAY="`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0"
#export DISPLAY="`sed -n 's/nameserver //p' /etc/resolv.conf`:0"
#export DISPLAY=$(ip route|awk '/^default/{print $3}'):0.0

# For teleoperation
# sudo gedit /opt/ros/noetic/lib/python3/dist-packages/rospy/impl/tcpros_base.py 
# replace line 160 
# from (e_errno, msg, *_) = e.args
# to (e_errno, msg) = e.args