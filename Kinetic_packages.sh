sudo apt install ros-kinetic-catkin ros-kinetic-rospack python-wstool openjdk-8-jdk -y

## Create a ROS Workspace
mkdir -p ~/rosjava/src
wstool init -j4 ~/rosjava/src https://raw.githubusercontent.com/rosjava/rosjava/kinetic/rosjava.rosinstall
eval $rossource
cd ~/rosjava
rosdep update
rosdep install --from-paths src -i -y
catkin_make
rossource2="source ~/rosjava/devel/setup.bash"
echo "$rossource2" >> ~/.bashrc
echo "cd ~/catkin_ws" >> ~/.bashrc

## Setup Display variables
echo "export LIBGL_ALWAYS_INDIRECT=" >> ~/.bashrc
echo "export DISPLAY=\"`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0\"" >> ~/.bashrc
echo "export DISPLAY=\"`sed -n 's/nameserver //p' /etc/resolv.conf`:0\"" >> ~/.bashrc
echo "export DISPLAY=$(ip route|awk '/^default/{print $3}'):0.0" >> ~/.bashrc
eval $rossource