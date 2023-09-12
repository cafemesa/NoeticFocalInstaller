# ROS Noetic Installer for Ubuntu 20.04 and WSL with compatibility for Turtlebot 2

## 1. Install Ubuntu 20.04

### 1.1. Windows 

1. Enable the Windows Subsystem for Linux (PowerShell as Administrator)
    ``` 
    dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
    ```
1. Enable Virtual Machine feature (PowerShell as Administrator)
    ``` 
    dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart 
    ```
1. Restar PC
1. Install Ubuntu 20.04 using Microsoft Store
1. Open Ubuntu 20.04 and set your user and password
1. Check WSL version(PowerShell as Administrator)
    ``` 
    wsl -l -v 
    ```
    1. If version is 2 change to 1 (PowerShell as Administrator)

    ``` 
    wsl --set-version Ubuntu-20.04 1 
    ```

1. Install VcXsrv Windows X Server
    ```
    https://sourceforge.net/projects/vcxsrv/
    ```

### 1.2. MacOS and VirtualBox

To install Ubuntu 20.04 on MacOs using VirtualBox refer to this video that explain the complete process: https://www.youtube.com/watch?v=Hzji7w882OY

### 1.3. Ubuntu 20.04 alongside Windows 10

To install Ubuntu 20.04 alongside Windows 10 refer to this video that explain the complete process: https://www.youtube.com/watch?v=Z-Hv9hOaKso

## 2. Install ROS Noetic and Packages

We have created some scripts to facilitate the installation of ROS and the packages needed for the tutorials. If you are using Ubuntu on Windows WSL just open Ubuntu or if you are using Ubuntu alongside or a virtual machine open terminal and execute the following scripts.

```
cd ~/ 
sudo apt-get install git -y
git clone https://github.com/cafemesa/NoeticFocalInstaller
cd ~/NoeticFocalInstaller/
```

### 2.1. For Ubuntu WSL on Windows 
```
bash ./Final_Noetic_WSL.sh
```

### 2.2. For Ubuntu in Virtual machine or alongside
```
bash ./Final_Noetic_Alongisde.sh
```

**FOR ALL** To fix the Turtlebot Teleoperation execution error replace line 160 from « (e_errno, msg, *_) = e.args » with « (e_errno, msg) = e.args » in tcpros_base.py file. Use the next command to open the file

```
sudo gedit /opt/ros/noetic/lib/python3/dist-packages/rospy/impl/tcpros_base.py 
```

## 3. Run the example files

### 3.1. Perception

1. RGB-D Camera
    - First terminal: `roslaunch project_2021 Perception_camera.launch`
    - Second terminal: `roslaunch kobuki_keyop keyop.launch`

1. 3D LiDAR
    - First terminal: `roslaunch project_2021 Perception_velodyne.launch`
    - Second terminal: `roslaunch kobuki_keyop keyop.launch`

### 3.2. Mapping

1. RGB-D Camera
    - First terminal: `roslaunch project_2021 Mapping_camera.launch`
    - Second terminal: `roslaunch kobuki_keyop keyop.launch`

1. 3D LiDAR
    - First terminal: `roslaunch project_2021 Mapping_velodyne.launch`
    - Second terminal: `roslaunch kobuki_keyop keyop.launch`

### 3.3. Navigation

1. RGB-D Camera
    - First terminal: `roslaunch project_2021 Navigation_camera.launch`

1. 3D LiDAR
    - First terminal: `roslaunch project_2021 Navigation_velodyne.launch`

### 3.4. People detection

1. 3D LiDAR
    - First terminal: `roslaunch project_2021 People_Detection_velodyne.launch`


# For Java Programers

This installer includes a version of rosjava compatible with ROS Noetic, with which you can create Java-based projects and packages.

## 1. Create a ROS Java package

```
cd ~/catkin_ws/src
catkin_create_rosjava_pkg [PACKAGE_NAME]
cd ~/catkin_ws
catkin_make
```

## 2. Create a ROS Java project

```
cd ~/catkin_ws/src/[PACKAGE_NAME]
catkin_create_rosjava_project [PROJECT_NAME]
cd ~/catkin_ws
catkin_make
```

## 3. Run example files

1. First terminal

```
roscore
```

2. Second terminal

```
rosrun [PACKAGE_NAME] [PROJECT_NAME] com.github.rosjava.[PACKAGE_NAME].[PROJECT_NAME].Talker
```

3. Third terminal

```
rosrun [PACKAGE_NAME] [PROJECT_NAME] com.github.rosjava.[PACKAGE_NAME].[PROJECT_NAME].Listener
```


# For Android Programers

This installer includes a version of base project for android that allows to subscribe to different topics.


1. Setup ROS network

    - Open .bashrc file: `gedit ~/.bashrc`
    - Add the export lines at the end of the file:

    ```
    Export ROS_IP=[YOUR_PC_IP]
    Export ROS_HOSTNAME=[YOUR_PC_IP]
    Export ROS_MASTER_URI=http://[YOUR_PC_IP]/11311
    ```

    - Save and exit
    - run `source ~/.bashrc`

2. Clone android project

```
cd ~/
git clone https://github.com/cafemesa/rosAndroidBaseProject.git
```

3. Open with Android Studio

4. Setup default IP Server in android project

    - Open LoginActivity.java file located in app &#8594; java &#8594; com.unidue.socialrobotics &#8594; LoginActivity.java

    - Modify the line 42 with your correct IP:

    ```
    editor.putString("MasterIP", "[YOUR_PC_IP]");
    ```

4. Run Turlebot Navigation Example `roslaunch project_2021 Navigation_velodyne.launch`

5. Compile and run android project: There is not user and password only push LOGIN