# Turtlebot 2 on Ubuntu 20.04 and WSL

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

## 2. Install ROS Noetic

We have created some scripts to facilitate the installation of ROS and the packages needed for the tutorials. If you are using Ubuntu on Windows WSL just open Ubuntu or if you are using Ubuntu alongside or a virtual machine open terminal and execute the following scripts.

```
cd ~/ 
sudo apt-get install git -y
git clone https://github.com/cafemesa/Focal_Noetic_WSL
cd ~/Focal_Noetic_WSL/
```

### 2.1. For Ubuntu WSL on Windows 
```
bash ./Ubuntu_WSL.sh
```

### 2.2. For Ubuntu in Virtual machine 
```
bash ./Ubuntu_virtual_machine.sh
```

### 2.3. For Ubuntu alongside
```
bash ./Ubuntu_alongside.sh
```

## 3. Compile Turtlebot 2 packages for noetic

```
cd ~/Focal_Noetic_WSL/
bash ./TurtleBot2.sh
```

To fix the Turtlebot Teleoperation execution error replace line 160 from « (e_errno, msg, *_) = e.args » with « (e_errno, msg) = e.args » in tcpros_base.py file. Use the next command to open the file

```
sudo gedit /opt/ros/noetic/lib/python3/dist-packages/rospy/impl/tcpros_base.py 
```

## 4. Run the example files

### 4.1. Perception

1. RGB-D Camera
    - First terminal: `roslaunch project_2021 Perception_camera.launch`
    - Second terminal: `roslaunch turtlebot_teleop keyboard_teleop.launch`

1. 3D LiDAR
    - First terminal: `roslaunch project_2021 Perception_velodyne.launch`
    - Second terminal: `roslaunch turtlebot_teleop keyboard_teleop.launch`

### 4.2. Mapping

1. RGB-D Camera
    - First terminal: `roslaunch project_2021 Mapping_camera.launch`
    - Second terminal: `roslaunch turtlebot_teleop keyboard_teleop.launch`

1. 3D LiDAR
    - First terminal: `roslaunch project_2021 Mapping_velodyne.launch`
    - Second terminal: `roslaunch turtlebot_teleop keyboard_teleop.launch`

### 4.3. Navigation

1. RGB-D Camera
    - First terminal: `roslaunch project_2021 Navigation_camera.launch`

1. 3D LiDAR
    - First terminal: `roslaunch project_2021 Navigation_velodyne.launch`

### 4.4. People detection

1. RGB-D Camera
    - First terminal: 

1. 3D LiDAR
    - First terminal: `roslaunch project_2021 People_Detection_velodyne.launch`
