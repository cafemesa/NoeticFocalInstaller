# Turtlebot 2 on Ubuntu 20.04 and WSL

## Install Ubuntu 20.04

### Windows 

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

### MacOS and VirtualBox

To install Ubuntu 20.04 on MacOs using VirtualBox refer to this video that explain the complete process: https://www.youtube.com/watch?v=Hzji7w882OY

### Ubuntu 20.04 alongside Windows 10

To install Ubuntu 20.04 alongside Windows 10 refer to this video that explain the complete process: https://www.youtube.com/watch?v=Z-Hv9hOaKso

## Install ROS Noetic

We have created some scripts to facilitate the installation of ROS and the packages needed for the tutorials. If you are using Ubuntu on Windows WSL just open Ubuntu or if you are using Ubuntu alongside or a virtual machine open terminal and execute the following scripts.

```
cd ~/ 
sudo apt-get install git -y
git clone https://github.com/cafemesa/Focal_Noetic_WSL
cd ~/Focal_Noetic_WSL/
```

### For Ubuntu WSL on Windows 
```
bash ./Ubuntu_WSL.sh
```

### For Ubuntu in Virtual machine 
```
bash ./Ubuntu_virtual_machine.sh
```

### For Ubuntu alongside
```
bash ./Ubuntu_alongside.sh
```

## Compile Turtlebot 2 packages for noetic

```
cd ~/Focal_Noetic_WSL/
bash ./TurtleBot2.sh
```

### Fix Turtlebot Teleoperation execution error

Replace line 160 from « (e_errno, msg, *_) = e.args » to « (e_errno, msg) = e.args » in tcpros_base.py file. Use the next command to open the file

```
sudo gedit /opt/ros/noetic/lib/python3/dist-packages/rospy/impl/tcpros_base.py 
```
