# Turtlebot 2 on Ubuntu 20.04 and WSL

## Install Ubuntu 20.04 subsystem

1. Open Windows PowerShell as Administrator
2. Enable the Windows Subsystem for Linux
```
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart  
```
3. Enable Virtual Machine feature
```
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
```
4. Restar PC
5. Install Ubuntu 20.04 using Microsoft Store
6. Open Ubuntu 20.04 and set your user and password
7. Check WSL version
```
wsl -l -v  
```

1. If version is 2

```
wsl --set-version <distribution name> 1
```

## Install VcXsrv Windows X Server

https://sourceforge.net/projects/vcxsrv/

## Install ROS Noetic

```
git clone https://github.com/cafemesa/Focal_Noetic_WSL
cd ~/Focal_Noetic_WSL/
bash ./ROS_Noetic_Install.sh
```
