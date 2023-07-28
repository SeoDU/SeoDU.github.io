---
layout: post
title: Azure Kinect DK install
subtitle: SDK & ROS driver
tags: [Camera]
comments: true
---

ToF방식의 RGB-D 카메라인 Azure Kinect DK ROS설치 과정 정리

- 경로 추가
```bash
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
sudo apt-get update
```



- 설치하기
```bash
sudo apt-get install k4a-tools
sudo apt search k4a
sudo apt-get install libk4a1.4-dev
```


- Linux device setup
```bash
cd /etc/udev/rules.d
gedit 99-k4a.rules
```

99-k4a.rules파일에 다음 경로의 텍스트를 복사하여 붙여넣기
[https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/scripts/99-k4a.rules](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/scripts/99-k4a.rules)

이때 k4aviewer하면 카메라가 보여야 함


- ROS driver 설치 및 실행

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver/tree/melodic
catkin build
source devel/setup.zsh

roslaunch azure_kinect_ros_driver driver.launch
또는
roslaunch azure_kinect_ros_driver kinect_rgbd.launch
```

- 참조 문서:
[https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/melodic/docs/building.md](https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/melodic/docs/building.md)
