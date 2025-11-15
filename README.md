# GreenSookSook(그린쑥쑥)

## UI(client)

## Backend(server)
```
pip3 install firebase-admin Flask flask-cors
```

## ROS2(ros2_ws)

### Python Dependencies

```
pip3 install firebase-admin
```

### Workspace path 설정
1. git clone 을 받아서 ~/workspace/rokey_a1 에 저장
2. ln -s ~/workspace/rokey_a1/ros2_ws ~/ros2_ws
3. cd ~/ros2_ws, 해당 위치로 이동
4. colcon build, 프로젝트 빌드
5. sb 명령어로 터미널 재실행

### .bashrc 설정
```
alias sb="source ~/.bashrc; echo \".bashrc is reloaded\""
alias ros_domain="export ROS_DOMAIN_ID=1"
alias humble="source /opt/ros/humble/setup.bash; ros_domain; echo \"ROS2 humble is activated.\""
alias ros2_ws="humble; source ~/ros2_ws/install/local_setup.bash; echo \"ros2_ws is activated.\""
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# === Pretty ROS2 Info ===
show_ros_info() {
  local ROS_DISTRO=${ROS_DISTRO:-"Not set"}
  local ROS_VERSION=${ROS_VERSION:-"?"}
  local PY_VER=$(python3 --version 2>/dev/null | awk '{print $2}')
  local HOST=$(hostname)
  local USER=$(whoami)
  local WS_PATH="~/ros2_ws"
  local DOMAIN_ID=${ROS_DOMAIN_ID:-"0 (default)"}

  echo -e "\n\033[1;36m┌────────────────────────────────────────────┐\033[0m"
  echo -e "  🤖 \033[1;32mROS2 Environment Loaded Successfully!\033[0m"
  echo -e "\033[1;36m├────────────────────────────────────────────┤\033[0m"
  echo -e "  🧭 \033[1;37mDistro      :\033[0m \033[1;33m$ROS_DISTRO\033[0m"
  echo -e "  ⚙️  \033[1;37mROS Version :\033[0m \033[1;33m$ROS_VERSION\033[0m"
  echo -e "  🐉 \033[1;37mPython Ver  :\033[0m \033[1;33m$PY_VER\033[0m"
  echo -e "  🌐 \033[1;37mDomain ID   :\033[0m \033[1;33m$DOMAIN_ID\033[0m"
  echo -e "  📂 \033[1;37mWorkspace   :\033[0m \033[1;33m$WS_PATH\033[0m"
  echo -e "  💻 \033[1;37mHost/User   :\033[0m \033[1;33m$HOST / $USER\033[0m"
  echo -e "\033[1;36m└────────────────────────────────────────────┘\033[0m\n"
}

# 터미널 열릴 때 자동 실행
ros2_ws
show_ros_info

```


### 로봇 실행
#### virtual
```
ros2 launch  dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m0609
```
#### real
```
ros2 launch  dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609
```