#!/bin/bash
source /opt/ros/humble/setup.bash
mkdir -p ~/logs
echo "[0/3] 기존 프로세스 정리..."
pkill -9 -f camera_node
pkill -f turtlebot3
pkill -f mqtt_gateway_lite
sleep 3
echo "[1/3] 카메라 시작..."
nohup ros2 run camera_ros camera_node --ros-args -p camera:=1 -p width:=1640 -p height:=1232 -p format:=RGB888 > ~/logs/cam.log 2>&1 &
sleep 4
if grep -q "failed to acquire" ~/logs/cam.log; then
  echo "!! 카메라 획득 실패 — 재시도"
  pkill -9 -f camera_node
  sleep 4
  nohup ros2 run camera_ros camera_node --ros-args -p camera:=1 -p width:=1640 -p height:=1232 -p format:=RGB888 > ~/logs/cam.log 2>&1 &
  sleep 4
fi
echo "[2/3] bringup 시작..."
nohup ros2 launch turtlebot3_bringup robot.launch.py usb_port:=/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00 > ~/logs/bringup.log 2>&1 &
echo "[3/3] 게이트웨이 시작... (팔 보정 동작 주의)"
nohup python3 ~/mqtt_gateway_lite.py /dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_1344B43523435121D616-if00 localhost > ~/logs/arm.log 2>&1 &
sleep 8
if grep -q "failed to acquire" ~/logs/cam.log; then
  echo "!! 카메라 비정상 — tail ~/logs/cam.log 확인 필요"
else
  echo "=== 기동 완료. mission 실행 가능 ==="
fi
