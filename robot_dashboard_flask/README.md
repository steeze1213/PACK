# Robot Dashboard Flask

ROS2 로봇 관제 대시보드를 Flask 서버로 실행하는 버전입니다.

## 실행 방법

```bash
cd robot_dashboard_flask
pip3 install -r requirements.txt
python3 app.py
```

브라우저에서 접속합니다.

```text
http://localhost:5000
```

다른 PC에서 접속할 때는 서버 PC의 IP를 사용합니다.

```text
http://서버PC_IP:5000
```

## rosbridge 실행

대시보드가 ROS2 토픽과 통신하려면 rosbridge가 켜져 있어야 합니다.

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

최종 시연 ROS 그래프는 별도 터미널에서 실행합니다. 기존 F1/F2 테스트
launch와 동시에 실행하면 안 됩니다.

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
ros2 launch encoder_bridge final_mission_system.launch.py
```

대시보드는 모터나 시리얼 장치를 직접 제어하지 않고 ROS2 String 토픽만
publish합니다. 비상정지는 `/f1/robot_cmd=s`, `/f2/robot_cmd=s`,
`/arm_cmd=stop`과 컨트롤러 disable 명령을 함께 보냅니다.

## ROSBridge 주소 수정

`static/config.js`에서 아래 값을 수정합니다.

```js
const ROSBRIDGE_URL = "ws://localhost:9090";
```

다른 PC 브라우저에서 대시보드를 열 경우, `localhost`는 브라우저를 실행한 PC 자신을 의미합니다.
따라서 로봇 PC의 IP로 바꿔야 합니다.

```js
const ROSBRIDGE_URL = "ws://로봇PC_IP:9090";
```

## 파일 구조

```text
robot_dashboard_flask/
├─ app.py
├─ requirements.txt
├─ run.sh
├─ templates/
│  └─ index.html
├─ static/
│  ├─ style.css
│  ├─ config.js
│  ├─ app.js
│  └─ map.png
└─ docs/
   └─ dashboard_screen.docx
```
