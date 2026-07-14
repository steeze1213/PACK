# PACK — 이동 피킹 연동 자율주행 운반 로봇

**로보테크 AI 자율주행로봇 개발자 과정 2기 · 최종 프로젝트 (4조 MYMY)**

> 물건을 집는 리더 한 대와 나르는 팔로워 두 대가 한 팀으로 움직인다.
> 리더(TurtleBot3 + 6축 로봇팔)가 자율주행으로 이동해 비전 피킹으로 물건을 집어 팔로워에 싣고,
> 짐을 실은 팔로워는 각자 지정된 자리까지 **스스로** 이동하는 다중 로봇 협업 시스템.

| 항목 | 내용 |
|------|------|
| 프로젝트명 | **PACK** (Precise Picking · Autonomous Arm · Collaborative Convoy · Kit) |
| 팀 | 4조 MYMY — 윤우영(팀장) · 길민준 · 김아영 · 안효민 / 멘토 이태화 |
| 기간 | 2026.05.15 ~ 07.16 |
| 개발 환경 | Ubuntu 22.04 · ROS2 Humble · Python · OpenCV · Arduino · Raspberry Pi |

---

## 기획

- **로봇팔은 비싸고, 컨베이어는 고정적이다.** 예산·공간이 부족한 소규모 현장일수록 자동화 진입장벽이 높다.
- 그래서 **고가 장비(로봇팔)는 1대만**, 운반은 **저가 팔로워 N대**가 나눠 맡는 구조를 택했다.
- 팔로워는 원격 조종이 아니라 **각자 벽면 마커로 자기 위치를 계산하는 개별 자율주행**으로 움직인다.

## 시스템 구성

| 구성 | 역할 | 주요 하드웨어 |
|------|------|--------------|
| **리더 (TB3)** | Nav2/A* 자율주행 + 비전 피킹 + 미션 지휘 | TurtleBot3 Waffle Pi, 자체 제작 6축 로봇팔(서보+PCA9685), Pi Camera(eye-in-hand) |
| **F1** | 리더 후면 마커(ID 159) 추종 운반 → 적재 후 지정 위치로 단독 이동 | Arduino 4WD + 쿼드러처 엔코더, Raspberry Pi, USB 카메라 |
| **F2** | F1 후면 마커(ID 158) 추종 → 재타겟 → **A\* 경로**로 접근·지정 위치 이동 | Arduino 4WD + 쿼드러처 엔코더, Raspberry Pi, USB 카메라 |
| **웹 대시보드** | 3대 위치·상태 실시간 관제, 미션 시작/정지 | Flask + rosbridge (브라우저가 로봇과 직접 통신) |

<p align="center">
  <img src="images/aruco_map_layout.png" width="500" alt="ArUco 마커 맵 배치">
  <br><sub>추종 차량 측위용 ArUco 마커 맵 — 벽면 마커의 부착 위치·정면 방향과 로봇 위치 시각화 (<code>map_pose_viewer_pc.py</code>)</sub>
</p>

## 미션 시나리오 (mission_manager 상태머신)

```
FORMATION_WAIT                 편대 구성 대기
→ GO_TO_LOAD_1                 리더 자율주행 → LOAD_POINT_1 (F1·F2 마커 추종)
→ (비전 피킹 → F1 적재)         load_done_f1 수신
→ F1_LOAD_EXIT_ROTATE          F1 제자리 회전 이탈
→ F1_LOAD_EXIT_SETTLE          정지·자세 안정
→ F1_TO_HOME                   F1 지정 위치(F1_HOME_POINT)로 단독 이동
→ F2_ASTAR_TO_LOAD2_APPROACH   F2가 A*로 2구역 접근점 이동
→ TB3_TO_LOAD2_WP1             리더 우측 벽 경유(RIGHT_WALL_WAYPOINT) → LOAD_POINT_2
→ F2_FINAL_MARKER_FOLLOW       F2 추종 대상 재타겟(마커 159) · 정밀 정렬
→ WAIT_LOAD_2                  2번째 피킹 → F2 적재 (load_done_f2)
→ HOME_RETURNING               리더는 홈 복귀 · F2는 경유점 4곳 거쳐 지정 위치 정지
→ MISSION_COMPLETE
```

## 핵심 기술

- **멀티마커 solvePnP 측위** — 벽면 ArUco 마커(0.05 m)를 여러 개 동시에 풀어
  각 로봇이 공용 좌표계 위 자기 좌표 (x, y, θ)를 계산. 바퀴 엔코더와 융합해 마커가 잠깐 안 보여도 위치 유지.
- **하이브리드 추종** — 앞차 후면 마커(159/158) 직접 추종을 우선하고, 마커를 놓치면 맵 기반 위치로 복구.
- **A\* + Pure Pursuit** — 정적 맵의 벽·장애물에 안전 비용(safety cost)을 얹어 최적 경로를 계획하고 추종 주행.
  F2의 접근·지정 위치 이동이 전부 이 방식.
- **비전 피킹 (IK)** — eye-in-hand 카메라로 박스의 색·위치 인식 → 픽셀 좌표를 로봇팔 기준 3D 좌표로 변환 →
  역기구학으로 관절 각도 계산 → 파지·적재.
- **MQTT 발행/구독** — Mosquitto 브로커 하나로 전 모듈 연결.
  미션 명령(대시보드 → 미션 매니저), 로봇팔 제어(`pack/arm/cmd` → 완료 시 `load_done` 회신), 로봇 상태 모니터링.
  서버 로직 없이 모듈끼리 직접 통신 — 한쪽이 꺼져도 재접속만으로 복구.
- **배타적 명령 구조** — 각 로봇은 mode manager / command mux를 통해 '한 번에 하나의 명령'만 수신 (안전·전압 강하 방지).

## 저장소 구조

```
PACK/
├── from_tb3/                     # 리더(TB3 라즈베리파이) 실행 코드
│   ├── multi_pick_v4.py          #   비전 피킹: 색 인식 → IK → 파지 → 적재 (3박스)
│   └── start_all.sh              #   리더 측 일괄 실행 스크립트
│
├── final_ws/                     # 리더 ROS2 워크스페이스
│   └── src/final_mission_robot/
│       ├── mission_manager.py            # 미션 상태머신 (시나리오 전체 지휘)
│       ├── air_clean_pure_controller.py  # A* + Pure Pursuit 주행 컨트롤러
│       ├── auto_initial_pose.py          # AMCL 초기 위치 자동 설정
│       └── config/final_params.yaml      # 노드·웨이포인트 파라미터
│
├── from_f1/encoder_bridge/       # F1(팔로워 1) ROS2 패키지
│   ├── encoder_bridge/
│   │   ├── aruco_node.py                 # 마커 검출 + 멀티마커 solvePnP 측위
│   │   ├── encoder_odom_node.py          # 엔코더 오도메트리
│   │   ├── relative_pose_node.py         # 마커·엔코더 융합 상대 위치
│   │   ├── f1_hybrid_follow_pose_node.py # 마커 우선 + 맵 보조 하이브리드 추종
│   │   ├── tb3_marker159_f1_pose_node.py # 리더 후면 마커(159) 추적
│   │   ├── f1_mode_manager_node.py       # 모드 관리 (단일 명령 보장)
│   │   └── waypoint_drive_node.py        # 웨이포인트 주행
│   ├── launch/f1_unified_system.launch.py
│   └── config/                           # 카메라 캘리브레이션 · 마커 맵
│
├── from_f2/encoder_bridge/       # F2(팔로워 2) ROS2 패키지 — F1 공통 노드 +
│   ├── encoder_bridge/
│   │   ├── f2_map_astar_planner_node.py  # 정적 맵 A* 경로 계획
│   │   ├── static_map_planner.py         # A* 구현 (안전 비용 포함)
│   │   ├── robot_cmd_mux_node.py         # 명령 mux (배타적 단일 명령)
│   │   └── tb3_marker159_f2_pose_node.py # 재타겟용 리더 마커 추적
│   ├── launch/f2_unified_system.launch.py
│   └── test/                             # A* 플래너 · 측위 게이팅 단위 테스트
│
├── robot_dashboard_flask/        # 웹 관제 대시보드
│   ├── app.py                    #   Flask 서버 (화면 제공)
│   ├── mission_manager.py        #   대시보드 측 미션 명령 처리
│   └── static/ · templates/      #   맵 뷰 · 로봇 상태 카드 · 미션 제어 UI
│
├── RobotArmCase.ino              # 로봇팔 펌웨어 (Arduino + PCA9685, 안전범위·FK 충돌가드·비상정지)
├── mqtt_gateway_lite.py          # MQTT → 시리얼 게이트웨이 (팔 무선 제어)
├── auto_pick.py                  # 피킹 단독 실행 버전
├── f1_car.ino                    # 4WD 팔로워 펌웨어 (주행 + 엔코더 발행)
├── map_pose_viewer_pc.py         # 마커 맵 · 로봇 위치 시각화 (PC 모니터링)
└── images/                       # 문서용 이미지
```

## 실행 개요

```bash
# 공통 — 관제 PC: Mosquitto 브로커 실행 (기본 포트 1883)

# 리더 (TB3 라즈베리파이)
cd final_ws && colcon build && source install/setup.bash
ros2 launch final_mission_robot final_mission.launch.py   # 미션 매니저 + 주행
./from_tb3/start_all.sh                                   # 피킹 등 일괄 실행

# 로봇팔 (팔 제어용 라즈베리파이)
python3 mqtt_gateway_lite.py        # MQTT 명령 → 시리얼(아두이노) 중계

# F1 / F2 (각 팔로워 라즈베리파이)
ros2 launch encoder_bridge f1_unified_system.launch.py
ros2 launch encoder_bridge f2_unified_system.launch.py

# 대시보드 (관제 PC)
cd robot_dashboard_flask && pip install -r requirements.txt && ./run.sh
# 브라우저 접속 → 미션 시작
```

## 팀 구성 · 역할

| 팀원 | 주 담당 | 보조 |
|------|---------|------|
| **윤우영** (팀장) | 리더 자율주행 · 미션 상태머신 | 통합 시나리오 |
| **길민준** | 로봇팔 · 비전 피킹 · 역기구학(IK) | 측위 연동 |
| **김아영** | 마커 맵 · 측위 융합 · 추종 | 카메라 캘리브레이션 |
| **안효민** | 통신 · 웹 대시보드 · 통합 | 통합 테스트 |

## 향후 발전 방향

추종 방식 파라미터화(마커/SLAM/라인 선택형) · 팔로워 N대 확장과 고장 시 순번 자동 대체 ·
인계·적재/하역 완전 자동화 · 라이다 기반 동적 장애물 대응 · 작업 모듈 교체로 물류/농업/병원 범용화
