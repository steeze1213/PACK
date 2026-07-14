// 이 파일이 하는 일: 대시보드에서 자주 바꾸게 되는 "설정값"만 따로 모아둔 곳
// (ROS 접속 주소, 각 로봇의 HOME 좌표, 미션 좌표, 마커 ID, 맵 크기).
// 여기만 고치면 app.js 로직은 최대한 안 건드려도 됨.

// rosbridge 서버 주소. 로봇 PC가 바뀌면 여기 IP만 바꾸면 됨.
const ROSBRIDGE_URL = "ws://localhost:9090";

// =========================
// HOME 좌표
// =========================

// F1 로봇이 복귀할 HOME 좌표
// /clicked_point 1번 좌표 기준
// 방향: 아래쪽(yaw=-1.5708)
const F1_HOME = {
  name: "F1_HOME",
  x: 0.128,
  y: -1.112,
  yaw: -1.5708,
};

// F2 로봇이 복귀할 HOME 좌표
// /clicked_point 2번 좌표 기준
// 방향: 아래쪽(yaw=-1.5708)
const F2_HOME = {
  name: "F2_HOME",
  x: -0.048,
  y: -2.668,
  yaw: -1.5708,
};

// TB3(리더 = 터틀봇)의 HOME 좌표
// 기존 home_x, home_y, home_yaw 기준
const TB3_HOME = {
  name: "TB3_HOME",
  x: -0.037569766737571876,
  y: 0.10523828207830883,
  yaw: -0.007148110090032988,
};

// =========================
// 미션 좌표
// =========================

// 1번 적재 위치
// 역할:
// - 터틀봇이 첫 번째로 이동하는 적재 위치
// - 이 위치에서 로봇팔이 F1에 짐을 싣는다.
//
// TurtleBot3 /amcl_pose 기준으로 측정한 map 좌표
// 원본:
// x = 2.068099731050025
// y = -0.07009420567558194
// orientation z = 0.017484554994104
// orientation w = 0.9998471334842433
// yaw ≈ 0.034971 rad ≈ 2.0 deg
const LOAD_POINT_1 = {
  name: "LOAD_POINT_1",
  x: 2.068,
  y: -0.070,
  yaw: 0.035,
  qz: 0.017484554994104,
  qw: 0.9998471334842433,
};

// F1 적재 후 대기 위치
// 역할:
// - LOAD_POINT_1에서 짐을 실은 F1이 대열에서 이탈한 뒤 이동하는 좌표
// - F1이 바로 Home으로 가거나 다음 경로와 겹쳐 충돌하지 않도록 잠시 빠지는 공간
// - 터틀봇이나 F2가 가는 위치가 아니라, F1이 이동하는 위치
//
// TurtleBot3 /amcl_pose 기준으로 측정한 map 좌표
// 원본:
// x = 1.0907364828468926
// y = -1.1282804099283446
// orientation z = -0.6868610836458342
// orientation w = 0.7267887256781508
// yaw ≈ -1.514 rad ≈ -86.8 deg
const F1_WAIT_POINT_AFTER_LOAD_1 = {
  name: "F1_WAIT_POINT_AFTER_LOAD_1",
  x: 1.091,
  y: -1.128,
  yaw: -1.514,
  qz: -0.6868610836458342,
  qw: 0.7267887256781508,
};

// 2번 적재 위치
// 역할:
// - F1이 대열에서 빠진 뒤, 터틀봇과 F2가 이동하는 두 번째 적재 위치
// - 이 위치에서 로봇팔이 F2에 짐을 싣는다.
//
// TurtleBot3 /amcl_pose 기준으로 측정한 map 좌표
// 원본:
// x = 1.0881550059200746
// y = -2.5840552651613824
// yaw = -3.132554802973684
// yaw ≈ -179.5 deg
const LOAD_POINT_2 = {
  name: "LOAD_POINT_2",
  x: 1.088,
  y: -2.584,
  yaw: -3.133,
  qz: -0.9999897887805898,
  qw: 0.004519107716300417,
};

// 미션 좌표를 묶어서 관리
const MISSION_POINTS = {
  loadPoint1: LOAD_POINT_1,
  f1WaitAfterLoad1: F1_WAIT_POINT_AFTER_LOAD_1,
  loadPoint2: LOAD_POINT_2,
};

// 예전 이름 호환용.
// app.js에서 LOAD_POINTS를 쓰고 있다면 깨지지 않게 유지한다.
const LOAD_POINTS = {
  point1: LOAD_POINT_1,
  point2: LOAD_POINT_2,
  f1WaitAfterLoad1: F1_WAIT_POINT_AFTER_LOAD_1,
};

// =========================
// ArUco 추종 마커 기준
// =========================

// 터틀봇 뒤에 붙은 마커
// F1이 follow 모드에서 따라갈 대상
// F1이 대열에서 빠진 뒤에는 F2가 따라갈 대상
const TB3_REAR_MARKER_ID = 159;

// F1 뒤에 붙은 마커
// 초기 대열에서 F2가 follow 모드로 따라갈 대상
const F1_REAR_MARKER_ID = 158;

// 시나리오 기준:
// 초기:
//   F1 -> 159 추종
//   F2 -> 158 추종
//
// F1이 LOAD_POINT_1에서 짐을 싣고 이탈한 뒤:
//   F1 -> F1_WAIT_POINT_AFTER_LOAD_1 이동
//   F2 target 변경: 158 -> 159
//   F2 -> 159 추종
const FOLLOW_TARGETS = {
  f1FollowTB3: TB3_REAR_MARKER_ID,
  f2FollowF1: F1_REAR_MARKER_ID,
  f2FollowTB3: TB3_REAR_MARKER_ID,
};

// =========================
// 실제 맵 기준값
// =========================

// 실제 맵(미터 단위)을 화면 좌표(%)로 변환할 때 쓰는 기준값.
// 맵을 새로 만들면 이 값들을 맞춰줘야 점 위치가 정확해짐.
const mapConfig = {
  originX: -1.78, // 맵 왼쪽 아래 모서리의 실제 x (미터)
  originY: -5.08, // 맵 왼쪽 아래 모서리의 실제 y (미터)
  widthM: 5.15,   // 맵 가로 실제 길이 (미터)
  heightM: 6.30,  // 맵 세로 실제 길이 (미터)
};
