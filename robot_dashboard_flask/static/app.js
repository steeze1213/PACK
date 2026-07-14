// 이 파일이 하는 일: 대시보드의 모든 동작(ROS 연결, 토픽 발행/구독, 버튼 클릭, 페이지 전환)을 담당.
// 설정값(주소/HOME/맵 크기)은 config.js에서 가져다 씀 (index.html에서 config.js를 먼저 로드).

// ros: rosbridge 연결 객체 / topics: 자주 쓰는 발행용 토픽들을 모아두는 곳
let ros = null;
let topics = {};
let rosUrl = ROSBRIDGE_URL; // 설정 화면에서 바꾸면 이 값이 갱신됨
let selectedRobot = null;
let f1Mode = "paused";
let f2Mode = "paused";

// 드롭다운에서 선택된 미션 명령.
// 기본값은 start라서 아무것도 안 바꾸고 Mission Start 버튼을 누르면 기존처럼 시작됨.
let selectedMissionCommand = "start";

// ===== 사이드바: 섹션 이동(home/map/robots) + 드로어(log/settings) =====

function showSection(name, el) {
  setNavActive(el);
  document.querySelector(".app").scrollTo({ top: 0, behavior: "smooth" });
}

function openDrawer(name, el) {
  closeDrawer();
  const drawer = document.getElementById("drawer-" + name);
  if (drawer) drawer.classList.add("open");
  document.getElementById("backdrop").classList.add("show");
  if (el) el.classList.add("nav-open");
}

function closeDrawer() {
  document.querySelectorAll(".drawer").forEach((d) => d.classList.remove("open"));
  document.getElementById("backdrop").classList.remove("show");
  document.querySelectorAll(".nav-item").forEach((n) => n.classList.remove("nav-open"));
}

function setNavActive(el) {
  document.querySelectorAll(".nav-item").forEach((n) => n.classList.remove("active"));
  if (el) el.classList.add("active");
}

document.addEventListener("keydown", (e) => {
  if (e.key === "Escape") closeDrawer();
});

function addLog(message, type = "") {
  const logBox = document.getElementById("log");
  if (!logBox) return;
  const line = document.createElement("div");
  line.className = "log-line " + type;
  const time = new Date().toLocaleTimeString();
  line.textContent = `[${time}] ${message}`;
  logBox.prepend(line);
}

function connectROS() {
  ros = new ROSLIB.Ros({ url: rosUrl });

  ros.on("connection", () => {
    setStatus("ROSBridge Connected", true);
    addLog("ROSBridge connected (" + rosUrl + ")", "ok");
    setupTopics();
  });

  ros.on("error", (error) => {
    setStatus("ROSBridge Error", false);
    addLog("ROSBridge error: " + error, "danger");
  });

  ros.on("close", () => {
    setStatus("ROSBridge Disconnected", false);
    addLog("ROSBridge disconnected", "warning");
  });
}

function reconnectROS() {
  const input = document.getElementById("ros-url");
  if (input && input.value.trim()) rosUrl = input.value.trim();
  addLog("Reconnecting to " + rosUrl + " ...", "warning");
  try {
    if (ros) ros.close();
  } catch (e) {}
  connectROS();
}

function setStatus(text, connected) {
  const el = document.getElementById("ros-status");
  if (!el) return;
  el.textContent = text;
  el.classList.toggle("connected", connected);
}

function setupTopics() {
  // --- 팔로워 F1 / F2 ---
  topics.f2FollowEnable = makeTopic("/f2/follow_enable");
  topics.f2WaypointEnable = makeTopic("/f2/waypoint_enable");

  topics.f1GoalPose = makeTopic("/f1/goal_pose", "geometry_msgs/PoseStamped");
  topics.f2GoalPose = makeTopic("/f2/goal_pose", "geometry_msgs/PoseStamped");

  topics.f2RobotCmd = makeTopic("/f2/robot_cmd");

  topics.f1Mode = makeTopic("/f1/mode");
  topics.f2Mode = makeTopic("/f2/mode");

  topics.f1LoadWait = makeTopic("/f1/load_wait");
  topics.f1LoadDone = makeTopic("/f1/load_done");
  topics.f1Reanchor = makeTopic("/f1/reanchor");

  topics.f2LoadWait = makeTopic("/f2/load_wait");
  topics.f2LoadDone = makeTopic("/f2/load_done");
  topics.f2ReturnHome = makeTopic("/f2/return_home");
  topics.f2Reanchor = makeTopic("/f2/reanchor");

  topics.missionCmd = makeTopic("/mission_cmd");
  topics.armCmd = makeTopic("/arm_cmd");

  // --- 리더 TB3(터틀봇) ---
  topics.tb3MissionEnable = makeTopic("/tb3/mission_enable");
  topics.tb3WaypointEnable = makeTopic("/tb3/waypoint_enable");
  topics.tb3GoalPose = makeTopic("/tb3/goal_pose");
  topics.tb3RobotCmd = makeTopic("/tb3/robot_cmd");

  subscribeStringTopic("/f1/relative_pose", handleF1Pose);
  subscribeStringTopic("/f2/relative_pose", handleF2Pose);

  subscribeStringTopic("/tb3/relative_pose", handleTB3Pose);
  subscribePoseWithCovarianceTopic("/amcl_pose", handleTB3AmclPose);

  subscribeStringTopic("/f1/cmd_mux_status", handleF1Mux);
  subscribeStringTopic("/f2/cmd_mux_status", handleF2Mux);
  subscribeStringTopic("/tb3/cmd_mux_status", handleTB3Mux);

  subscribeStringTopic("/f1/waypoint_status", (msg) => addLog("F1 waypoint: " + msg.data));
  subscribeStringTopic("/f2/waypoint_status", (msg) => addLog("F2 waypoint: " + msg.data));
  subscribeStringTopic("/tb3/waypoint_status", (msg) => addLog("TB3 waypoint: " + msg.data));

  subscribeStringTopic("/f1/follow_status", (msg) => addLog("F1 follow: " + msg.data));
  subscribeStringTopic("/f2/follow_status", (msg) => addLog("F2 follow: " + msg.data));

  subscribeStringTopic("/f1/hybrid_status", (msg) => handleHybrid("f1", msg));
  subscribeStringTopic("/f1/mode_status", handleF1ModeStatus);

  subscribeStringTopic("/f2/hybrid_status", (msg) => handleHybrid("f2", msg));
  subscribeStringTopic("/f2/mode_status", handleF2ModeStatus);

  subscribeStringTopic("/mission_status", (msg) => addLog("Mission: " + msg.data));
  subscribeStringTopic("/tb3/mission_status", (msg) => addLog("TB3 mission: " + msg.data));
}

function makeTopic(name, messageType = "std_msgs/String") {
  return new ROSLIB.Topic({
    ros: ros,
    name: name,
    messageType: messageType,
  });
}

function subscribeStringTopic(name, callback) {
  makeTopic(name).subscribe(callback);
}

function subscribePoseWithCovarianceTopic(name, callback) {
  new ROSLIB.Topic({
    ros: ros,
    name: name,
    messageType: "geometry_msgs/PoseWithCovarianceStamped",
  }).subscribe(callback);
}

function publishString(topic, data) {
  if (!topic) {
    addLog("Topic not ready", "danger");
    return;
  }
  topic.publish(new ROSLIB.Message({ data: data }));
}

function publishF1Mode(mode, source) {
  f1Mode = mode;
  console.log("[F1_MODE_PUBLISH]", mode, "from", source);
  addLog("F1 mode publish: " + mode + " from " + source, "warning");
  publishString(topics.f1Mode, mode);
}

function publishF2Mode(mode, source) {
  f2Mode = mode;
  console.log("[F2_MODE_PUBLISH]", mode, "from", source);
  addLog("F2 mode publish: " + mode + " from " + source, "warning");
  publishString(topics.f2Mode, mode);
}

// ===== 버튼 동작들 =====

function startFollow(robot) {
  if (robot === "f1") {
    selectedRobot = "f1";
    publishF1Mode("follow", "startFollow");
    addLog("F1 FOLLOW start", "ok");
  }

  if (robot === "f2") {
    selectedRobot = "f2";
    publishF2Mode("follow", "startFollow");
    addLog("F2 FOLLOW start", "ok");
  }
}

function leadStart(robot) {
  if (robot === "tb3") {
    publishString(topics.tb3WaypointEnable, "stop");
    publishString(topics.tb3MissionEnable, "start");
    addLog("TB3 LEAD start", "ok");
  }
}

function startWaypoint(robot) {
  if (robot === "f1") {
    selectedRobot = "f1";
    publishF1Mode("waypoint", "startWaypoint");
    addLog("F1 WAYPOINT selected; click the map to set a goal", "ok");
  }

  if (robot === "f2") {
    selectedRobot = "f2";
    publishF2Mode("waypoint", "startWaypoint");
    addLog("F2 WAYPOINT selected; click the map to set a goal", "ok");
  }

  if (robot === "tb3") {
    publishString(topics.tb3MissionEnable, "stop");
    publishString(topics.tb3WaypointEnable, "start");
    addLog("TB3 WAYPOINT start", "ok");
  }
}

function stopRobot(robot) {
  if (robot === "f1") {
    publishF1Mode("stop", "stopRobot");
    addLog("F1 STOP", "warning");
  }

  if (robot === "f2") {
    publishF2Mode("stop", "stopRobot");
    publishString(topics.f2RobotCmd, "s");
    addLog("F2 STOP", "warning");
  }

  if (robot === "tb3") {
    publishString(topics.tb3MissionEnable, "stop");
    publishString(topics.tb3WaypointEnable, "stop");
    publishString(topics.tb3RobotCmd, "s");
    addLog("TB3 STOP", "warning");
  }
}

function allStop() {
  publishString(topics.missionCmd, "emergency_stop");
  stopRobot("tb3");
  stopRobot("f1");
  stopRobot("f2");
  publishString(topics.armCmd, "stop");
  addLog("ALL STOP", "danger");
}

// ===== Mission command dropdown =====
// 드롭다운에서 명령만 선택하고, Mission Start 버튼을 눌렀을 때 선택된 명령을 전송한다.

const MISSION_COMMANDS = [
  { label: "미션 시작", value: "start" },
  { label: "F1 적재 완료", value: "load_done_f1" },
  { label: "F1 홈 도착 완료", value: "f1_home_done" },
  { label: "F2 159 추종 시작", value: "follow_159" },
  { label: "F2 적재 완료", value: "load_done_f2" },
  { label: "홈 복귀 완료", value: "home_done" },
  { label: "일시정지", value: "pause" },
  { label: "긴급정지", value: "emergency_stop" },
];

function ensureMissionCommandDropdown() {
  if (document.getElementById("mission-command-dropdown")) {
    return;
  }

  const buttons = Array.from(document.querySelectorAll("button"));
  const missionStartButton = buttons.find((button) => {
    return button.textContent.trim().toLowerCase() === "mission start";
  });

  if (!missionStartButton) {
    addLog("Mission command dropdown create failed: Mission Start button not found", "warning");
    return;
  }

  const select = document.createElement("select");
  select.id = "mission-command-dropdown";
  select.title = "Select mission command";

  select.style.height = "36px";
  select.style.marginLeft = "8px";
  select.style.padding = "0 10px";
  select.style.border = "1px solid #cbd5e1";
  select.style.borderRadius = "10px";
  select.style.background = "#ffffff";
  select.style.color = "#1e293b";
  select.style.fontSize = "13px";
  select.style.fontWeight = "700";
  select.style.outline = "none";
  select.style.cursor = "pointer";

  MISSION_COMMANDS.forEach((item) => {
    const option = document.createElement("option");
    option.value = item.value;
    option.textContent = item.label;
    select.appendChild(option);
  });

  select.value = selectedMissionCommand;

  select.addEventListener("change", () => {
    selectedMissionCommand = select.value || "start";
    addLog("Mission command selected: " + selectedMissionCommand, "warning");
  });

  missionStartButton.insertAdjacentElement("afterend", select);

  addLog("Mission command dropdown added next to Mission Start", "ok");
}

function publishMissionCommand(cmd, source = "manual") {
  if (!cmd) {
    return;
  }

  publishString(topics.missionCmd, cmd);
  addLog("Mission command publish: " + cmd + " from " + source, "ok");
}

// HOME: config의 HOME 좌표로 goal을 보내고, 잠시 뒤 waypoint 주행을 켜서 이동시킴
const HOME_COORDS = { f1: F1_HOME, f2: F2_HOME, tb3: TB3_HOME };

function goHome(robot) {
  const home = HOME_COORDS[robot];
  if (!home) return;

  const goal = `GOAL,x=${home.x.toFixed(3)},y=${home.y.toFixed(3)},yaw=${home.yaw.toFixed(3)}`;

  if (robot === "f1") {
    selectedRobot = "f1";
    publishF1Mode("return_home", "goHome");
  } else if (robot === "f2") {
    selectedRobot = "f2";
    publishF2Mode("return_home", "goHome");
  } else if (robot === "tb3") {
    publishString(topics.tb3MissionEnable, "stop");
    publishString(topics.tb3GoalPose, goal);
    setTimeout(() => publishString(topics.tb3WaypointEnable, "start"), 300);
  }

  addLog(robot.toUpperCase() + " HOME goal published: " + goal, "ok");
}

// Mission Start 버튼은 이제 "선택된 미션 명령 전송 버튼" 역할.
// 드롭다운이 Mission Start이면 start 전송.
// 드롭다운이 F1 Load Done이면 load_done_f1 전송.
function missionStart() {
  const cmd = selectedMissionCommand || "start";

  publishMissionCommand(cmd, "Mission Start button");

  if (cmd === "start") {
    publishString(topics.tb3WaypointEnable, "stop");
    publishString(topics.tb3MissionEnable, "start");

    publishF1Mode("follow", "missionStart");
    publishF2Mode("follow", "missionStart");

    addLog("Mission start: TB3 lead + F1/F2 follow", "ok");
    return;
  }

  addLog("Mission command sent by Mission Start button: " + cmd, "ok");
}

function missionPause() {
  publishString(topics.missionCmd, "pause");

  publishString(topics.tb3MissionEnable, "stop");
  publishString(topics.tb3WaypointEnable, "stop");

  publishF1Mode("paused", "missionPause");
  publishF2Mode("paused", "missionPause");

  addLog("Mission pause", "warning");
}

// ===== 수신 데이터 파싱/표시 =====

function parseKeyValue(data) {
  const result = {};
  const parts = data.split(",");

  for (let i = 1; i < parts.length; i++) {
    const part = parts[i];
    if (!part.includes("=")) continue;

    const pair = part.split("=");
    const key = pair[0].trim();
    const value = pair.slice(1).join("=").trim();
    const num = Number(value);

    result[key] = Number.isNaN(num) ? value : num;
  }

  return result;
}

function handleF1Pose(msg) {
  updateRobotPose("f1", parseKeyValue(msg.data));
}

function handleF2Pose(msg) {
  updateRobotPose("f2", parseKeyValue(msg.data));
}

function handleTB3Pose(msg) {
  updateRobotPose("tb3", parseKeyValue(msg.data));
}

function handleTB3AmclPose(msg) {
  if (!msg || !msg.pose || !msg.pose.pose) return;

  const position = msg.pose.pose.position;
  const orientation = msg.pose.pose.orientation;

  const x = Number(position.x || 0);
  const y = Number(position.y || 0);

  const qx = Number(orientation.x || 0);
  const qy = Number(orientation.y || 0);
  const qz = Number(orientation.z || 0);
  const qw = Number(orientation.w || 1);

  const sinyCosp = 2.0 * (qw * qz + qx * qy);
  const cosyCosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  const yaw = Math.atan2(sinyCosp, cosyCosp);

  updateRobotPose("tb3", {
    x: x,
    y: y,
    yaw: yaw,
    marker_id: "AMCL",
  });
}

function handleMux(robot, msg) {
  const p = parseKeyValue(msg.data);

  if (robot === "f1") {
    if (p.cmd !== undefined) setText("f1-cmd", p.cmd);
    return;
  }

  if (p.mode !== undefined) setText(`${robot}-mode`, p.mode);
  if (p.cmd !== undefined) setText(`${robot}-cmd`, p.cmd);
}

function handleF1Mux(msg) {
  handleMux("f1", msg);
}

function handleF2Mux(msg) {
  handleMux("f2", msg);
}

function handleTB3Mux(msg) {
  handleMux("tb3", msg);
}

function handleHybrid(robot, msg) {
  const p = parseKeyValue(msg.data);

  if (robot === "f1") {
    if (p.state !== undefined) setText("f1-hybrid-state", p.state);
    if (p.cmd !== undefined) setText("f1-cmd", p.cmd);
    return;
  }

  if (p.state !== undefined) setText(`${robot}-mode`, p.state);
  if (p.cmd !== undefined) setText(`${robot}-cmd`, p.cmd);
}

function handleF1ModeStatus(msg) {
  const p = parseKeyValue(msg.data);

  if (p.mode !== undefined) {
    f1Mode = String(p.mode);
    setText("f1-mode", f1Mode);
  }

  if (p.cmd !== undefined) {
    setText("f1-cmd", p.cmd);
  }
}

function handleF2ModeStatus(msg) {
  const p = parseKeyValue(msg.data);

  if (p.mode !== undefined) {
    f2Mode = String(p.mode);
    setText("f2-mode", f2Mode);
  }

  if (p.cmd !== undefined) {
    setText("f2-cmd", p.cmd);
  }
}

// ===== 지도 goal 표시 =====

function ensureGoalMarker(robot) {
  const map = document.getElementById("map");
  if (!map) return null;

  let marker = document.getElementById(`goal-${robot}`);
  if (marker) return marker;

  marker = document.createElement("div");
  marker.id = `goal-${robot}`;
  marker.textContent = "×";
  marker.style.position = "absolute";
  marker.style.transform = "translate(-50%, -50%)";
  marker.style.fontSize = "30px";
  marker.style.fontWeight = "900";
  marker.style.zIndex = "30";
  marker.style.pointerEvents = "none";
  marker.style.lineHeight = "1";
  marker.style.color = "red";
  marker.title = `${robot.toUpperCase()} goal`;

  map.appendChild(marker);
  return marker;
}

function updateGoalMarker(robot, x, y) {
  const marker = ensureGoalMarker(robot);
  if (!marker) return;

  const pos = mapToScreen(x, y);
  marker.style.left = pos.left + "%";
  marker.style.top = pos.top + "%";
}

function publishMapGoal(event) {
  let robot = null;
  let mode = "";
  let screenMode = "";

  if (selectedRobot === "f1") {
    robot = "f1";
    mode = f1Mode;
    screenMode = document.getElementById("f1-mode")
      ? document.getElementById("f1-mode").textContent.trim()
      : "";
  } else if (selectedRobot === "f2") {
    robot = "f2";
    mode = f2Mode;
    screenMode = document.getElementById("f2-mode")
      ? document.getElementById("f2-mode").textContent.trim()
      : "";
  }

  const isWaypointMode =
    robot !== null &&
    (mode === "waypoint" || screenMode === "waypoint");

  if (!isWaypointMode) {
    addLog(
      "Map click ignored: selected=" +
        selectedRobot +
        ", f1Mode=" +
        f1Mode +
        ", f2Mode=" +
        f2Mode +
        ", screenMode=" +
        screenMode,
      "warning"
    );
    return;
  }

  const goalTopic = robot === "f1" ? topics.f1GoalPose : topics.f2GoalPose;

  if (!goalTopic) {
    addLog(robot.toUpperCase() + " goal topic is not ready", "danger");
    return;
  }

  const map = document.getElementById("map");
  if (!map) {
    addLog("Map element not found", "danger");
    return;
  }

  const rect = map.getBoundingClientRect();
  const ratioX = Math.max(0, Math.min(1, (event.clientX - rect.left) / rect.width));
  const ratioY = Math.max(0, Math.min(1, (event.clientY - rect.top) / rect.height));

  const x = mapConfig.originX + ratioX * mapConfig.widthM;
  const y = mapConfig.originY + (1 - ratioY) * mapConfig.heightM;
  const nowMs = Date.now();

  updateGoalMarker(robot, x, y);

  goalTopic.publish(
    new ROSLIB.Message({
      header: {
        stamp: {
          sec: Math.floor(nowMs / 1000),
          nanosec: (nowMs % 1000) * 1000000,
        },
        frame_id: "map",
      },
      pose: {
        position: { x: x, y: y, z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
      },
    })
  );

  addLog(`${robot.toUpperCase()} waypoint goal: x=${x.toFixed(3)}, y=${y.toFixed(3)}`, "ok");
}

function setText(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

function updateRobotPose(robot, pose) {
  const x = Number(pose.x || 0);
  const y = Number(pose.y || 0);
  const yaw = Number(pose.yaw || 0);
  const marker = pose.marker_id !== undefined ? pose.marker_id : "-";

  setText(`${robot}-x`, x.toFixed(3));
  setText(`${robot}-y`, y.toFixed(3));
  setText(`${robot}-yaw`, (yaw * 180 / Math.PI).toFixed(1) + "°");
  setText(`${robot}-marker`, marker);

  updateMapDot(robot, x, y);
}

function updateMapDot(robot, x, y) {
  const pos = mapToScreen(x, y);

  const dot = document.getElementById(`dot-${robot}`);
  const label = document.getElementById(`label-${robot}`);

  if (!dot || !label) return;

  dot.style.left = pos.left + "%";
  dot.style.top = pos.top + "%";
  label.style.left = pos.left + "%";
  label.style.top = pos.top + "%";
}

function mapToScreen(x, y) {
  let left = ((x - mapConfig.originX) / mapConfig.widthM) * 100;
  let top = 100 - ((y - mapConfig.originY) / mapConfig.heightM) * 100;

  left = Math.max(0, Math.min(100, left));
  top = Math.max(0, Math.min(100, top));

  return { left: left, top: top };
}

// ===== 설정 화면 초기값 채우기 =====

function initSettings() {
  const urlInput = document.getElementById("ros-url");
  if (urlInput) urlInput.value = rosUrl;

  const box = document.getElementById("home-config");
  if (box) {
    const rows = [
      ["TB3 Home", TB3_HOME],
      ["F1 Home", F1_HOME],
      ["F2 Home", F2_HOME],
    ];

    box.innerHTML = rows
      .map(
        ([n, h]) =>
          `<div class="status-row"><span class="k">${n}</span><span class="v">x ${h.x}, y ${h.y}, yaw ${h.yaw}</span></div>`
      )
      .join("");
  }

  const mapBox = document.getElementById("map-config");
  if (mapBox) {
    const rows = [
      ["Origin", `x ${mapConfig.originX}, y ${mapConfig.originY}`],
      ["Size", `${mapConfig.widthM} × ${mapConfig.heightM} m`],
      ["Image", "map.png (map_cleaned)"],
    ];

    mapBox.innerHTML = rows
      .map(
        ([k, v]) =>
          `<div class="status-row"><span class="k">${k}</span><span class="v">${v}</span></div>`
      )
      .join("");
  }
}

function bindMapClick() {
  const mapElement = document.getElementById("map");

  if (!mapElement) {
    addLog("Map click bind failed: #map not found", "danger");
    console.error("[MAP_CLICK] #map not found");
    return;
  }

  mapElement.style.cursor = "crosshair";

  mapElement.addEventListener("click", (event) => {
    console.log("[MAP_CLICK]", {
      selectedRobot: selectedRobot,
      f1Mode: f1Mode,
      f2Mode: f2Mode,
      targetId: event.target ? event.target.id : null,
      targetClass: event.target ? event.target.className : null,
    });

    publishMapGoal(event);
  });

  addLog("Map click handler bound to #map", "ok");
  console.log("[MAP_CLICK] handler bound to #map");
}

// 페이지가 열리면 설정값을 채우고 자동으로 ROS 연결 시작
initSettings();
bindMapClick();
ensureMissionCommandDropdown();
connectROS();
