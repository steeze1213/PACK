"""multi_pick_v4.py — 3박스 연속 시퀀스 (자리별 CH15 + cx정렬 + orange_blue IK).

auto_pick_new.py 의 단일 잡기 흐름을 자리별로 반복:
  for box in order (default LCR):
    1. 검출 자세 (자리별 시작 CH15)
    2. box_detect_3d cx 정렬
    3. orange_blue_detect → 박스 중심 (베이스)
    4. box_front = 중심 + 깊이/2 → IK (grip_z, pitch)
    5. 자세 도달 + 그리퍼 닫기
    6. 후방 놓기

사용:
  python3 multi_pick_v4.py
  python3 multi_pick_v4.py --order LCR
  python3 multi_pick_v4.py --skip-place
  python3 multi_pick_v4.py --no-pick
"""

import argparse
import re
import subprocess
import time

import numpy as np
import yaml

from arm_ik import ik_safe


DETECT_POSE = dict(ch15=47, ch13=80, ch11=45, ch9=90, ch7=130)

BOX_START_CH15 = {
    "C": 47,
    "L": 58,
    "R": 33,
}

EXPECTED_Y = {
    "C": 0.0,
    "L": 32.0,
    "R": -55.0,
}

Y_TOL = 25.0

BOX_DEPTH_HALF = 40.0
GRIP_Z = -50.0
TCP_PITCH = -42.0

SAFE_SHOULDER = 90
PLACE_BASE = 150

GRIP_CLOSE = 20
GRIP_OPEN = 0

MQTT_TOPIC = "pack/arm/cmd"

IK_MODE = "maxch9"


def ik_auto(x, y, z, pitch_pref=-42.0, pitch_steep=-55.0, pitch_flat=-15.0, step=0.5):
    """IK 후보 중 도달 가능한 자세를 찾는다.

    IK_MODE='maxch9':
      도달 가능한 후보 중 CH9가 가장 큰 자세 선택.
      손목을 최대한 펴서 reach를 확보한다.

    IK_MODE='steep':
      pitch_pref부터 시작해서 점점 눕히며 첫 성공 후보 선택.

    반환:
      (pose, used_pitch)
    """
    if IK_MODE == "steep":
        tp = pitch_pref
        while tp <= pitch_flat + 1e-6:
            p, _ = ik_safe(x, y, z, tp)
            if p is not None:
                return p, tp
            tp += step
        return None, pitch_pref

    best = None

    tp = pitch_steep
    while tp <= pitch_flat + 1e-6:
        p, _ = ik_safe(x, y, z, tp)
        if p is not None:
            if best is None or p.ch9 > best[0]:
                best = (p.ch9, p, tp)
        tp += step

    if best is None:
        return None, pitch_pref

    return best[1], best[2]


def mqtt(cmd, dwell=0.0):
    subprocess.run(
        [
            "mosquitto_pub",
            "-h",
            "localhost",
            "-t",
            MQTT_TOPIC,
            "-m",
            cmd,
        ],
        check=True,
    )

    print(f"  → {cmd}")

    if dwell > 0:
        time.sleep(dwell)


def run_box(ch15, target_cx):
    res = subprocess.run(
        [
            "python3",
            "/home/group4/box_detect_3d.py",
            "--ch15",
            str(ch15),
            "--ch13",
            str(DETECT_POSE["ch13"]),
            "--ch11",
            str(DETECT_POSE["ch11"]),
            "--ch9",
            str(DETECT_POSE["ch9"]),
            "--ch7",
            str(DETECT_POSE["ch7"]),
            "--target-cx",
            str(target_cx),
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )

    if res.returncode != 0:
        print("  box_detect_3d 실패")
        if res.stdout.strip():
            print(res.stdout.strip())
        if res.stderr.strip():
            print(res.stderr.strip())
        return None

    m_cx = re.search(r"cx=([\-\d.]+)", res.stdout)

    if not m_cx:
        print("  box_detect_3d 출력에서 cx 파싱 실패")
        if res.stdout.strip():
            print(res.stdout.strip())
        if res.stderr.strip():
            print(res.stderr.strip())
        return None

    return float(m_cx.group(1))


def run_orange_blue(ch15, anchor_cx):
    res = subprocess.run(
        [
            "python3",
            "/home/group4/orange_blue_detect.py",
            "--ch15",
            str(ch15),
            "--ch13",
            str(DETECT_POSE["ch13"]),
            "--ch11",
            str(DETECT_POSE["ch11"]),
            "--ch9",
            str(DETECT_POSE["ch9"]),
            "--ch7",
            str(DETECT_POSE["ch7"]),
            "--anchor-cx",
            str(anchor_cx),
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )

    if res.returncode != 0:
        print("  orange_blue_detect 실패")
        if res.stdout.strip():
            print(res.stdout.strip())
        if res.stderr.strip():
            print(res.stderr.strip())
        return None

    m_x = re.search(r"x\s*=\s*([\-\d.]+),\s*y\s*=\s*([\-\d.]+)", res.stdout)

    if not m_x:
        print("  orange_blue_detect 출력에서 x/y 파싱 실패")
        if res.stdout.strip():
            print(res.stdout.strip())
        if res.stderr.strip():
            print(res.stderr.strip())
        return None

    return float(m_x.group(1)), float(m_x.group(2))


def move_detect_pose(start_ch15):
    # 박스마다 R 리셋을 치면 팔이 홈/수직 자세로 크게 들릴 수 있어서 제거.
    mqtt(f"5 {GRIP_OPEN}", 0.6)

    mqtt(f"13 {SAFE_SHOULDER}", 2.0)
    mqtt(f"15 {start_ch15}", 3.0)
    mqtt(f"11 {DETECT_POSE['ch11']}", 2.0)
    mqtt(f"7 {DETECT_POSE['ch7']}", 1.0)
    mqtt(f"9 {DETECT_POSE['ch9']}", 2.5)
    mqtt(f"13 {DETECT_POSE['ch13']}", 4.0)

    # 오픈루프 서보 정착용 재발행
    mqtt(f"11 {DETECT_POSE['ch11']}", 1.5)
    mqtt(f"9 {DETECT_POSE['ch9']}", 2.0)
    mqtt(f"13 {DETECT_POSE['ch13']}", 3.0)

    time.sleep(1.0)


def align_cx(start_ch15, cx_target):
    K_ALIGN = 0.025
    STEP = 6

    cur = start_ch15

    for it in range(8):
        cx = run_box(cur, cx_target)

        if cx is None:
            print(f"  iter {it + 1}: 박스 X")
            return None

        e = cx - cx_target

        print(f"  iter {it + 1}: cx={cx:.0f} e{e:+.0f}")

        if abs(e) <= 25:
            print("  ✓")
            break

        d = max(-STEP, min(STEP, -K_ALIGN * e))
        new_ch15 = int(max(0, min(180, cur + d)))

        if new_ch15 == cur:
            break

        mqtt(f"15 {new_ch15}", 2.5)

        time.sleep(0.5)

        cur = new_ch15

    return cur


def pick_box(box, cx_target, no_pick=False):
    start_ch15 = BOX_START_CH15[box]

    print(f"\n=== {box} 잡기 (시작 CH15={start_ch15}) ===")

    print("[1] 검출 자세")
    move_detect_pose(start_ch15)

    print("[2] cx 정렬")
    cur = align_cx(start_ch15, cx_target)

    if cur is None:
        print(f"  {box}: cx 정렬 실패 — SKIP")
        return False

    print(f"[3] 박스 위치 측정 (CH15={cur})")
    pos = run_orange_blue(cur, cx_target)

    if pos is None:
        print(f"  {box}: 측정 실패 — SKIP")
        return False

    bx, by = pos

    print(f"  박스 중심 (베이스): ({bx:.1f}, {by:.1f}, -41)")

    exp_y = EXPECTED_Y[box]
    dy = by - exp_y

    if abs(dy) > Y_TOL:
        print(
            f"  ⚠ 밀림 감지: y={by:.1f} "
            f"(기대 {exp_y:+.0f}, 편차 {dy:+.1f} > {Y_TOL:.0f}) "
            f"— 가드 무시, 측정 y로 진행"
        )
    else:
        print(f"  자리 검증 OK: y 편차 {dy:+.1f} (기대 {exp_y:+.0f})")

    box_front_x = bx + BOX_DEPTH_HALF

    print(f"[4] 박스 정면 = ({box_front_x:.1f}, {by:.1f}, {GRIP_Z}) pitch_pref={TCP_PITCH}")

    pose, used_pitch = ik_auto(box_front_x, by, GRIP_Z, pitch_pref=TCP_PITCH)

    if pose is None:
        print("  IK 실패: 어떤 pitch로도 도달 불가 — SKIP")
        return False

    print(
        f"  IK 자세(pitch={used_pitch}): "
        f"CH15={pose.ch15} CH13={pose.ch13} CH11={pose.ch11} CH9={pose.ch9}"
    )

    if no_pick:
        print("  (--no-pick: 검출/IK만, 잡기 생략)")
        return False

    print(f"[5] 자세 도달 (CH15=cx정렬 {cur}, CH13/CH11/CH9=IK)")

    mqtt(f"13 {SAFE_SHOULDER}", 2.5)
    mqtt(f"15 {cur}", 3.5)
    mqtt(f"11 {pose.ch11}", 2.0)
    mqtt(f"9 {pose.ch9}", 2.5)
    mqtt(f"13 {pose.ch13}", 4.0)

    # 오픈루프 서보 정착용 재발행
    mqtt(f"11 {pose.ch11}", 1.5)
    mqtt(f"9 {pose.ch9}", 2.0)
    mqtt(f"13 {pose.ch13}", 3.5)

    time.sleep(0.5)

    print("[6] 그리퍼 닫기")
    mqtt(f"5 {GRIP_CLOSE}", 1.2)

    return True


def place_rear():
    print("[놓기 — 내려놓고 팔 접어서 치우기]")

    # 너무 높이 들지 않고 살짝만 들어 뒤쪽으로 회전
    mqtt("13 74", 3.0)
    mqtt(f"15 {PLACE_BASE}", 4.0)

    # 내려놓기 위해 팔을 펴되, 이후 다시 접어준다
    mqtt("11 0", 1.8)
    mqtt("9 170", 1.8)

    # 물건 내려놓기
    mqtt("13 20", 3.5)
    mqtt(f"5 {GRIP_OPEN}", 1.2)

    # 물건을 놓은 뒤 수직으로 들지 않고 낮은 접힘 자세로 정리
    mqtt("13 60", 1.8)
    mqtt("9 90", 1.5)
    mqtt("11 45", 1.8)
    mqtt("13 80", 2.0)


def main():
    global TCP_PITCH, GRIP_Z, BOX_DEPTH_HALF

    ap = argparse.ArgumentParser()

    ap.add_argument("--order", default="LCR")
    ap.add_argument("--intrinsic", default="/home/group4/camera_intrinsic.yaml")
    ap.add_argument("--skip-place", action="store_true")
    ap.add_argument(
        "--no-pick",
        action="store_true",
        help="검출/IK만, 잡기 생략 (진단용)",
    )
    ap.add_argument(
        "--pitch",
        type=float,
        default=TCP_PITCH,
        help="TCP pitch(deg). 멀어서 reach 한계면 -35 등으로 눕혀 연장",
    )
    ap.add_argument(
        "--grip-z",
        type=float,
        default=GRIP_Z,
        dest="grip_z",
    )
    ap.add_argument(
        "--front-offset",
        type=float,
        default=BOX_DEPTH_HALF,
        dest="front_offset",
        help="박스 중심→정면 깊이 오프셋(mm). 멀리서 near-pillar 물면 키워 중앙으로",
    )

    args = ap.parse_args()

    TCP_PITCH = args.pitch
    GRIP_Z = args.grip_z
    BOX_DEPTH_HALF = args.front_offset

    print(f"TCP_PITCH={TCP_PITCH} GRIP_Z={GRIP_Z} BOX_DEPTH_HALF={BOX_DEPTH_HALF}")

    with open(args.intrinsic) as f:
        intr = yaml.safe_load(f)

    K = np.array(intr["camera_matrix"]["matrix"])
    cx_target = K[0, 2]

    print(f"cx_target (광축) = {cx_target:.0f}")

    print("\n[R + 시작 안전 자세]")
    mqtt("R", 0.3)
    mqtt(f"5 {GRIP_OPEN}", 0.6)
    mqtt(f"13 {SAFE_SHOULDER}", 2.5)

    for box in args.order:
        if box not in BOX_START_CH15:
            print(f"\n{box}: 알 수 없음 SKIP")
            continue

        picked = pick_box(box, cx_target, no_pick=args.no_pick)

        if not picked:
            continue

        if not args.skip_place:
            place_rear()

    print("\n=== 완료 ===")


if __name__ == "__main__":
    main()
