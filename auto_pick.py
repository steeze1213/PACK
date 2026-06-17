"""
auto_pick.py — 3개 물건 자동 집기 (위에서 하강 방식)
=====================================================
마커 기준 좌/중/우 3개 물건을 위에서 내려와 집어 후면에 내려놓는다.
MQTT로 명령 발행 → 게이트웨이 → 아두이노. (gateway 켜져 있어야 함)

핵심 동작 원칙:
  - 베이스 회전은 '팔 든 상태'에서만 (옆 물건 안 침)
  - 하강·상승은 제자리에서 (위→아래→위)

실행:
    python3 auto_pick.py          # 대화형 (L/C/R 입력)
    python3 auto_pick.py C        # 중앙 물건 한 번 집기

의존성: pip install paho-mqtt   (또는 sudo apt install python3-paho-mqtt)
"""

import sys, time
import paho.mqtt.client as mqtt

BROKER = "192.168.0.187"
TOPIC  = "pack/arm/cmd"

# ── 물건별 베이스각 (Phase 2 실측) ──
OBJECTS = {"L": 49, "C": 37, "R": 25}   # 재조립 후 실측 확정 (2026-06-12)

# ── 공통 팔자세 (집는 순간 값, 실측) ──
GRIP_SHOULDER = 20     # 어깨13 (낮춰서 물건에)
GRIP_ELBOW    = 20     # 팔꿈치11
GRIP_WRIST    = 150    # 손목9 (새 집게: 155→150)
GRIP_ROLL     = 130    # 손목회전7
GRIP_CLOSE    = 20     # 집게 닫기 (새 집게: 50→20)
GRIP_OPEN     = 0      # 집게 열기

# ── 동작 파라미터 ──
HIGH_SHOULDER = 120    # 팔 든 높이 (안전 이동/회전용)
PLACE_BASE    = 150    # 후면 차량 방향
PLACE_SHOULDER= 60     # 후면 내려놓을 때 어깨 높이
PLACE_WRIST   = 90     # 놓을 때 손목 숙임 (110→90, 더 푹 숙여 중력낙하 강화)

# ── 이동 대기시간 (초) — 동작 완료까지 기다림 ──
MOVE_DELAY = 17        # 펌웨어 팔 이동속도 (ms/도) 25→17 = 약 1.5배 빠르게
WAIT_BIG   = 3.0       # 큰 관절 이동 (속도 1.5배 맞춤)
WAIT_SMALL = 1.7       # 작은 이동
WAIT_GRIP  = 1.0       # 집게


# MQTT (paho 1.x/2.x 호환)
try:
    cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
except (AttributeError, TypeError):
    cli = mqtt.Client()
cli.connect(BROKER, 1883, 60)
cli.loop_start()

def send(cmd, wait):
    print(f"  > {cmd:10s} (대기 {wait}s)")
    cli.publish(TOPIC, cmd)
    time.sleep(wait)


def ready():
    """집기 준비: 팔 들고 집게 열고 자세 세팅."""
    send(f"5 {GRIP_OPEN}", WAIT_GRIP)
    send(f"13 {HIGH_SHOULDER}", WAIT_BIG)      # 어깨 올림(팔 높이)
    send(f"11 {GRIP_ELBOW}", WAIT_SMALL)       # 팔꿈치 세팅
    send(f"9 {GRIP_WRIST}", WAIT_SMALL)        # 손목
    send(f"7 {GRIP_ROLL}", WAIT_SMALL)         # 손목회전


def grab_place(base):
    """준비된 상태에서: 조준→하강→집기→들기→후면 내려놓기. (팔은 든 채로 끝남)"""
    send(f"15 {base}", WAIT_BIG)               # 조준 (팔 든 상태 → 옆 물건 안 침)
    send(f"13 {GRIP_SHOULDER}", WAIT_BIG)      # 하강
    send(f"5 {GRIP_CLOSE}", WAIT_GRIP)         # 집기
    send(f"13 {HIGH_SHOULDER}", WAIT_BIG)      # 들기 (수직 상승)
    send(f"15 {PLACE_BASE}", WAIT_BIG)         # 후면 회전 (팔 든 상태)
    send(f"13 {PLACE_SHOULDER}", WAIT_BIG)     # 놓기 하강
    send(f"9 {PLACE_WRIST}", WAIT_SMALL)       # 손목 숙임 (고리 중력으로 빠짐)
    send(f"5 {GRIP_OPEN}", WAIT_GRIP)          # 열기
    send(f"9 {GRIP_WRIST}", WAIT_SMALL)        # 손목 복귀(일자)
    send(f"13 {HIGH_SHOULDER}", WAIT_BIG)      # 다시 들기


def pick(obj):
    """단일 물건: 준비 → 집기·놓기 → 홈."""
    if obj not in OBJECTS:
        print("L / C / R 중 하나"); return
    print(f"\n=== {obj} 집기 (베이스 {OBJECTS[obj]}) ===")
    ready()
    grab_place(OBJECTS[obj])
    send("H", WAIT_BIG)
    print(f"=== {obj} 완료 ===\n")


def pick_all():
    """연속: 준비 1회 → 3개 집기·놓기(중간 홈 생략) → 마지막에만 홈."""
    print("\n=== ALL 연속 집기 ===")
    ready()
    for o in ["L", "C", "R"]:
        print(f"-- {o} (베이스 {OBJECTS[o]}) --")
        grab_place(OBJECTS[o])
    send("H", WAIT_BIG)
    print("=== ALL 완료 ===\n")


def main():
    send(f"SP {MOVE_DELAY}", 0.5)              # 팔 속도 2배로 (펌웨어)
    if len(sys.argv) > 1:
        arg = sys.argv[1].upper()
        if arg == "ALL":
            pick_all()
        else:
            pick(arg)
    else:
        print("물건 집기: L(좌) / C(중) / R(우) / ALL(전부) / q(종료)")
        while True:
            c = input("pick> ").strip().upper()
            if c == "Q":
                break
            elif c == "ALL":
                pick_all()
            elif c in OBJECTS:
                pick(c)
            else:
                print("L / C / R / ALL / q")
    cli.loop_stop()
    cli.disconnect()


if __name__ == "__main__":
    main()
