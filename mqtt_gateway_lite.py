"""
mqtt_gateway_lite.py — 단독 실행 MQTT→시리얼 게이트웨이 (라즈베리파이용)
=====================================================================
RobotArmCase.py 없이 단독으로 돈다. MQTT 명령을 아두이노 시리얼로 그대로 전달.

실행:
    python3 mqtt_gateway_lite.py /dev/ttyACM1 localhost

의존성 (라파이 OS Bookworm 권장):
    sudo apt install -y python3-paho-mqtt python3-serial
"""
import sys, time, threading, serial
import paho.mqtt.client as mqtt

PORT   = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM1"
BROKER = sys.argv[2] if len(sys.argv) > 2 else "localhost"
TOPIC_CMD    = "pack/arm/cmd"
TOPIC_STATUS = "pack/arm/status"

# 아두이노 연결 (열면 아두이노 리셋 → 부팅 보정 동작)
ser = serial.Serial(PORT, 9600, timeout=1)
time.sleep(2)

# 아두이노 응답을 화면에 출력 (디버그)
def reader():
    while True:
        try:
            line = ser.readline().decode("utf-8", "ignore").strip()
            if line:
                print("  [arm]", line)
        except Exception:
            break
threading.Thread(target=reader, daemon=True).start()

def on_connect(c, u, f, rc):
    c.subscribe(TOPIC_CMD)
    c.publish(TOPIC_STATUS, "gateway connected")
    print(f"[MQTT] connected, sub {TOPIC_CMD}")

def on_message(c, u, msg):
    cmd = msg.payload.decode("utf-8", "ignore").strip()
    if not cmd:
        return
    print("[cmd]", cmd)
    ser.write((cmd + "\n").encode())

# paho 1.x / 2.x 호환
try:
    cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
except (AttributeError, TypeError):
    cli = mqtt.Client()
cli.on_connect = on_connect
cli.on_message = on_message
cli.connect(BROKER, 1883, 60)
print(f"[gateway] running on {PORT}, broker {BROKER}")
cli.loop_forever()
