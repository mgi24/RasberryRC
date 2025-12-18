import cv2
#window error klo nggk dijalanin sebelum picamera2
# cv2.namedWindow("Preview", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("Preview", 1366, 768)

from picamera2 import Picamera2

# Kamera utama (kamera 0)
picam2 = Picamera2(0)
config = picam2.create_video_configuration(
    main={"size": (256, 144), "format": "RGB888"},#set img res
    raw={"size": (picam2.sensor_resolution)}
)
picam2.configure(config)
picam2.start()
picam2.set_controls({"AeEnable": False})  # Matikan autoexposure

VPS_WS_URL = "ws://168.110.218.135:8766"
import gpiod
import signal
import sys
import time
import websockets
import json

from picamera2 import Picamera2
import threading
import asyncio

# GPIO pin mapping
PINS = {
    "forward": 17,
    "backward": 27,
    "left": 22,
    "right": 23,
    "light": 24
}

CHIP_NAME = "gpiochip0"
chip = gpiod.Chip(CHIP_NAME)
lines = []

# Setup all GPIO lines at startup
for direction, pin in PINS.items():
    line = chip.get_line(pin)
    line.request(consumer="rc", type=gpiod.LINE_REQ_DIR_OUT)
    lines.append(line)
    line.set_value(0) #disable all at start

def set_lines(lines, value):
    for line in lines:
        line.set_value(value)


exposure = 10000
gain = 1.0


# ===== AUDIO (1 arah: Pi -> WS -> Client) =====
AUDIO_DEV_IN = "plughw:1,0"     # USB PnP Audio Device (mic)
AUDIO_RATE = 16000             # Hz
AUDIO_CH = 1                   # mono
AUDIO_FRAME_MS = 20            # 20ms per packet (umum untuk low latency)
AUDIO_BYTES_PER_SAMPLE = 2     # S16_LE

AUDIO_SAMPLES_PER_FRAME = int(AUDIO_RATE * AUDIO_FRAME_MS / 1000)  # 320
AUDIO_BYTES_PER_FRAME = AUDIO_SAMPLES_PER_FRAME * AUDIO_CH * AUDIO_BYTES_PER_SAMPLE  # 640


async def send_frames(ws):
    try:
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        while True:
            frame = picam2.capture_array()
            frame = cv2.flip(frame, -1)
            ret, jpeg = cv2.imencode('.jpg', frame, encode_param)
            if not ret:
                continue
            payload = b'\x01' + jpeg.tobytes()
            await ws.send(payload)
            await asyncio.sleep(0.05)  # ~20fps
    except Exception as e:
        print(f"Stream error: {e}")


async def send_audio(ws):
    """
    Capture mic PCM via arecord dan kirim ke WS sebagai binary:
      0x02 + PCM(20ms, S16_LE, 16kHz, mono)
    """
    proc = await asyncio.create_subprocess_exec(
        "arecord",
        "-D", AUDIO_DEV_IN,
        "-f", "S16_LE",
        "-r", str(AUDIO_RATE),
        "-c", str(AUDIO_CH),
        "-t", "raw",
        "-q",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.DEVNULL,
    )
    try:
        while True:
            pcm = await proc.stdout.readexactly(AUDIO_BYTES_PER_FRAME)
            await ws.send(b"\x02" + pcm)
    except asyncio.IncompleteReadError:
        print("[AUDIO] arecord ended")
    except Exception as e:
        print(f"[AUDIO] error: {e}")
    finally:
        if proc.returncode is None:
            proc.terminate()


last_heartbeat = time.monotonic()
async def receive_commands(ws):
    try:
        while True:
            msg = await ws.recv()

            # Binary dari server (mis. video/audio) diabaikan oleh Pi pada mode ini
            if isinstance(msg, (bytes, bytearray)):
                continue

            if msg == "1":
                global last_heartbeat
                elapsed = time.monotonic() - last_heartbeat
                print(f"Heartbeat interval: {elapsed:.3f}s")
                last_heartbeat = time.monotonic()
                continue

            if msg.startswith("exposure"):
                try:
                    value = int(msg[len("exposure"):])
                    global exposure
                    exposure = value
                    print(f"Exposure set to {exposure}")
                    # Set exposure for both cameras
                    picam2.set_controls({"ExposureTime": exposure})
                except ValueError:
                    print("Invalid exposure value")
                continue

            if msg.startswith("gain"):
                try:
                    value = float(msg[len("gain"):])/1000.0
                    global gain
                    gain = value
                    print(f"Gain set to {gain}")
                    picam2.set_controls({"AnalogueGain": gain})
                except ValueError:
                    print("Invalid gain value")
                continue

            if msg.startswith("light"):
                try:
                    value = int(msg[len("light"):])
                    light_line = lines[list(PINS.keys()).index("light")]
                    light_line.set_value(value)
                    print(f"Light set to {value}")
                except (ValueError, KeyError) as e:
                    print(f"Invalid light command: {e}")
                continue

            try:
                data = json.loads(msg)
                print(data)
                for direction, state in data.items():
                    if direction in PINS:
                        line = lines[list(PINS.keys()).index(direction)]
                        line.set_value(1 if state else 0)
            except Exception as e:
                print(f"GPIO control error: {e}")
    except Exception as e:
        print(f"Websocket closed: {e}")
        # Stop all motors on disconnect
        set_lines(lines, 0)


async def heartbeat_watcher():
    global last_heartbeat
    while True:
        if time.monotonic() - last_heartbeat > 0.5:
            print("Heartbeat lost, stopping motors")
            set_lines(lines, 0)
        await asyncio.sleep(0.05)

async def run_client():
    while True:
        try:
            async with websockets.connect(VPS_WS_URL, max_size=2**23) as ws:
                print(f"Connected to {VPS_WS_URL}")
                # Identify this client
                await ws.send("pi")

                consumer_task = asyncio.create_task(receive_commands(ws))
                producer_task = asyncio.create_task(send_frames(ws))
                audio_task = asyncio.create_task(send_audio(ws))
                heartbeat_task = asyncio.create_task(heartbeat_watcher())

                done, pending = await asyncio.wait(
                    [consumer_task, producer_task, audio_task, heartbeat_task],
                    return_when=asyncio.FIRST_COMPLETED,
                )
                for task in pending:
                    task.cancel()
        except Exception as e:
            print(f"Connection error: {e}")
        finally:
            # Ensure motors are off between reconnects
            set_lines(lines, 0)
            await asyncio.sleep(3)  # retry delay


def cleanup():
    print("Cleaning up GPIO...")
    set_lines(lines, 0)
    for line in lines:
        line.release()
    chip.close()


if __name__ == "__main__":
    try:
        asyncio.run(run_client())
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        cleanup()
