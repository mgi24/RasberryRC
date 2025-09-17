import cv2
#window error klo nggk dijalanin sebelum picamera2
# cv2.namedWindow("Preview", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("Preview", 1366, 768)

from picamera2 import Picamera2

# Kamera utama (kamera 0)
picam2 = Picamera2(1)
config = picam2.create_video_configuration(
    main={"size": (256, 144), "format": "RGB888"},
    raw={"size": (picam2.sensor_resolution)}
)
picam2.configure(config)
picam2.start()
picam2.set_controls({"AfMode": 2, "AeEnable": False})  # Matikan autoexposure

# Kamera kedua (kamera 1)
picam2_2 = Picamera2(0)
config2 = picam2_2.create_video_configuration(
    main={"size": (128, 64), "format": "RGB888"},
    raw={"size": (picam2_2.sensor_resolution)}
)
picam2_2.configure(config2)
picam2_2.start()
picam2_2.set_controls({"AfMode": 2, "AeEnable": False})  # Matikan autoexposure

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
    "right": 23
}

CHIP_NAME = "gpiochip4"
chip = gpiod.Chip(CHIP_NAME)
lines = []

# Setup all GPIO lines at startup
for direction, pin in PINS.items():
    line = chip.get_line(pin)
    line.request(consumer="rc", type=gpiod.LINE_REQ_DIR_OUT)
    lines.append(line)

def set_lines(lines, value):
    for line in lines:
        line.set_value(value)

last_heartbeat = time.monotonic()
exposure = 10000

VPS_WS_URL = "ws://IPVPS:8765"
active_cam = 1
async def send_frames(ws):
    try:
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        while True:
            if active_cam == 1:
                frame = picam2.capture_array()
                frame = cv2.flip(frame, -1)
                ret, jpeg = cv2.imencode('.jpg', frame, encode_param)
                if not ret:
                    continue
                payload = b'\x01' + jpeg.tobytes()
                await ws.send(payload)
            elif active_cam == 2:
                # Kamera 1
                frame1 = picam2.capture_array()
                frame1 = cv2.flip(frame1, -1)
                ret1, jpeg1 = cv2.imencode('.jpg', frame1, encode_param)
                if ret1:
                    payload1 = b'\x01' + jpeg1.tobytes()
                    await ws.send(payload1)
                # Kamera 2
                frame2 = picam2_2.capture_array()
                frame2 = cv2.flip(frame2, -1)
                ret2, jpeg2 = cv2.imencode('.jpg', frame2, encode_param)
                if ret2:
                    payload2 = b'\x02' + jpeg2.tobytes()
                    await ws.send(payload2)
            await asyncio.sleep(0.05)  # ~20fps
    except Exception as e:
        print(f"Stream error: {e}")




async def receive_commands(ws):
    try:
        while True:
            msg = await ws.recv()
            if msg == "1":
                global last_heartbeat
                elapsed = time.monotonic() - last_heartbeat
                print(f"Heartbeat interval: {elapsed:.3f}s")
                last_heartbeat = time.monotonic()
                continue
            if msg == "camera2on":
                global active_cam
                active_cam = 2
                print("Switched on camera 2")
                continue
            if msg == "camera2off":
                active_cam = 1
                print("Switched off camera 2")
                continue
            if msg.startswith("exposure"):
                try:
                    value = int(msg[len("exposure"):])
                    global exposure
                    exposure = value
                    print(f"Exposure set to {exposure}")
                    # Set exposure for both cameras
                    picam2.set_controls({"ExposureTime": exposure})
                    picam2_2.set_controls({"ExposureTime": exposure})
                except ValueError:
                    print("Invalid exposure value")
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
                heartbeat_task = asyncio.create_task(heartbeat_watcher())
                
                done, pending = await asyncio.wait(
                    [consumer_task, producer_task, heartbeat_task],
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
