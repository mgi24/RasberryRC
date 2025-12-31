import cv2
#window error klo nggk dijalanin sebelum picamera2
# cv2.namedWindow("Preview", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("Preview", 1366, 768)

from picamera2 import Picamera2

# Kamera utama (kamera 0)
picam2 = Picamera2(0)
config = picam2.create_video_configuration(
    main={"size": (1280, 720), "format": "RGB888"},  # record source (and can still be used for capture if needed)
    lores={"size": (256, 144), "format": "YUV420"},  # streaming source
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

import threading
import asyncio
import wave
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput
import os
from datetime import datetime
import shutil
import subprocess

#######################GPIO AREA#############################
# GPIO pin mapping
SERVO_PIN = 12

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
    line.set_value(0)

# ===== SOFTWARE PWM =====

class SoftwarePWM:
    def __init__(self, line, frequency=1000):
        self.line = line
        self.frequency = frequency
        self.duty_cycle = 0  # 0-100
        self.enabled = False
        self.running = False
        self.thread = None
    
    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._pwm_loop, daemon=True)
            self.thread.start()
    
    def _pwm_loop(self):
        period = 1.0 / self.frequency
        while self.running:
            if self.enabled and self.duty_cycle > 0:
                on_time = period * (self.duty_cycle / 100.0)
                off_time = period - on_time
                
                self.line.set_value(1)
                time.sleep(on_time)
                self.line.set_value(0)
                time.sleep(off_time)
            else:
                self.line.set_value(0)
                time.sleep(period)
    
    def set_duty_cycle(self, duty_cycle):
        """Set duty cycle 0-100"""
        self.duty_cycle = max(0, min(100, duty_cycle))
    
    def enable(self):
        """Enable PWM output"""
        self.enabled = True
    
    def disable(self):
        """Disable PWM output"""
        self.enabled = False
        self.line.set_value(0)
    
    def stop(self):
        """Stop PWM thread"""
        self.running = False
        if self.thread:
            self.thread.join()
        self.line.set_value(0)

# Create PWM instances for forward and backward
forward_pwm = SoftwarePWM(lines[list(PINS.keys()).index("forward")], frequency=1000)
backward_pwm = SoftwarePWM(lines[list(PINS.keys()).index("backward")], frequency=1000)

# Start PWM threads
forward_pwm.start()
backward_pwm.start()

# Global speed variable (0-100)
motor_speed = 50

# ===== Helper functions =====
def set_lines(lines, value):
    for line in lines:
        line.set_value(value)

def update_motor_direction(direction, state):
    """Update motor direction with PWM speed control"""
    if direction == "forward":
        if state:
            backward_pwm.disable()
            forward_pwm.set_duty_cycle(motor_speed)
            forward_pwm.enable()
            print(f"Forward at {motor_speed}% speed")
        else:
            forward_pwm.disable()
    elif direction == "backward":
        if state:
            forward_pwm.disable()
            backward_pwm.set_duty_cycle(motor_speed)
            backward_pwm.enable()
            print(f"Backward at {motor_speed}% speed")
        else:
            backward_pwm.disable()
    elif direction in ["left", "right"]:
        # Left/Right tetap digital (tanpa PWM)
        line = lines[list(PINS.keys()).index(direction)]
        line.set_value(1 if state else 0)
        
def disable_motors():
    forward_pwm.disable()
    backward_pwm.disable()
    set_lines([lines[list(PINS.keys()).index("left")], 
              lines[list(PINS.keys()).index("right")]], 0)  # left, right only


########################END OF GPIO AREA#############################


#########################AUDIO STREAM AREA###########################
AUDIO_DEV_IN = "plughw:1,0"     # USB PnP Audio Device (mic)
AUDIO_RATE = 16000             # Hz
AUDIO_CH = 1                   # mono
AUDIO_FRAME_MS = 20            # 20ms per packet (umum untuk low latency)
AUDIO_BYTES_PER_SAMPLE = 2     # S16_LE

RECORD_DIR = os.path.join(os.path.dirname(__file__), "recordings")
os.makedirs(RECORD_DIR, exist_ok=True)
record_filename = None
audio_file_lock = threading.Lock()
audio_wav = None
def open_audio_wav():
    global record_filename, audio_wav
    audio_path = os.path.join(RECORD_DIR, f"{record_filename}.wav")
    wf = wave.open(audio_path, 'wb')
    wf.setnchannels(AUDIO_CH)
    wf.setsampwidth(AUDIO_BYTES_PER_SAMPLE)
    wf.setframerate(AUDIO_RATE)
    audio_wav = wf

def close_audio_wav():
    global audio_wav
    if audio_wav is None:
        return
    try:
        audio_wav.close()
        print(f"[REC] STOP audio -> {record_filename}.wav")
    except Exception as e:
        print(f"[REC] stop audio error: {e}")
    audio_wav = None


AUDIO_SAMPLES_PER_FRAME = int(AUDIO_RATE * AUDIO_FRAME_MS / 1000)  # 320
AUDIO_BYTES_PER_FRAME = AUDIO_SAMPLES_PER_FRAME * AUDIO_CH * AUDIO_BYTES_PER_SAMPLE  # 640

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
            if recording:
                with audio_file_lock:
                    if audio_wav is not None:
                        audio_wav.writeframes(pcm)
            await ws.send(b"\x02" + pcm)



    except asyncio.IncompleteReadError:
        print("[AUDIO] arecord ended")
    except Exception as e:
        print(f"[AUDIO] error: {e}")
    finally:
        if proc.returncode is None:
            proc.terminate()
        with audio_file_lock:
            close_audio_wav()



###################### END OF AUDIO STREAM AREA###########################


######################## VIDEO STREAM AREA ################################


recording = False
record_request = False
h264_encoder = H264Encoder(bitrate=3_000_000)  # ~3Mbps; adjust as needed

video_path = None

def new_record_name():
    return datetime.now().strftime("%Y%m%d_%H%M%S")

def start_video_recording():
    global recording, video_output, video_path, record_filename
    if not recording:
        record_filename = new_record_name()
        video_path = os.path.join(RECORD_DIR, f"{record_filename}.h264")
        print(f"[REC] START video -> {video_path}")
        video_output = FileOutput(video_path)
        picam2.start_recording(h264_encoder, video_output)
        with audio_file_lock:
            open_audio_wav()
        recording = True

def stop_video_recording():
    global recording, video_output, video_path
    if recording:
        print("[REC] STOP video")
        try:
            picam2.stop_encoder()
            with audio_file_lock:
                close_audio_wav()
            video_output = None
            recording = False   
        except Exception as e:
            print(f"[REC] stop error: {e}")

last_record_report=0
exposure = 10000
gain = 1.0
async def send_frames(ws):
    global recording, last_record_report, record_request
    try:
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        while True:
            # Apply record state changes here (single Picamera2 access point)
            # Kirim status recording setiap detik
            current_time = time.monotonic()
            if current_time - last_record_report >= 1:
                status = "record1" if recording else "record2"
                await ws.send(status)
                last_record_report = current_time

            # Apply record state changes here (single Picamera2 access point)
            if record_request and not recording:
                start_video_recording()

            elif (not record_request) and recording:
                stop_video_recording()
            frame = picam2.capture_array("lores")  # 256x144 already
            bgr = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_I420)
            ret, jpeg = cv2.imencode('.jpg', bgr, encode_param)
            if not ret:
                continue
            payload = b'\x01' + jpeg.tobytes()
            await ws.send(payload)
            await asyncio.sleep(0.05)  # ~20fps
    except Exception as e:
        print(f"Stream error: {e}")
        with audio_file_lock:
            close_audio_wav()

######################### END OF VIDEO STREAM AREA #############################


######################### WEBSOCKET HANDLER AREA #############################
last_heartbeat = time.monotonic()

async def receive_commands(ws):
    global record_request
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

            if msg.startswith("speed"):
                try:
                    value = int(msg[len("speed"):])
                    global motor_speed
                    motor_speed = max(0, min(100, value))
                    print(f"Motor speed set to {motor_speed}%")
                    # Update duty cycle untuk motor yang sedang aktif
                    if forward_pwm.enabled:
                        forward_pwm.set_duty_cycle(motor_speed)
                    if backward_pwm.enabled:
                        backward_pwm.set_duty_cycle(motor_speed)
                except ValueError:
                    print("Invalid speed value")
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
            if msg.startswith("record"):
                cmd = msg.strip()
                if cmd == "record1":
                    
                    record_request = True
                elif cmd == "record2":
                    
                    record_request = False
                continue
            if msg.startswith("servo"):
                try:
                    value = float(msg[len("servo"):])
                    print(f"Servo angle set to {value}")
                except ValueError:
                    print("Invalid servo value")
                continue

            try:
                data = json.loads(msg)
                print(data)
                for direction, state in data.items():
                    if direction in PINS:
                        update_motor_direction(direction, state)
            except Exception as e:
                print(f"GPIO control error: {e}")
    except Exception as e:
        print(f"Websocket closed: {e}")
        # Stop all motors on disconnect
        disable_motors()
        

async def heartbeat_watcher():
    global last_heartbeat
    while True:
        if time.monotonic() - last_heartbeat > 0.5:
            print("Heartbeat lost, stopping motors")
            disable_motors()
        await asyncio.sleep(0.05)


############################ END OF WEBSOCKET HANDLER AREA #############################


########################### MAIN LOOP AREA #############################

def _audio_device_available() -> bool:
    """Best-effort check: arecord exists and can open AUDIO_DEV_IN."""
    if shutil.which("arecord") is None:
        return False
    try:
        # Quick open test (no real data needed). arecord will fail fast if device missing.
        r = subprocess.run(
            ["arecord", "-D", AUDIO_DEV_IN, "-f", "S16_LE", "-r", str(AUDIO_RATE), "-c", str(AUDIO_CH), "-t", "raw", "-d", "1", "-q"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=2,
            check=False,
        )
        return r.returncode == 0
    except Exception:
        return False

async def run_client():
    stream_audio = False
    stream_audio = _audio_device_available()
    if not stream_audio:
        print(f"[AUDIO] device not available ({AUDIO_DEV_IN}). Audio task will not start.")

    while True:
        try:
            async with websockets.connect(VPS_WS_URL, max_size=2**23) as ws:
                print(f"Connected to {VPS_WS_URL}")
                # Identify this client
                await ws.send("pi")

                consumer_task = asyncio.create_task(receive_commands(ws))
                producer_task = asyncio.create_task(send_frames(ws))
                
                heartbeat_task = asyncio.create_task(heartbeat_watcher())
                tasks = [consumer_task, producer_task, heartbeat_task]
                if stream_audio:
                    audio_task = asyncio.create_task(send_audio(ws))
                    tasks.append(audio_task)
                done, pending = await asyncio.wait(
                    tasks,
                    return_when=asyncio.FIRST_COMPLETED,
                )
                for task in pending:
                    task.cancel()
        except Exception as e:
            print(f"Connection error: {e}")
        finally:
            disable_motors()
            await asyncio.sleep(3)

def cleanup():
    print("Cleaning up GPIO...")
    forward_pwm.stop()
    backward_pwm.stop()
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
######################### END OF MAIN LOOP AREA #############################