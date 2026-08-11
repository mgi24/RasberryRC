import asyncio
import fcntl
import socket as _socket
import struct
import subprocess
import threading
import time
import json
from contextlib import asynccontextmanager
from pathlib import Path

import gpiod
import requests
import serial
import uvicorn
import websockets
from fastapi import FastAPI
from fastapi.responses import FileResponse

# Camera imports
import cv2
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput
from datetime import datetime
import wave

# ===================== Config =====================

VPS_WS_URL = "ws://192.168.0.103:8766"       # WebSocket VPS (ganti jika perlu)
WEB_HTML   = Path(__file__).parent / "web.html" # path ke file HTML statis

# ===================== GPIO Config & State =====================
LED_PIN = 23
CHIP_NAME = "gpiochip4"

try:
    chip = gpiod.Chip(CHIP_NAME)
    led_line = chip.get_line(LED_PIN)
    led_line.request(consumer="rc_zero", type=gpiod.LINE_REQ_DIR_OUT)
    print(f"[GPIO] Pin {LED_PIN} initialized on {CHIP_NAME}")
except Exception as e:
    print(f"[GPIO] Failed to initialize pin {LED_PIN}: {e}")
    led_line = None

# ===================== Camera Config & State =====================

# Inisialisasi kamera utama (kamera 0)
# Akan diinisialisasi di dalam lifespan() agar pesan info/error 
# libcamera tidak hilang akibat sudo/environment uvicorn
picam2 = None

recording = False
record_request = False
record_start_time = 0.0          # waktu mulai rekam (monotonic)
h264_encoder = H264Encoder(bitrate=3_000_000)  # ~3Mbps
video_output = None
video_path = None
record_filename = None
last_record_report = 0.0
exposure = 10000
gain = 1.0

# ===================== Audio Config & State =====================
AUDIO_DEV_IN = "plughw:1,0"
AUDIO_RATE = 48000
AUDIO_CH = 1
AUDIO_FRAME_MS = 20
AUDIO_BYTES_PER_SAMPLE = 2  # S16_LE
AUDIO_SAMPLES_PER_FRAME = int(AUDIO_RATE * AUDIO_FRAME_MS / 1000)  # 960
AUDIO_BYTES_PER_FRAME = AUDIO_SAMPLES_PER_FRAME * AUDIO_CH * AUDIO_BYTES_PER_SAMPLE  # 1920

audio_clients = set()  # set containing "stream" and/or "record"
audio_proc = None
audio_reader_task = None
audio_ws = None
audio_wav = None
audio_wav_lock = threading.Lock()

async def audio_reader_loop():
    global audio_proc, audio_wav, audio_ws, audio_clients
    try:
        while True:
            pcm = await audio_proc.stdout.readexactly(AUDIO_BYTES_PER_FRAME)
            
            # Write full quality 48k PCM to local WAV if recording is active
            if "record" in audio_clients:
                with audio_wav_lock:
                    if audio_wav is not None:
                        audio_wav.writeframes(pcm)
                        
            # Stream 16k PCM to websocket client if streaming is active
            if "stream" in audio_clients and audio_ws is not None:
                samples = struct.unpack(f"<{len(pcm)//2}h", pcm)
                samples_16k = samples[::3]
                pcm_16k = struct.pack(f"<{len(samples_16k)}h", *samples_16k)
                try:
                    await audio_ws.send(b"\x02" + pcm_16k)
                except Exception as e:
                    print(f"[AUDIO] WS send error: {e}")
                    audio_clients.discard("stream")
    except asyncio.CancelledError:
        print("[AUDIO] Reader loop cancelled")
    except asyncio.IncompleteReadError:
        print("[AUDIO] arecord stream ended")
    except Exception as e:
        print(f"[AUDIO] Reader loop error: {e}")
    finally:
        if audio_proc and audio_proc.returncode is None:
            try:
                audio_proc.terminate()
                await audio_proc.wait()
            except Exception:
                pass
        audio_proc = None
        print("[AUDIO] stopped")

async def update_audio_state(client: str, active: bool, ws=None):
    global audio_proc, audio_reader_task, audio_ws, audio_clients
    if active:
        audio_clients.add(client)
        if ws:
            audio_ws = ws
    else:
        audio_clients.discard(client)
        if client == "stream":
            audio_ws = None

    if audio_clients:
        if audio_proc is None or audio_proc.returncode is not None:
            try:
                audio_proc = await asyncio.create_subprocess_exec(
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
                print("[AUDIO] arecord started")
                await asyncio.sleep(0.2)
                if audio_proc.returncode is not None:
                    print(f"[AUDIO] arecord failed to start (returncode: {audio_proc.returncode})")
                    audio_proc = None
                    audio_clients.clear()
                    if ws and client == "stream":
                        await ws.send(json.dumps({"audio_confirm": "fail", "reason": "device error"}))
                    return
                
                if audio_reader_task is None or audio_reader_task.done():
                    audio_reader_task = asyncio.create_task(audio_reader_loop())
                
                if ws and client == "stream":
                    await ws.send(json.dumps({"audio_confirm": "ok", "audio": 1}))
            except Exception as e:
                print(f"[AUDIO] start arecord failed: {e}")
                audio_proc = None
                audio_clients.clear()
                if ws and client == "stream":
                    await ws.send(json.dumps({"audio_confirm": "fail", "reason": str(e)}))
        else:
            if ws and client == "stream":
                await ws.send(json.dumps({"audio_confirm": "ok", "audio": 1}))
    else:
        if audio_reader_task and not audio_reader_task.done():
            audio_reader_task.cancel()
            try:
                await audio_reader_task
            except asyncio.CancelledError:
                pass
        audio_reader_task = None
        audio_proc = None
        if ws and client == "stream":
            await ws.send(json.dumps({"audio_confirm": "ok", "audio": 0}))

RECORD_DIR = Path(__file__).parent / "recordings"
RECORD_DIR.mkdir(exist_ok=True)

def open_audio_wav(filename: str):
    global audio_wav
    audio_path = RECORD_DIR / f"{filename}.wav"
    print(f"[REC] START audio -> {audio_path}")
    try:
        wf = wave.open(str(audio_path), 'wb')
        wf.setnchannels(AUDIO_CH)
        wf.setsampwidth(AUDIO_BYTES_PER_SAMPLE)
        wf.setframerate(AUDIO_RATE)
        with audio_wav_lock:
            audio_wav = wf
    except Exception as e:
        print(f"[REC] open audio wav failed: {e}")

def close_audio_wav():
    global audio_wav
    with audio_wav_lock:
        if audio_wav is not None:
            try:
                audio_wav.close()
                print("[REC] STOP audio")
            except Exception as e:
                print(f"[REC] close audio wav failed: {e}")
            audio_wav = None

def new_record_name():
    return datetime.now().strftime("%Y%m%d_%H%M%S")

async def start_video_recording():
    global recording, video_output, video_path, record_filename, record_start_time
    if picam2 is None:
        print("[REC] Camera is not initialized, cannot record")
        return
    if not recording:
        record_filename = new_record_name()
        video_path = RECORD_DIR / f"{record_filename}.h264"
        print(f"[REC] START video -> {video_path}")
        
        # Mulai rekam audio ke WAV local
        open_audio_wav(record_filename)
        await update_audio_state("record", True)
        
        video_output = FileOutput(str(video_path))
        picam2.start_recording(h264_encoder, video_output)
        record_start_time = time.monotonic()
        recording = True

async def stop_video_recording():
    global recording, video_output, video_path
    if picam2 is None:
        return
    if recording:
        duration = time.monotonic() - record_start_time
        print(f"[REC] STOP video (durasi: {duration:.1f}s)")
        
        # Hentikan rekam audio WAV local
        await update_audio_state("record", False)
        close_audio_wav()
        
        try:
            picam2.stop_encoder()
            video_output = None
            recording = False
            # Rename file: tambah info FPS ke nama file
            if video_path and video_path.exists():
                threading.Thread(
                    target=_rename_with_fps,
                    args=(video_path, duration),
                    daemon=True
                ).start()
        except Exception as e:
            print(f"[REC] stop error: {e}")

def _rename_with_fps(h264_path: Path, duration: float):
    """Rename .h264 file: tambah info FPS ke nama.
    Contoh: 20260608_223232.h264 -> 20260608_223232-15_60.h264
    """
    if duration < 0.5:
        print("[REC] Durasi terlalu pendek, skip rename")
        return
    file_size_kb = h264_path.stat().st_size // 1024
    # Format duration: one decimal place, replace '.' with '_' for filename safety
    duration_str = f"{duration:.1f}".replace('.', '_')
    new_name = f"{h264_path.stem}-{duration_str}.h264"
    new_path = h264_path.parent / new_name
    h264_path.rename(new_path)
    print(f"[REC] Renamed: {new_name} ({file_size_kb} KB, {duration_str.replace('_', '.')}s)")


# ===================== Firebase Credentials =====================
# Sinkron dengan rc_esp.ino

API_KEY       = "AIzaSyA_mmjlGHFOLw5xfv8VNfya7RubMs1YHH0"
DATABASE_URL  = "https://rc-control-a07b8-default-rtdb.asia-southeast1.firebasedatabase.app"
USER_EMAIL    = "miskamumtaza123@gmail.com"
USER_PASSWORD = "apalah123"

# ===================== Network Utilities =====================

def get_wlan0_ip() -> str:
    """Ambil IP address interface wlan0 secara langsung via ioctl."""
    SIOCGIFADDR = 0x8915
    s = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
    try:
        ifreq = struct.pack("16sH14s", b"wlan0", _socket.AF_INET, b"\x00" * 14)
        res = fcntl.ioctl(s.fileno(), SIOCGIFADDR, ifreq)
        return _socket.inet_ntoa(res[20:24])
    except OSError:
        return "0.0.0.0"
    finally:
        s.close()

# ===================== Firebase Auth =====================

_id_token: str = ""
_token_expiry: float = 0.0

def firebase_signin() -> bool:
    """Sign-in ke Firebase dengan email+password, simpan ID token."""
    global _id_token, _token_expiry
    url = (
        "https://identitytoolkit.googleapis.com/v1/accounts:signInWithPassword"
        f"?key={API_KEY}"
    )
    payload = {"email": USER_EMAIL, "password": USER_PASSWORD, "returnSecureToken": True}
    try:
        r = requests.post(url, json=payload, timeout=10)
        r.raise_for_status()
        data = r.json()
        _id_token = data["idToken"]
        _token_expiry = time.time() + int(data.get("expiresIn", 3600)) - 60
        print("[Firebase] Sign-in OK")
        return True
    except Exception as e:
        print(f"[Firebase] Sign-in FAILED: {e}")
        return False

def _ensure_token() -> bool:
    """Re-sign-in jika token expired."""
    if time.time() >= _token_expiry:
        return firebase_signin()
    return True

# ===================== Firebase RTDB =====================

def firebase_set(path: str, value) -> bool:
    """PUT value ke path Firebase RTDB. Return True jika sukses."""
    if not _ensure_token():
        return False
    url = f"{DATABASE_URL}{path}.json?auth={_id_token}"
    try:
        r = requests.put(url, json=value, timeout=10)
        r.raise_for_status()
        return True
    except Exception as e:
        print(f"[Firebase] SET {path} FAILED: {e}")
        return False

# ===================== Startup Sequence =====================

def startup():
    """Jalankan sekali saat boot: sign-in Firebase, kirim IP wlan0, & kirim waktu RPi."""
    print("[Startup] Signing in to Firebase...")
    if not firebase_signin():
        print("[Startup] WARNING: Firebase tidak tersambung, lanjut tanpa Firebase")

    ip = get_wlan0_ip()
    print(f"[Startup] wlan0 IP: {ip}")

    if firebase_set("/rpi_ip", ip):
        print(f"[Startup] /rpi_ip updated: {ip}")
    else:
        print("[Startup] WARNING: Gagal kirim IP ke Firebase")

    time_str = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    if firebase_set("/time_rpi", time_str):
        print(f"[Startup] /time_rpi updated: {time_str}")
    else:
        print("[Startup] WARNING: Gagal kirim waktu ke Firebase")

# ===================== Serial Com =====================
# UART ke ESP32 via /dev/ttyAMA0  ← /dev/serial0 setelah dtoverlay=disable-bt
# Sisi ESP32: HardwareSerial rpSerial(0) — UART0, RX=18, TX=17, 115200 baud
#
# Syarat RPi:  /boot/firmware/config.txt harus ada:
#   enable_uart=1
#   dtoverlay=disable-bt
# Dan pastikan: ls -la /dev/serial0 → ttyAMA0  (bukan ttyS0)

SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200

_serial_ref: "serial.Serial | None" = None  # shared ref untuk write dari VPS client
_current_mode: int = 2  # 0=FWD, 1=RWD, 2=AWD

def open_serial() -> serial.Serial:
    """Buka koneksi serial ke ESP32, retry sampai berhasil."""
    while True:
        try:
            ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=SERIAL_BAUD,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,    # non-blocking, cukup responsif
            )
            print(f"[Serial] Terbuka: {SERIAL_PORT} @ {SERIAL_BAUD} baud")
            return ser
        except serial.SerialException as e:
            print(f"[Serial] Gagal buka {SERIAL_PORT}: {e} — retry 3s...")
            time.sleep(3)

def serial_loop(ser: serial.Serial):
    """
    Thread blocking: baca data dari ESP32 → print ke console.
    Write ke ESP32 dilakukan dari thread lain via _serial_ref.
    """
    global _serial_ref
    print("[Serial] Loop dimulai (background thread)")
    rx_buf = b""

    while True:
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except serial.SerialException as e:
            print(f"[Serial] Read error: {e} — reconnect...")
            try:
                ser.close()
            except Exception:
                pass
            ser = open_serial()
            _serial_ref = ser   # update shared ref setelah reconnect
            rx_buf = b""
            continue

        if chunk:
            rx_buf += chunk
            while b"\n" in rx_buf:
                line, rx_buf = rx_buf.split(b"\n", 1)
                decoded = line.decode("utf-8", errors="replace").strip()
                if decoded:
                    print(f"[ESP32->Pi] {decoded}")

# ===================== VPS WebSocket Client =====================
# Pi connect ke VPS sebagai "pi", terima perintah dari browser client
# Protokol vps.py:
#   - Pesan pertama "pi"  → dikenali sebagai Pi (handle_pi)
#   - Pesan pertama lain → dikenali sebagai client web
#
# Format perintah dari browser: JSON string atau Plain Text

async def send_frames(ws):
    global recording, last_record_report, record_request
    try:
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        while True:
            # Kirim status recording setiap detik
            current_time = time.monotonic()
            if current_time - last_record_report >= 1.0:
                status = "record1" if recording else "record2"
                await ws.send(status)
                last_record_report = current_time

            # Apply record state changes here (single Picamera2 access point)
            if record_request and not recording:
                await start_video_recording()
            elif (not record_request) and recording:
                await stop_video_recording()

            # Capture frame dari lores
            frame = picam2.capture_array("lores")
            bgr = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_I420)
            ret, jpeg = cv2.imencode('.jpg', bgr, encode_param)
            if not ret:
                continue
            payload = b'\x01' + jpeg.tobytes()
            await ws.send(payload)
            await asyncio.sleep(0.05)  # ~20fps
    except Exception as e:
        print(f"[Cam-Stream] Stream error: {e}")

async def receive_commands(ws):
    global record_request, _current_mode, exposure, gain, audio_task
    async for msg in ws:
        # Binary data dari server diabaikan
        if isinstance(msg, (bytes, bytearray)):
            continue

        if msg.startswith("exposure"):
            try:
                value = int(msg[len("exposure"):])
                exposure = value
                print(f"[Cam] Exposure set to {exposure}")
                if picam2:
                    picam2.set_controls({"ExposureTime": exposure})
            except ValueError:
                print("[Cam] Invalid exposure value")
            continue

        if msg.startswith("gain"):
            try:
                value = float(msg[len("gain"):]) / 1000.0
                gain = value
                print(f"[Cam] Gain set to {gain}")
                if picam2:
                    picam2.set_controls({"AnalogueGain": gain})
            except ValueError:
                print("[Cam] Invalid gain value")
            continue

        if msg.startswith("record"):
            cmd = msg.strip()
            if cmd == "record1":
                record_request = True
            elif cmd == "record2":
                record_request = False
            continue

        if msg.startswith("light1,"):
            try:
                status = int(msg.split(",")[1])
                if led_line:
                    led_line.set_value(1 if status else 0)
                    print(f"[GPIO] LED Pin {LED_PIN} set to {status}")
                else:
                    print("[GPIO] LED pin not initialized, cannot control LED")
            except ValueError:
                print("[VPS-CMD] Invalid light1 status")
            except Exception as e:
                print(f"[GPIO] Error setting LED pin value: {e}")
            continue

        # Handle JSON commands (movement, mode, audio)
        try:
            data = json.loads(msg)
            
            if "set_audio" in data:
                active = int(data["set_audio"])
                await update_audio_state("stream", active == 1, ws)
                continue

            if "set_mode" in data:
                new_mode = int(data["set_mode"])
                if new_mode in (0, 1, 2):
                    _current_mode = new_mode
                    # Send confirmation back to VPS
                    confirm_msg = json.dumps({"mode_confirm": "ok", "mode": _current_mode})
                    await ws.send(confirm_msg)
                    print(f"[VPS-WS] Mode updated to {new_mode} and confirmed")
                else:
                    fail_msg = json.dumps({"mode_confirm": "fail", "reason": "invalid mode"})
                    await ws.send(fail_msg)
                continue

            f = int(data.get("forward", 0))
            b = int(data.get("backward", 0))
            l = int(data.get("left", 0))
            r = int(data.get("right", 0))
            s = int(data.get("speed", 100))
            
            cmd = f"mov {f},{b},{l},{r},{s},{_current_mode}\n"
            
            if _serial_ref and _serial_ref.is_open:
                try:
                    _serial_ref.write(cmd.encode("utf-8"))
                    print(f"[VPS->ESP32] {cmd.strip()}")
                except serial.SerialException as e:
                    print(f"[Serial] Write error: {e}")
                    
        except json.JSONDecodeError:
            print(f"[VPS-CMD] Unknown text: {msg}")
        except Exception as e:
            print(f"[VPS-CMD] Error processing command: {e}")

async def vps_client():
    """Async task: terhubung ke VPS WS, identifikasi sebagai Pi, kirim frames & terima perintah."""
    while True:
        try:
            print(f"[VPS-WS] Connecting → {VPS_WS_URL}")
            async with websockets.connect(
                VPS_WS_URL,
                ping_interval=20,
                ping_timeout=10,
                max_size=2**23,
            ) as ws:
                await ws.send("pi")
                print("[VPS-WS] Connected — identified as Pi")

                consumer_task = asyncio.create_task(receive_commands(ws))
                producer_task = asyncio.create_task(send_frames(ws))

                done, pending = await asyncio.wait(
                    [consumer_task, producer_task],
                    return_when=asyncio.FIRST_COMPLETED,
                )
                for task in pending:
                    task.cancel()
                
                await update_audio_state("stream", False)

        except Exception as e:
            print(f"[VPS-WS] Disconnected: {e} — retry 3s...")
            await update_audio_state("stream", False)
            await asyncio.sleep(3)

# ===================== Web Server (FastAPI) =====================
# Serve web.html sebagai halaman statis di port 80
#
# Run:  sudo python3 zero.py
#       (port 80 butuh sudo / cap_net_bind_service di Linux)

@asynccontextmanager
async def lifespan(app: FastAPI):
    # ─── Startup ───────────────────────────────────────────────
    startup()

    global _serial_ref, picam2
    
    print("[Camera] Inisialisasi kamera...")
    try:
        picam2 = Picamera2(0)
        config = picam2.create_video_configuration(
            main={"size": (1280, 720), "format": "RGB888"},
            lores={"size": (256, 144), "format": "YUV420"},
            raw={"size": (1296, 972)}  # 2x2 bin → full FOV @ ~46fps
        )
        picam2.configure(config)
        picam2.start()
        picam2.set_controls({"AeEnable": False})
        print("[Camera] SUKSES: Kamera berhasil di-start!")
    except Exception as e:
        print(f"[Camera] ERROR: Gagal start kamera: {e}")

    ser = open_serial()
    _serial_ref = ser

    # Serial reader: background thread (blocking I/O)
    threading.Thread(target=serial_loop, args=(ser,), daemon=True).start()

    # VPS WS client: asyncio task (berjalan di loop yang sama dengan uvicorn)
    vps_task = asyncio.create_task(vps_client())

    yield  # ← server berjalan di sini

    # ─── Shutdown ──────────────────────────────────────────────
    vps_task.cancel()
    try:
        await vps_task
    except asyncio.CancelledError:
        pass
    # Clean up audio state
    try:
        await update_audio_state("stream", False)
        await update_audio_state("record", False)
        close_audio_wav()
    except Exception as e:
        print(f"[Main] Audio cleanup error: {e}")
    if ser.is_open:
        ser.close()
    if picam2:
        picam2.stop()
    # Clean up GPIO
    if 'led_line' in globals() and led_line:
        try:
            led_line.set_value(0)
            led_line.release()
            print("[GPIO] LED pin released")
        except Exception as e:
            print(f"[GPIO] Error releasing LED pin: {e}")
    if 'chip' in globals() and chip:
        try:
            chip.close()
            print("[GPIO] Chip closed")
        except Exception as e:
            print(f"[GPIO] Error closing GPIO chip: {e}")
    print("[Main] Shutdown selesai")


app = FastAPI(lifespan=lifespan)

@app.get("/")
async def index():
    """Serve web.html statis."""
    return FileResponse(WEB_HTML)

# ===================== Main =====================

if __name__ == "__main__":
    # port 80 butuh: sudo python3 zero.py
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="info")
