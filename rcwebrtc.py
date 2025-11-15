import os
import cv2
import av
import json
import time
import asyncio
import websockets
import numpy as np
from fractions import Fraction
from picamera2 import Picamera2

from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    VideoStreamTrack,
    RTCConfiguration,
    RTCIceServer,
    AudioStreamTrack,
)
from aiortc.sdp import candidate_from_sdp
from aiortc.rtcrtpsender import RTCRtpSender

SIGNALING_URL = os.getenv("SIGNALING_URL", "ws://168.110.218.135:8766")
import gpiod
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

def cleanup():
    print("Cleaning up GPIO...")
    set_lines(lines, 0)
    for line in lines:
        line.release()
    chip.close()
    
class ArecordAudioTrack(AudioStreamTrack):
    kind = "audio"
    def __init__(self, card="plughw:2,0", fmt="S32_LE", rate=16000, channels=1):
        super().__init__()
        self._arecord_task = asyncio.create_task(self._start_arecord(card, fmt, rate, channels))
        self.queue = asyncio.Queue(maxsize=50)  # lebih besar untuk jitter buffer
        self.rate = 16000            # target ke encoder (Opus akan upsample)
        self.channels = 1            # mono
        self._proc = None
        self._mono_buf = np.empty(0, dtype=np.int16)
        self._pts = 0
        self._time_base = Fraction(1, self.rate)
        # jitter buffer warmup
        self._warmup_frames = 10
        self._warmed = asyncio.Event()

    async def _start_arecord(self, card, fmt, rate, channels):
        # 20ms @ 16kHz, 32-bit source = 320 samples x 4 bytes = 1280 bytes
        src_bytes_per_sample = 4
        frame_src_samples = int(rate * 0.02)  # 320
        chunk = frame_src_samples * src_bytes_per_sample

        # Tuning ALSA: period 20ms, buffer 200ms untuk stabilitas
        period_us = 20000
        buffer_us = 200000

        self._proc = await asyncio.create_subprocess_exec(
            "arecord",
            "-D", card, "-f", fmt, "-r", str(rate), "-c", str(channels),
            "-t", "raw", "-q",
            "-F", str(period_us), "-B", str(buffer_us),
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL,
        )
        # 20ms @ 16kHz, 32-bit source = 320 samples x 4 bytes = 1280 bytes
        src_bytes_per_sample = 4
        frame_src_samples = int(rate * 0.02)  # 320
        chunk = frame_src_samples * src_bytes_per_sample

        GAIN_DB = 8.0
        SHIFT_BITS = 10            # INMP441 24-bit in 32-bit
        GAIN = 5 ** (GAIN_DB / 10.0)

        try:
            while True:
                try:
                    # pastikan selalu dapat 1280 bytes per 20ms
                    data = await self._proc.stdout.readexactly(chunk)
                except asyncio.IncompleteReadError as e:
                    data = e.partial
                    if not data:
                        print("[AUDIO] arecord EOF")
                        break
                # print(f"[AUDIO] read {len(data)} bytes from arecord")

                # S32_LE -> float32 -> gain -> clip -> int16 mono (16k)
                i32 = np.frombuffer(data, dtype="<i4").copy()
                i32 = i32 >> SHIFT_BITS
                f32 = i32.astype(np.float32) * GAIN
                np.clip(f32, -32768, 32767, out=f32)
                i16 = f32.astype("<i2", copy=False)

                # Buffering 20ms @ 16kHz = 320 sampel
                self._mono_buf = np.concatenate((self._mono_buf, i16))
                while self._mono_buf.size >= frame_src_samples:
                    block = self._mono_buf[:frame_src_samples]
                    self._mono_buf = self._mono_buf[frame_src_samples:]
                    try:
                        self.queue.put_nowait(block.tobytes())
                    except asyncio.QueueFull:
                        _ = await self.queue.get()
                        await self.queue.put(block.tobytes())
                        print(f"[AUDIO] queue full -> dropped oldest, queued {block.size} samples, qsize={self.queue.qsize()}")
                    if not self._warmed.is_set() and self.queue.qsize() >= self._warmup_frames:
                        self._warmed.set()
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"[AUDIO] capture error: {e}")

    async def recv(self):
        try:
            # tunggu warmup agar tidak underflow di awal
            if not self._warmed.is_set():
                await self._warmed.wait()

            # Ambil 20ms mono @16k; beri sedikit waktu sebelum fallback silence
            try:
                data = await asyncio.wait_for(self.queue.get(), timeout=0.06)  # >20ms sedikit
            except asyncio.TimeoutError:
                samples = int(self.rate * 0.02)  # 320
                data = (np.zeros(samples, dtype="<i2")).tobytes()
                print(f"[AUDIO] queue empty, sending silence {samples} samples ({len(data)}B), qsize={self.queue.qsize()}")

            if len(data) % 2 != 0:
                data = data[: len(data) - (len(data) % 2)]

            pcm = np.frombuffer(data, dtype="<i2")
            samples = pcm.shape[0]

            frame = av.AudioFrame(format="s16", layout="mono", samples=samples)
            frame.pts = self._pts
            frame.time_base = self._time_base
            frame.sample_rate = self.rate
            frame.planes[0].update(pcm.tobytes())  # mono packed

            self._pts += samples
            # Optional: log ringkas
            #print(f"[AUDIO] send {samples} @ {self.rate}Hz")
            return frame
        except Exception as e:
            print(f"[AUDIO] recv error: {e}")
            samples = int(self.rate * 0.02)
            pcm = np.zeros(samples, dtype="<i2")
            frame = av.AudioFrame(format="s16", layout="mono", samples=samples)
            frame.pts = self._pts
            frame.time_base = self._time_base
            frame.sample_rate = self.rate
            frame.planes[0].update(pcm.tobytes())
            self._pts += samples
            print(f"[AUDIO] sending fallback silence {samples} samples")
            return frame

    def stop(self):
        print("[AUDIO] Track stopped")
        if hasattr(self, "_arecord_task") and self._arecord_task:
            self._arecord_task.cancel()
        if self._proc and self._proc.returncode is None:
            try:
                self._proc.terminate()
            except Exception:
                pass
        super().stop()

exposure = 10000
def make_camera_track(
    device_index: int = 0,
    width: int = 142,   # 142x80 = 16:9 aspect ratio (80p)
    height: int = 80,
    fps: int = 24,
    preview: bool = False,
) -> VideoStreamTrack:
    class _PiCamTrack(VideoStreamTrack):
        kind = "video"
        def __init__(self):
            super().__init__()
            # Kamera utama (kamera 0)
            self.picam2 = Picamera2(1)
            config = self.picam2.create_video_configuration(
                main={"size": (width, height), "format": "RGB888"},
                raw={"size": (self.picam2.sensor_resolution)}
            )
            self.picam2.configure(config)
            self.picam2.start()
            self.picam2.set_controls({
                "AfMode": 0,        # Manual focus (0=manual, 2=auto)
                "LensPosition": 0.0, # 0.0=infinite focus, 10.0=closest focus
                "AeEnable": False,
                "ExposureTime": exposure
            })

            # Kamera kedua (kamera 1)
            self.picam2_2 = Picamera2(0)
            config2 = self.picam2_2.create_video_configuration(
                main={"size": (width, height), "format": "RGB888"},
                raw={"size": (self.picam2_2.sensor_resolution)}
            )
            self.picam2_2.configure(config2)
            self.picam2_2.start()
            self.picam2_2.set_controls({
                "AfMode": 0,        # Manual focus
                "LensPosition": 0.0, # Focus ke infinity
                "AeEnable": False,
                "ExposureTime": exposure
            })
            self._ts = 0
            self._time_base = Fraction(1, max(1, fps))
            self._frame_interval = 1.0 / max(1, fps)
            self.preview = preview
            self._win_ready = False

        async def recv(self):
            await asyncio.sleep(self._frame_interval)
            # Ambil frame dari kedua kamera
            frame1 = self.picam2.capture_array()
            frame2 = self.picam2_2.capture_array()
            # Flip kedua frame
            frame1 = cv2.flip(frame1, -1)
            frame2 = cv2.flip(frame2, -1)
            # Gabung horizontal
            combined = np.hstack((frame1, frame2))
            video_frame = av.VideoFrame.from_ndarray(combined, format="bgr24")
            video_frame.pts = self._ts
            video_frame.time_base = self._time_base
            self._ts += 1
            return video_frame

    return _PiCamTrack()
#audio_track: AudioStreamTrack
def create_peer_connection(camera_track: VideoStreamTrack, audio_track: None, ws, client_id: str) -> RTCPeerConnection:
    pc = RTCPeerConnection(
        RTCConfiguration(iceServers=[RTCIceServer(urls="stun:stun.l.google.com:19302")])
    )

    # Simpan strong reference agar tidak di-GC
    pc._local_tracks = []
    if camera_track: pc._local_tracks.append(camera_track)
    if audio_track: pc._local_tracks.append(audio_track)

    # Video sendonly
    vtx = pc.addTransceiver("video", direction="sendonly")
    vtx.sender.replaceTrack(camera_track)

    # Audio sendonly + force Opus
    if audio_track is not None:
        atx = pc.addTransceiver("audio", direction="sendonly")
        atx.sender.replaceTrack(audio_track)
        try:
            caps = RTCRtpSender.getCapabilities("audio")
            opus = [c for c in caps.codecs if c.mimeType.lower() == "audio/opus"]
            if opus:
                atx.setCodecPreferences(opus)
                print("[AUDIO] codec set to Opus")
        except Exception as e:
            print(f"[AUDIO] setCodecPreferences error: {e}")
    else:
        print("[WARN] No audio track available")

    @pc.on("iceconnectionstatechange")
    async def on_ice_state():
        print("[ICE] state:", pc.iceConnectionState)

    @pc.on("connectionstatechange")
    async def on_conn_state():
        print("[PC] state:", pc.connectionState)

    @pc.on("signalingstatechange")
    async def on_sig_state():
        print("[PC] signaling:", pc.signalingState)

    @pc.on("icecandidate")
    async def on_icecandidate(cand):
        if cand is None:
            return
        msg = {
            "type": "candidate",
            "client_id": client_id,
            "candidate": {
                "candidate": cand.to_sdp(),
                "sdpMid": cand.sdpMid,
                "sdpMLineIndex": cand.sdpMLineIndex,
            },
        }
        await ws.send(json.dumps(msg))

    return pc


async def handle_offer(pcs: dict, ws, camera_track: VideoStreamTrack, client_id: str, sdp: dict):
    try:
        #audio_track = ArecordAudioTrack(card="plughw:2,0", fmt="S32_LE", rate=16000, channels=1)
        audio_track = None
    except Exception as e:
        print(f"[WARN] Audio track init failed: {e}")
        audio_track = None

    pc = create_peer_connection(camera_track, audio_track, ws, client_id)
    pcs[client_id] = pc

    await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp["sdp"], type=sdp["type"]))
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    # print(answer)
    print(pc.localDescription.sdp)
    await ws.send(json.dumps({
        "type": "answer",
        "client_id": client_id,
        "sdp": {"type": pc.localDescription.type, "sdp": pc.localDescription.sdp},
    }))


async def add_candidate(pcs: dict, client_id: str, candidate: dict):
    """Tambahkan ICE candidate dari client ke PC terkait."""
    pc = pcs.get(client_id)
    if not pc:
        return
    cand_str = candidate.get("candidate")
    if not cand_str:
        await pc.addIceCandidate(None)
        return
    ice = candidate_from_sdp(cand_str)
    ice.sdpMid = candidate.get("sdpMid")
    ice.sdpMLineIndex = candidate.get("sdpMLineIndex")
    await pc.addIceCandidate(ice)

last_heartbeat = time.monotonic()
async def heartbeat_watcher():
    global last_heartbeat
    while True:
        if time.monotonic() - last_heartbeat > 0.5:
            # print("Heartbeat lost, stopping motors")
            set_lines(lines, 0)
        await asyncio.sleep(0.05)

async def run():
    # Ubah width/height/fps di sini untuk membatasi stream
    camera_track = make_camera_track(width=426, height=240, fps=24, preview=False)
    pcs: dict[str, RTCPeerConnection] = {}
    heartbeat_task = asyncio.create_task(heartbeat_watcher())

    async with websockets.connect(SIGNALING_URL) as ws:
        await ws.send(json.dumps({"type": "register", "role": "pi"}))
        print("Pi registered to signaling server:", SIGNALING_URL)

        try:
            async for msg in ws:
                try:
                    data = json.loads(msg)
                except Exception:
                    continue

                t = data.get("type")
                if t == "offer":
                    await handle_offer(pcs, ws, camera_track, data["client_id"], data["sdp"])
                elif t == "candidate":
                    await add_candidate(pcs, data["client_id"], data["candidate"])
                elif t == "bye":
                    cid = data.get("client_id")
                    pc = pcs.pop(cid, None)
                    if pc:
                        await pc.close()
                elif t == "control":
                    try:
                        print(data)
                        for direction, state in data.items():
                            if direction in PINS:
                                line = lines[list(PINS.keys()).index(direction)]
                                line.set_value(1 if state else 0)
                    except Exception as e:
                        print(f"GPIO control error: {e}")
                elif t == "ping":
                    global last_heartbeat
                    last_heartbeat = time.monotonic()
                elif t == "exposure":
                    try:
                        exposure_value = data.get("value")
                        if exposure_value is not None:
                            global exposure
                            exposure = int(exposure_value)
                            # Set exposure for both cameras
                            camera_track.picam2.set_controls({"ExposureTime": exposure})
                            camera_track.picam2_2.set_controls({"ExposureTime": exposure})
                            print(f"Exposure set to {exposure} for both cameras")
                    except Exception as e:
                        print(f"Exposure setting error: {e}")
                    
        finally:
            heartbeat_task.cancel()
            for pc in list(pcs.values()):
                await pc.close()
            pcs.clear()
            cleanup()  # clear GPIO before exit


def main():
    asyncio.run(run())


if __name__ == "__main__":
    main()