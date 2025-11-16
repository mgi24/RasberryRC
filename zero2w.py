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
    "right": 23,
    "light":24
}

CHIP_NAME = "gpiochip0"
chip = gpiod.Chip(CHIP_NAME)
lines = []

def set_lines(lines, value):
    for line in lines:
        line.set_value(value)

# Setup all GPIO lines at startup
for direction, pin in PINS.items():
    line = chip.get_line(pin)
    line.request(consumer="rc", type=gpiod.LINE_REQ_DIR_OUT)
    lines.append(line)
    set_lines(lines, 0)



def cleanup():
    print("Cleaning up GPIO...")
    set_lines(lines, 0)
    for line in lines:
        line.release()
    chip.close()
    
exposure = 10000
gain_val = 1.0
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
            self.picam2 = Picamera2(0)
            config = self.picam2.create_video_configuration(
                main={"size": (width, height), "format": "RGB888"},
                raw={"size": (self.picam2.sensor_resolution)}
            )
            self.picam2.configure(config)
            self.picam2.start()
            self.picam2.set_controls({
                "AeEnable": True,
                "ExposureTime": exposure,
                "AnalogueGain": float(gain_val)
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
            # Gabung horizontal
            video_frame = av.VideoFrame.from_ndarray(frame1, format="bgr24")
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

    # Video sendonly
    vtx = pc.addTransceiver("video", direction="sendonly")
    vtx.sender.replaceTrack(camera_track)

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

    #Handle all websocket signaling
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
                            print(f"Exposure set to {exposure} for both cameras")
                    except Exception as e:
                        print(f"Exposure setting error: {e}")
                elif t == "light":
                    try:
                        light_value = data.get("value")
                        print("Light value received:", light_value)
                        if light_value is not None:
                            line = lines[list(PINS.keys()).index("light")]
                            line.set_value(1 if light_value else 0)
                            print(f"Light set to {light_value}")
                    except Exception as e:
                        print(f"Light control error: {e}")
                    
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