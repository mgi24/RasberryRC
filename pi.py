import asyncio
import websockets
import cv2

VPS_WS_URL = "ws://192.168.0.101:8765"  # Tanpa /pi, karena identifikasi pakai pesan pertama

async def _recv_print(ws):
    # Handler untuk baca pesan dari server dan print
    try:
        async for msg in ws:
            print(msg)
    except Exception as e:
        print(f"Receiver stopped: {e}")

async def send_mjpeg():
    cap = cv2.VideoCapture(0)
    try:
        async with websockets.connect(VPS_WS_URL, max_size=None) as ws:
            # Kirim pesan identifikasi terlebih dahulu
            await ws.send("pi")

            # Jalankan receiver di task terpisah
            recv_task = asyncio.create_task(_recv_print(ws))

            try:
                while True:
                    if recv_task.done():
                        break

                    ret, frame = cap.read()
                    if not ret:
                        break

                    ok, jpeg = cv2.imencode('.jpg', frame)
                    if not ok:
                        continue

                    jpeg_bytes = jpeg.tobytes()
                    payload = b'\x01' + jpeg_bytes  # prefix 0x01 to mark video stream
                    await ws.send(payload)
                    await asyncio.sleep(0.03)  # ~30 fps
            finally:
                if not recv_task.done():
                    recv_task.cancel()
                try:
                    await recv_task
                except asyncio.CancelledError:
                    pass
    finally:
        cap.release()

if __name__ == "__main__":
    asyncio.run(send_mjpeg())