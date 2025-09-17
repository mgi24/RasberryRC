import asyncio
import contextlib
import sys
import websockets
import numpy as np

VPS_WS_URL = "ws://192.168.0.101:8765"

CARD = "plughw:2,0"
FMT = "S32_LE"    # S16_LE jika ingin 16-bit
RATE = 16000
CHANNELS = 1

# 20 ms/chunk → 320 frames @16kHz
CHUNK_FRAMES = 320
BYTES_PER_SAMPLE = 4   # 4 utk S32_LE, 2 utk S16_LE
CHUNK_SIZE = CHUNK_FRAMES * CHANNELS * BYTES_PER_SAMPLE

MAX_QUEUE_CHUNKS = 15  # ~300 ms max buffer (15 * 20ms)

async def stream_once():
    proc = await asyncio.create_subprocess_exec(
        "arecord",
        "-D", CARD,
        "-f", FMT,
        "-r", str(RATE),
        "-c", str(CHANNELS),
        "-t", "raw",
        "-q",
        "-B", "160000",   # buffer 160ms
        "-F", "20000",    # period 20ms
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.DEVNULL,
    )

    async with websockets.connect(
        VPS_WS_URL,
        max_size=None,
        ping_interval=10,
        ping_timeout=10,
        close_timeout=5,
    ) as ws:
        await ws.send("pi")

        q = asyncio.Queue(maxsize=MAX_QUEUE_CHUNKS)

        async def producer():
            try:
                while True:
                    chunk = await proc.stdout.readexactly(CHUNK_SIZE)
                    # jika penuh, drop chunk tertua (prioritas realtime)
                    if q.full():
                        try:
                            old = q.get_nowait()
                            q.task_done()
                        except asyncio.QueueEmpty:
                            pass
                    await q.put(chunk)
            except asyncio.IncompleteReadError:
                pass

        async def consumer():
            GAIN_DB = 8.0
            SHIFT_BITS = 16 
            GAIN = 10 ** (GAIN_DB / 20.0)  # ≈ 2.512
            while True:
                chunk = await q.get()
                try:
                    # Convert S32_LE to S16_LE + Amplify
                    i32 = np.frombuffer(chunk, dtype='<i4').copy()
                    # (Opsional) shift 8 bit jika INMP441 (24-bit in 32-bit)
                    i32 = i32 >> SHIFT_BITS
                    samples = i32.astype(np.float32)
                    np.clip(samples, -32768, 32767, out=samples)
                    i16 = samples.astype('<i2', copy=False)
                    await ws.send(b"\x02" + i16.tobytes())
                finally:
                    q.task_done()

        try:
            prod = asyncio.create_task(producer())
            cons = asyncio.create_task(consumer())
            await asyncio.wait([prod, cons], return_when=asyncio.FIRST_COMPLETED)
        finally:
            for t in (prod, cons):
                if not t.done():
                    t.cancel()

    with contextlib.suppress(ProcessLookupError):
        proc.terminate()
    with contextlib.suppress(Exception):
        await proc.wait()

async def main():
    backoff = 1
    while True:
        try:
            await stream_once()
            backoff = 1
        except (websockets.ConnectionClosed, websockets.InvalidState, OSError) as e:
            print(f"WS error: {e}", file=sys.stderr)
        except Exception as e:
            print(f"Error: {e}", file=sys.stderr)
        finally:
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 20)

if __name__ == "__main__":
    asyncio.run(main())