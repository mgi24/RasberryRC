import argparse
import shutil
import subprocess
import sys
import time

def check_bin(name: str):
    if shutil.which(name) is None:
        print(f"'{name}' not found. Install it first.")
        sys.exit(1)

def record(output: str, width=1280, height=720, fps=30, bitrate=3_000_000, duration_ms=60_000, alsa_device="plughw:1,0"):
    # Pastikan tools tersedia
    check_bin("libcamera-vid")
    check_bin("ffmpeg")

    # Opsional: pastikan hardware codec aktif
    try:
        subprocess.run(["/sbin/modprobe", "bcm2835-codec"], check=False)
    except Exception:
        pass

    libcam_cmd = [
        "libcamera-vid",
        "-t", str(duration_ms),
        "-n",
        "--width", str(width),
        "--height", str(height),
        "--framerate", str(fps),
        "--codec", "h264",
        "--bitrate", str(bitrate),
        "--inline",
        "-o", "-"  # stream H.264 ke stdout
    ]

    ffmpeg_cmd = [
        "ffmpeg", "-y", "-hide_banner", "-loglevel", "warning",
        "-fflags", "+genpts",
        "-r", str(fps),
        "-thread_queue_size", "2048", "-f", "h264", "-i", "pipe:0",
        "-thread_queue_size", "1024", "-f", "alsa", "-ar", "16000", "-ac", "1", "-i", alsa_device,
        "-shortest",
        "-c:v", "copy",                # copy H.264 dari libcamera-vid (HW encoded)
        "-c:a", "aac", "-b:a", "96k",
        "-movflags", "+faststart",
        output
    ]

    print("Starting recording...")
    print("Video:", f"{width}x{height}@{fps} H.264 {bitrate}bps")
    print("Audio:", f"ALSA {alsa_device} 16kHz mono")
    print("Output:", output)

    libcam = None
    ffm = None
    try:
        libcam = subprocess.Popen(libcam_cmd, stdout=subprocess.PIPE)
        ffm = subprocess.Popen(ffmpeg_cmd, stdin=libcam.stdout)
        # Tutup stdout di parent agar ffmpeg dapat EOF saat libcamera-vid selesai
        if libcam and libcam.stdout:
            libcam.stdout.close()
        # Tunggu kedua proses
        rc_ffm = ffm.wait()
        rc_cam = libcam.wait()
        if rc_cam != 0:
            print(f"libcamera-vid exited with code {rc_cam}")
        if rc_ffm != 0:
            print(f"ffmpeg exited with code {rc_ffm}")
        if rc_cam == 0 and rc_ffm == 0:
            print("Recording completed.")
    except KeyboardInterrupt:
        print("Interrupted, stopping...")
    finally:
        # Cleanup proses
        for p in (ffm, libcam):
            if p and p.poll() is None:
                try:
                    p.terminate()
                    time.sleep(0.3)
                    if p.poll() is None:
                        p.kill()
                except Exception:
                    pass

def main():
    ap = argparse.ArgumentParser(description="Record 1 minute H.264 (HW) + USB audio to MP4")
    ap.add_argument("-o", "--output", default="out_720p.mp4")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--bitrate", type=int, default=3_000_000)
    ap.add_argument("--duration-ms", type=int, default=60_000)
    ap.add_argument("--alsa-device", default="plughw:1,0", help="arecord -l to find the right card,device")
    args = ap.parse_args()

    record(
        output=args.output,
        width=args.width,
        height=args.height,
        fps=args.fps,
        bitrate=args.bitrate,
        duration_ms=args.duration_ms,
        alsa_device=args.alsa_device,
    )

if __name__ == "__main__":
    main()