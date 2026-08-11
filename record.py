import subprocess
import shutil
import sys
from pathlib import Path

#!/usr/bin/env python3

def main():
    if not shutil.which("arecord"):
        print("Error: 'arecord' not found. Install ALSA utils.", file=sys.stderr)
        sys.exit(1)


    # Settings per request
    CARD = "plughw:1,0"
    FMT = "S16_LE"
    RATE = 48000
    CHANNELS = 1

    # Use raw PCM; save as .pcm
    output = Path("record_48k_16bit.wav")

    cmd = [
        "arecord",
        "-D", CARD,
        "-f", FMT,
        "-r", str(RATE),
        "-c", str(CHANNELS),
        "-d", "10",    # 10 seconds
        str(output)
    ]

    try:
        print(f"Recording 10s @ 48kHz 16-bit to {output} ...")
        subprocess.run(cmd, check=True)
        print("Done.")
    except subprocess.CalledProcessError as e:
        print(f"arecord failed with exit code {e.returncode}", file=sys.stderr)
        sys.exit(e.returncode)

if __name__ == "__main__":
    main()