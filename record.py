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
    CARD = "plughw:2,0"
    FMT = "S32_LE"
    RATE = 16000
    CHANNELS = 1

    # Use raw PCM; save as .pcm
    output = Path("record_16k_32bit.wav")

    cmd = [
        "arecord",
        "-D", CARD,
        "-f", FMT,
        "-r", str(RATE),
        "-c", str(CHANNELS),
        "-d", "5",     # 5 seconds
        str(output)
    ]

    try:
        print(f"Recording 5s @ 16kHz 32-bit to {output} ...")
        subprocess.run(cmd, check=True)
        print("Done.")
    except subprocess.CalledProcessError as e:
        print(f"arecord failed with exit code {e.returncode}", file=sys.stderr)
        sys.exit(e.returncode)

if __name__ == "__main__":
    main()