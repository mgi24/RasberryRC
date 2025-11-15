import sounddevice as sd
from scipy.io.wavfile import write

duration = 5  # detik
samplerate = 16000  # Hz
channels = 1
device = None

# Temukan index device USB PnP Audio Device
for idx, info in enumerate(sd.query_devices()):
    if "USB PnP" in info['name'] and info['max_input_channels'] > 0:
        device = idx
        print(f"Using device: {info['name']} (index {idx})")
        break

if device is None:
    raise RuntimeError("USB PnP Audio Device not found!")

print("Recording...")
audio = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=channels, dtype='int16', device=device)
sd.wait()
write("recorded.wav", samplerate, audio)
print("Saved to recorded.wav")