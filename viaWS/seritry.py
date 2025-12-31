import serial
import time

# Serial port settings for Raspberry Pi default UART (GPIO 14=TX, 15=RX)
SERIAL_PORT = '/dev/serial0'
BAUDRATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)

try:
    while True:
        # Send "hello world"
        ser.write(b'hello world\n')
        time.sleep(1)
        # Read all available responses
        while ser.in_waiting:
            response = ser.readline().decode(errors='ignore').strip()
            if response:
                print("Received:", response)
except KeyboardInterrupt:
    pass
finally:
    ser.close()