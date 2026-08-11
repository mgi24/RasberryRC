# Simple HTTP file server
# Jalankan: py server.py --dir "E:\KERJA\ARDUINO\ac\updatefirmware" --port 8000

from email import parser
import os
import argparse
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

def main():
    # Konfigurasi variabel
    SERVE_DIR = os.getcwd()
    PORT = 8080

    os.chdir(SERVE_DIR)
    server = ThreadingHTTPServer(("0.0.0.0", PORT), SimpleHTTPRequestHandler)
    print(f"Serving folder: {SERVE_DIR}")
    print(f"URL: http://localhost:{PORT}/")
    print("Tekan Ctrl+C untuk berhenti.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping...")
        server.server_close()
if __name__ == "__main__":
    main()
