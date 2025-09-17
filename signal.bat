@echo off
REM Aktifkan virtual environment
call venv\Scripts\activate

REM Jalankan VPS Python (ganti app.py dengan file utama Anda)
python signalrc.py

REM Nonaktifkan virtual environment setelah selesai
deactivate

pause