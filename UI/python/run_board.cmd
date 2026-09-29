@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
 echo Run setup.cmd first.
 pause
 exit /b 1
)
".venv\Scripts\python.exe" main.py --udp --host 0.0.0.0 --port 7000 --video-port 7001 --board-ip 192.168.10.2 --stats
pause
