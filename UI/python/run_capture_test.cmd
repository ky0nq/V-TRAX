@echo off
cd /d "%~dp0"
set "CAPTURE_DEVICE=auto"
if not "%~1"=="" set "CAPTURE_DEVICE=%~1"
".venv\Scripts\python.exe" main.py --capture-device "%CAPTURE_DEVICE%" --stats
pause
