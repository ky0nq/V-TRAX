@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
 echo Run setup.cmd first.
 pause
 exit /b 1
)
set "CAPTURE_DEVICE=auto"
if not "%~3"=="" set "CAPTURE_DEVICE=%~3"
if "%~1"=="" (
 echo Usage: run_car_ui.cmd ZYBO_COM_PORT [ESP32_CAM_STREAM_URL ^| OFF] [CAPTURE_DEVICE]
 echo Example: run_car_ui.cmd COM4
 pause
 exit /b 1
)
if /I "%~2"=="OFF" (
 ".venv\Scripts\python.exe" main.py --serial-port "%~1" --capture-device "%CAPTURE_DEVICE%" --stats
) else (
 if "%~2"=="" (
  ".venv\Scripts\python.exe" main.py --serial-port "%~1" --capture-device "%CAPTURE_DEVICE%" --esp32-cam-url auto --stats
 ) else (
  ".venv\Scripts\python.exe" main.py --serial-port "%~1" --capture-device "%CAPTURE_DEVICE%" --esp32-cam-url "%~2" --stats
 )
)
pause
