@echo off
cd /d "%~dp0"
set "PY=%LOCALAPPDATA%\Python\pythoncore-3.14-64\python.exe"
if not exist "%PY%" (
 echo Python executable not found: %PY%
 echo Edit the PY path in setup.cmd to point to your python.exe.
 pause
 exit /b 1
)
"%PY%" -m venv .venv
if errorlevel 1 goto end
".venv\Scripts\python.exe" -m pip install -r requirements.txt
:end
pause
