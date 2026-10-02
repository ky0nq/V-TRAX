@echo off
setlocal
cd /d "%~dp0"

set "PY="
for %%P in (
 "%LOCALAPPDATA%\Programs\Python\Python314\python.exe"
 "%LOCALAPPDATA%\Programs\Python\Python313\python.exe"
 "%LOCALAPPDATA%\Python\pythoncore-3.14-64\python.exe"
 "%LOCALAPPDATA%\Python\pythoncore-3.13-64\python.exe"
 "C:\Python314\python.exe"
 "C:\Python313\python.exe"
) do (
 if not defined PY if exist "%%~P" set "PY=%%~P"
)

if not defined PY (
 echo Python executable was not found in the common install locations.
 echo Edit setup.cmd and set PY to the full path of python.exe.
 pause
 exit /b 1
)

echo Using Python: %PY%
"%PY%" -m venv .venv
if errorlevel 1 goto :end
".venv\Scripts\python.exe" -m pip install --upgrade pip
".venv\Scripts\python.exe" -m pip install -r requirements.txt
:end
pause
