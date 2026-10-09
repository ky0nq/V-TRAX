@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
 echo Run setup.cmd first.
 pause
 exit /b 1
)
title Zybo COM4 - Board Terminal
".venv\Scripts\python.exe" -u main.py %*
