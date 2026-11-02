@echo off
chcp 65001 >nul
cd /d "%~dp0"
where py >nul 2>&1
if not errorlevel 1 (
    set "PY=py -3"
) else (
    set "PY=python"
)
%PY% -c "import serial" >nul 2>&1
if errorlevel 1 (
    echo Installing pyserial ...
    %PY% -m pip install -r requirements.txt
    if errorlevel 1 (
        echo Failed to install pyserial. Please install Python 3 and run again.
        pause
        exit /b 1
    )
)
%PY% app.py
if errorlevel 1 pause
