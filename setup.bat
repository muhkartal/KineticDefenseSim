@echo off
REM Kinetic Defense Simulation - Environment Setup

cd /d "%~dp0"

echo [SETUP] Checking for Python...
python --version >nul 2>&1
IF %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Python not found in PATH. Please install Python 3.10+.
    pause
    exit /b 1
)

echo [SETUP] Creating Virtual Environment (.venv)...
IF EXIST .venv (
    rmdir /s /q .venv
)
python -m venv .venv
IF %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Failed to create venv.
    pause
    exit /b 1
)

echo [SETUP] Upgrading PIP...
.venv\Scripts\python -m pip install --upgrade pip

echo [SETUP] Installing Dependencies...
.venv\Scripts\pip install -r requirements.txt
IF %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Failed to install dependencies.
    pause
    exit /b 1
)

echo [SUCCESS] Environment Ready. Run 'run_sim.bat' to start.
pause