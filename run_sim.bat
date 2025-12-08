@echo off
REM Kinetic Defense Simulation - Launcher

cd /d "%~dp0"

IF NOT EXIST .venv (
    echo [ERROR] Virtual environment not found. Please run setup.bat first.
    exit /b 1
)

:menu
cls
echo ===================================================
echo   KINETIC DEFENSE SIMULATION
echo ===================================================
echo.
echo Select Mode:
echo 1. Standard 3-DOF (Visually Rich, Randomized)
echo 2. Research 6-DOF (High Fidelity Physics)
echo 3. Exit
echo.

set /p choice="Enter Choice (1-3): "

if "%choice%"=="1" goto menu_3dof
if "%choice%"=="2" goto run_6dof
if "%choice%"=="3" exit /b 0
goto menu

:menu_3dof
cls
echo [3-DOF MODE] Select Scenario:
echo 1. Random
echo 2. Dogfight (Drone)
echo 3. Ballistic (Rocket)
echo 4. Replay Seed
echo 5. Back
echo.
set /p subchoice="Enter Choice (1-5): "

if "%subchoice%"=="1" set ARGS=--mode random
if "%subchoice%"=="2" set ARGS=--mode dogfight
if "%subchoice%"=="3" set ARGS=--mode ballistic
if "%subchoice%"=="4" (
    set /p seed="Enter Seed ID: "
    set ARGS=--seed %seed%
)
if "%subchoice%"=="5" goto menu

echo.
echo [RUN] Launching 3-DOF Simulation...
.venv\Scripts\python main.py %ARGS%
pause
goto menu

:run_6dof
echo.
echo [RUN] Launching 6-DOF High Fidelity Simulation...
.venv\Scripts\python main_6dof.py
pause
goto menu
