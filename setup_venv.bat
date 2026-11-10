@echo off
setlocal

echo ========================================================
echo   Setting up Virtual Environment for SwarmControl
echo ========================================================

:: Check if Python is installed and accessible
where python >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo [ERROR] Python was not found in PATH. Please install Python 3.8+ and try again.
    pause
    exit /b 1
)

:: Create virtual environment if it does not already exist
if not exist "venv" (
    echo [INFO] Creating virtual environment (.venv)...
    python -m venv venv
    if %ERRORLEVEL% neq 0 (
        echo [ERROR] Failed to create virtual environment.
        pause
        exit /b 1
    )
    echo [INFO] Virtual environment created successfully.
) else (
    echo [INFO] Virtual environment 'venv' already exists.
)

:: Upgrade pip and install requirements
echo [INFO] Activating virtual environment and installing dependencies...
call venv\Scripts\activate.bat
python -m pip install --upgrade pip
pip install -r requirements.txt

if %ERRORLEVEL% neq 0 (
    echo [ERROR] Failed to install dependencies.
    pause
    exit /b 1
)

echo.
echo ========================================================
echo   Setup Complete!
echo   To activate your virtual environment in a shell:
echo       .\venv\Scripts\activate
echo   Then launch SwarmControl:
echo       python DroneContUI.py --real --drones COM3:1 COM3:2
echo ========================================================
pause
