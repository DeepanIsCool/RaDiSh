@echo off
echo ==========================================
echo     Lancer AV Simulator Setup Script (Win)
echo ==========================================

:: Check Python installation
where python >nul 2>nul
if %errorlevel% neq 0 (
    echo ❌ Error: Python is not installed or not in your PATH.
    echo Please install Python 3.10+ before running this script.
    pause
    exit /b 1
)

:: Recreate venv if it exists
if exist .venv (
    echo 🧹 Removing existing .venv directory to avoid path conflicts...
    rmdir /s /q .venv
)

:: Create virtual environment
echo 📦 Creating virtual environment...
python -m venv .venv

:: Activate and install requirements
echo 📥 Activating virtual environment and installing packages...
call .venv\Scripts\activate.bat
python -m pip install --upgrade pip
pip install -r requirements.txt

echo ==========================================
echo 🎉 Setup Complete!
echo ==========================================
echo To run the application, execute:
echo   .venv\Scripts\activate.bat
echo   python main.py
echo ==========================================
pause
