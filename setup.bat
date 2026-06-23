@echo off
title Pathan Steel - Raster to DXF Laser Converter - Setup

echo ============================================
echo    RASTER IMAGE  -^>  MACHINE-READY DXF
echo    Pathan Steel - One Click Setup
echo ============================================
echo.

echo [1/5] Checking Python installation...
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python nahi mila system me.
    echo         https://www.python.org/downloads/ se install karo.
    echo         Install ke time "Add Python to PATH" tick zaroor karna.
    echo.
    pause
    exit /b 1
)
python --version
echo [OK] Python found.
echo.

echo [2/5] Creating virtual environment (.venv)...
if exist ".venv\Scripts\activate.bat" (
    echo [INFO] .venv pehle se mojood hai, usi ko use kar rahe hain.
) else (
    python -m venv .venv
    if errorlevel 1 (
        echo [ERROR] Virtual environment banane me dikkat aayi.
        pause
        exit /b 1
    )
)
echo [OK] Virtual environment ready.
echo.

echo [3/5] Activating virtual environment...
call ".venv\Scripts\activate.bat"
if errorlevel 1 (
    echo [ERROR] venv activate nahi hua.
    pause
    exit /b 1
)
echo [OK] Activated.
echo.

echo [4/5] Upgrading pip and installing all packages...
echo       (Pehli baar me 2-4 minute lag sakte hain, internet on rakho)
python -m pip install --upgrade pip
pip install -r requirements.txt
if errorlevel 1 (
    echo [ERROR] Package install fail hua. Internet check karo aur setup.bat dobara chalao.
    pause
    exit /b 1
)
echo [OK] Saare packages install ho gaye.
echo.

echo [5/5] Launching the application...
echo ============================================
echo  App browser me khul raha hai...
echo  Band karne ke liye is window me Ctrl + C dabao.
echo ============================================
echo.
streamlit run app.py

echo.
echo App band ho gaya. Dobara chalane ke liye setup.bat phir se run karo.
pause