@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title Quest 3 Controller Signal Server

rem ============================================================
rem Locate Conda
rem ============================================================

set "CONDA_BAT="

for %%I in (conda.bat) do (
    if not "%%~$PATH:I"=="" set "CONDA_BAT=%%~$PATH:I"
)

if not defined CONDA_BAT if exist "D:\Anaconda\install\condabin\conda.bat" (
    set "CONDA_BAT=D:\Anaconda\install\condabin\conda.bat"
)

if not defined CONDA_BAT if exist "%USERPROFILE%\anaconda3\condabin\conda.bat" (
    set "CONDA_BAT=%USERPROFILE%\anaconda3\condabin\conda.bat"
)

if not defined CONDA_BAT if exist "%USERPROFILE%\miniconda3\condabin\conda.bat" (
    set "CONDA_BAT=%USERPROFILE%\miniconda3\condabin\conda.bat"
)

if not defined CONDA_BAT (
    cls
    echo ============================================================
    echo       Quest 3 Controller Signal Server
    echo ============================================================
    echo.
    echo [ERROR] conda.bat was not found.
    echo.
    echo Checked:
    echo   PATH
    echo   D:\Anaconda\install\condabin\conda.bat
    echo   %USERPROFILE%\anaconda3\condabin\conda.bat
    echo   %USERPROFILE%\miniconda3\condabin\conda.bat
    echo.
    echo Edit this BAT file if Conda is installed somewhere else.
    echo.
    pause
    exit /b 1
)

rem ============================================================
rem User selects the Conda environment
rem ============================================================

:SELECT_ENV
cls
echo ============================================================
echo       Quest 3 Controller Signal Server
echo ============================================================
echo.
echo Available Conda environments:
echo ------------------------------------------------------------
call "%CONDA_BAT%" env list
echo ------------------------------------------------------------
echo.
echo Enter the environment NAME shown above.
echo Example: VR_Teleop
echo.
set "SELECTED_ENV="
set /p SELECTED_ENV=Conda environment: 

if not defined SELECTED_ENV (
    echo.
    echo [ERROR] No environment was entered.
    timeout /t 2 >nul
    goto SELECT_ENV
)

echo.
echo Activating Conda environment: %SELECTED_ENV%
call "%CONDA_BAT%" activate "%SELECTED_ENV%"

if errorlevel 1 (
    echo.
    echo [ERROR] Failed to activate: %SELECTED_ENV%
    echo Check that the environment name is correct.
    echo.
    pause
    goto SELECT_ENV
)

if /I not "%CONDA_DEFAULT_ENV%"=="%SELECTED_ENV%" (
    echo.
    echo [WARNING] Conda reported the active environment as:
    echo           %CONDA_DEFAULT_ENV%
    echo.
    echo Requested:
    echo           %SELECTED_ENV%
    echo.
    set /p CONTINUE_ANYWAY=Continue anyway? Type Y to continue: 
    if /I not "%CONTINUE_ANYWAY%"=="Y" goto SELECT_ENV
)

echo.
echo Environment activated successfully.
echo Active environment: %CONDA_DEFAULT_ENV%
echo Python executable:
where python
echo.
pause


rem ============================================================
rem Main menu
rem ============================================================

:MENU
cls
echo ============================================================
echo       Quest 3 Controller Signal Server
echo ============================================================
echo.
echo Selected Conda environment: %CONDA_DEFAULT_ENV%
echo.
echo   1. Terminal Mode
echo   2. GUI Mode
echo   3. Connection Test
echo   4. Install Dependencies
echo   5. Exit
echo.
set "CHOICE="
set /p CHOICE=Select 1, 2, 3, 4, or 5: 

if "%CHOICE%"=="1" goto TERMINAL_MODE
if "%CHOICE%"=="2" goto GUI_MODE
if "%CHOICE%"=="3" goto CONNECTION_TEST
if "%CHOICE%"=="4" goto INSTALL_DEPENDENCIES
if "%CHOICE%"=="5" goto EXIT_PROGRAM

echo.
echo Invalid selection.
timeout /t 2 >nul
goto MENU


rem ============================================================
rem Helper: check ADB
rem ============================================================

:CHECK_ADB
where adb >nul 2>nul
if errorlevel 1 (
    echo.
    echo [ERROR] adb was not found in PATH.
    echo Open Meta Quest Developer Hub once, or add Android platform-tools to PATH.
    echo.
    exit /b 1
)
exit /b 0


rem ============================================================
rem Helper: USB forwarding
rem ============================================================

:SET_USB_FORWARDING
call :CHECK_ADB
if errorlevel 1 exit /b 1

echo Configuring USB forwarding:
echo   adb reverse tcp:8766 tcp:8766
adb reverse tcp:8766 tcp:8766

if errorlevel 1 (
    echo.
    echo [ERROR] adb reverse failed.
    echo Check:
    echo   1. Quest is connected through USB
    echo   2. Developer Mode is enabled
    echo   3. USB debugging permission is accepted
    echo.
    exit /b 1
)

exit /b 0


rem ============================================================
rem Helper: base Python dependencies
rem ============================================================

:CHECK_BASE_DEPENDENCIES
python -c "import numpy, websockets" >nul 2>nul
if errorlevel 1 (
    echo.
    echo [ERROR] numpy or websockets is missing in:
    echo         %CONDA_DEFAULT_ENV%
    echo.
    echo Return to the menu and select option 4.
    echo.
    exit /b 1
)
exit /b 0


rem ============================================================
rem Option 1: Terminal Mode
rem ============================================================

:TERMINAL_MODE
cls
echo ============================================================
echo   Controller Signal - Terminal Mode
echo ============================================================
echo.
echo Conda environment: %CONDA_DEFAULT_ENV%
echo.

call :CHECK_BASE_DEPENDENCIES
if errorlevel 1 (
    pause
    goto MENU
)

call :SET_USB_FORWARDING
if errorlevel 1 (
    pause
    goto MENU
)

echo.
echo Starting Terminal Server:
echo   ws://127.0.0.1:8766
echo.
echo Press Ctrl+C to stop.
echo.

python quest_controller_signal_server.py --mode terminal

echo.
echo Terminal server stopped.
pause
goto MENU


rem ============================================================
rem Option 2: GUI Mode
rem ============================================================

:GUI_MODE
cls
echo ============================================================
echo   Controller Signal - GUI Mode
echo ============================================================
echo.
echo Conda environment: %CONDA_DEFAULT_ENV%
echo.

call :CHECK_BASE_DEPENDENCIES
if errorlevel 1 (
    pause
    goto MENU
)

python -c "import tkinter, matplotlib" >nul 2>nul
if errorlevel 1 (
    echo.
    echo [ERROR] tkinter or matplotlib is missing or broken in:
    echo         %CONDA_DEFAULT_ENV%
    echo.
    echo Return to the menu and select option 4.
    echo.
    pause
    goto MENU
)

call :SET_USB_FORWARDING
if errorlevel 1 (
    pause
    goto MENU
)

echo.
echo Starting GUI Server:
echo   ws://127.0.0.1:8766
echo.
echo Close the GUI window to stop the server.
echo.

python quest_controller_signal_server.py --mode gui

echo.
echo GUI server closed.
pause
goto MENU


rem ============================================================
rem Option 3: Connection Test
rem ============================================================

:CONNECTION_TEST
cls
echo ============================================================
echo   Quest Controller Signal - Connection Test
echo ============================================================
echo.

echo [1/5] Selected Conda environment
echo       %CONDA_DEFAULT_ENV%
echo.
echo       Python executable:
where python
echo.

echo [2/5] Python version
python --version
echo.

echo [3/5] Python dependencies
python -c "import numpy, websockets; print('       numpy and websockets: PASSED')" 2>nul
if errorlevel 1 (
    echo       numpy and websockets: FAILED
    echo       Select option 4 to install them.
)

python -c "import tkinter, matplotlib; print('       tkinter and matplotlib: PASSED')" 2>nul
if errorlevel 1 (
    echo       tkinter and matplotlib: FAILED
    echo       Select option 4 to install or repair them.
)
echo.

echo [4/5] ADB device
call :CHECK_ADB
if errorlevel 1 (
    echo       ADB: FAILED
) else (
    adb devices
)
echo.

echo [5/5] USB port forwarding
adb reverse tcp:8766 tcp:8766
if errorlevel 1 (
    echo       Port forwarding: FAILED
) else (
    echo       Port forwarding: PASSED
    adb reverse --list
)
echo.

echo Connection test completed.
pause
goto MENU


rem ============================================================
rem Option 4: Install dependencies into selected environment
rem ============================================================

:INSTALL_DEPENDENCIES
cls
echo ============================================================
echo   Install Controller Signal Server Dependencies
echo ============================================================
echo.
echo Dependencies will be installed into:
echo   %CONDA_DEFAULT_ENV%
echo.
echo Python executable:
where python
echo.
pause

echo [1/3] Installing or repairing Tkinter...
call conda install -n "%CONDA_DEFAULT_ENV%" tk -y

if errorlevel 1 (
    echo.
    echo [ERROR] Tk installation failed.
    pause
    goto MENU
)

echo.
echo [2/3] Installing Python packages...
python -m pip install -r requirements.txt

if errorlevel 1 (
    echo.
    echo [ERROR] Python package installation failed.
    pause
    goto MENU
)

echo.
echo [3/3] Testing imports...
python -c "import tkinter, numpy, matplotlib, websockets; print('All dependencies are ready.')"

if errorlevel 1 (
    echo.
    echo [WARNING] Import test failed.
    echo Attempting a forced Tk repair in:
    echo   %CONDA_DEFAULT_ENV%
    echo.
    call conda install -n "%CONDA_DEFAULT_ENV%" --force-reinstall tk -y
    python -c "import tkinter, numpy, matplotlib, websockets; print('All dependencies are ready.')"
)

echo.
echo Dependency installation completed for:
echo   %CONDA_DEFAULT_ENV%
echo.
pause
goto MENU


:EXIT_PROGRAM
endlocal
exit /b 0
