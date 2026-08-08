@echo off
setlocal EnableExtensions
cd /d "%~dp0"

rem ==========================================================
rem Quest Hand Pose Server - Conda environment launcher
rem ==========================================================

set "CONDA_BAT=D:\Anaconda\install\condabin\conda.bat"
set "DEFAULT_ENV=VR_Teleop"

if not exist "%CONDA_BAT%" (
    echo ==========================================================
    echo Conda launcher was not found:
    echo %CONDA_BAT%
    echo ==========================================================
    echo.
    echo Edit start_quest_hand_pose_server.bat and update CONDA_BAT.
    echo Typical locations:
    echo   D:\Anaconda\install\condabin\conda.bat
    echo   C:\Users\YOUR_NAME\anaconda3\condabin\conda.bat
    echo   C:\Users\YOUR_NAME\miniconda3\condabin\conda.bat
    echo.
    pause
    exit /b 1
)

echo ==========================================================
echo Available Conda environments
echo ==========================================================
call "%CONDA_BAT%" env list
echo.

set /p "CONDA_ENV=Enter environment name [%DEFAULT_ENV%]: "
if not defined CONDA_ENV set "CONDA_ENV=%DEFAULT_ENV%"

echo.
echo Activating Conda environment: %CONDA_ENV%
call "%CONDA_BAT%" activate "%CONDA_ENV%"

if errorlevel 1 (
    echo.
    echo Failed to activate Conda environment: %CONDA_ENV%
    echo Check the environment name shown above.
    pause
    exit /b 1
)

echo.
echo Active environment:
echo   %CONDA_DEFAULT_ENV%
echo Python executable:
python -c "import sys; print('  ' + sys.executable)"

if errorlevel 1 (
    echo.
    echo Python is unavailable inside environment: %CONDA_ENV%
    pause
    exit /b 1
)

echo.
echo ==========================================================
echo Quest Hand Pose Server
echo ==========================================================
echo 1. Start with GUI
echo 2. Start in terminal mode
echo 3. Install / update dependencies
echo 4. Test tkinter
echo 5. Exit
echo.

set /p "MODE=Select 1, 2, 3, 4, or 5: "

if "%MODE%"=="1" goto GUI
if "%MODE%"=="2" goto TERMINAL
if "%MODE%"=="3" goto INSTALL
if "%MODE%"=="4" goto TESTTK
if "%MODE%"=="5" goto END

echo Invalid selection.
pause
exit /b 1

:GUI
python quest_hand_pose_server.py --gui
goto FINISH

:TERMINAL
python quest_hand_pose_server.py --no-gui
goto FINISH

:INSTALL
python -m pip install --upgrade -r requirements.txt
goto FINISH

:TESTTK
python -m tkinter
goto FINISH

:FINISH
echo.
echo Program finished.
pause

:END
endlocal
