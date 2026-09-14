@echo off
REM ===========================================================================
REM  GPS_Localize - one click, Windows
REM ===========================================================================
REM  Windows cannot run ROS 2 Humble, so this file does the two things a
REM  Windows PC can do:
REM
REM    1. open the CONTROL page of a robot that is already running on the
REM       network  (type its IP address when asked)  -> full control
REM    2. or start the local VIEWER  -> view only, cannot drive the robot
REM
REM  The robot software itself must run on the Ubuntu PC:
REM      ./setup/setup_and_run.sh
REM ===========================================================================
setlocal enabledelayedexpansion
title GPS_Localize
cd /d "%~dp0.."

echo.
echo   ###########################################################
echo   #     GPS_Localize   -   Windows launcher                 #
echo   ###########################################################
echo.
echo   [check] project folder : %CD%

REM The address other devices on this Wi-Fi should open. Ask the routing table
REM which interface actually reaches the internet - picking the "first" adapter
REM returns VirtualBox or a VPN, which no phone can reach.
for /f "delims=" %%i in ('powershell -NoProfile -Command ^
  "(Find-NetRoute -RemoteIPAddress 8.8.8.8 -ErrorAction SilentlyContinue ^| Select-Object -First 1).IPAddress" 2^>nul') do set "MYIP=%%i"
if defined MYIP (
    echo   [ ok  ] this PC       : %MYIP%
) else (
    echo   [warn ] this PC       : could not detect the network address
)

if exist "webapp\exe\GPS_Localize_Viewer.exe" (
    echo   [ ok  ] viewer app     : webapp\exe\GPS_Localize_Viewer.exe
    set "VIEWER=exe"
) else (
    echo   [warn ] viewer app     : not built yet
    set "VIEWER="
)

where python >nul 2>&1
if %errorlevel%==0 (
    for /f "tokens=*" %%v in ('python --version 2^>^&1') do echo   [ ok  ] python        : %%v
    set "HAVEPY=1"
) else (
    echo   [warn ] python        : not installed ^(only needed to build the viewer^)
    set "HAVEPY="
)

if not defined VIEWER if defined HAVEPY (
    echo.
    set /p BUILD="   Build the viewer app now? [Y/n] "
    if /i not "!BUILD!"=="n" (
        call webapp\build_exe.bat
        if exist "webapp\exe\GPS_Localize_Viewer.exe" set "VIEWER=exe"
    )
)

echo.
echo   ---------------------------------------------------------------
echo    ควบคุมหุ่นยนต์จริง / TO CONTROL THE ROBOT
echo      พิมพ์ไอพีของเครื่อง Ubuntu ที่รันหุ่นอยู่ เช่น 192.168.1.50
echo      Type the IP of the Ubuntu PC that runs the robot, e.g. 192.168.1.50
echo.
echo    ดูโปรแกรมเฉย ๆ / JUST LOOK AT THE APP
echo      กด Enter เพื่อเปิดโหมดดูอย่างเดียว ^(ควบคุมหุ่นไม่ได้^)
echo      Press Enter for the local VIEW ONLY app
echo   ---------------------------------------------------------------
echo.
set /p ROBOTIP="   Robot IP (or Enter): "

if not "%ROBOTIP%"=="" (
    echo %ROBOTIP% | findstr /r "^http" >nul
    if errorlevel 1 (set "URL=http://%ROBOTIP%:8080") else (set "URL=%ROBOTIP%")
    echo   opening !URL!
    start "" "!URL!"
    echo.
    echo   If the page does not load: the Ubuntu PC must be running
    echo     ros2 launch gps_localize bringup.launch.py
    echo   and both machines must be on the same Wi-Fi.
    echo.
    pause
    exit /b 0
)

if defined VIEWER (
    echo   starting the VIEW ONLY app...
    "webapp\exe\GPS_Localize_Viewer.exe" %*
) else if defined HAVEPY (
    echo   starting the VIEW ONLY app from source...
    python webapp\viewer_server.py %*
) else (
    echo.
    echo   Nothing to run: install Python 3 from https://www.python.org/downloads/
    echo   or copy a folder that already contains webapp\exe\GPS_Localize_Viewer.exe
    echo.
)

pause
