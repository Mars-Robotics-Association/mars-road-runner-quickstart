@echo off
setlocal EnableDelayedExpansion

:: Make sure we're talking to the robot over adb, then build & install TeamCode.
::
:: Confirms an adb device is present; if the connection dropped (common over
:: Wi-Fi as the hub sleeps or roams), it retries `adb connect` a few times before
:: giving up. Once connected it runs the Gradle install of the TeamCode app.
::
:: Connection: handled by ensure-robot-connected.bat — uses a USB device if present,
:: else connects to the Control Hub AP (192.168.43.1:5555 by default; override with
:: ADB_HOST). Set ADB_SERIAL to target a specific device when several are attached.
::
:: Usage:
::   deploy.bat                 # ensure connection, then installDebug
::   deploy.bat --no-install    # just (re)establish the adb connection
::   set ADB_HOST=192.168.43.1 & deploy.bat

set "SCRIPT_DIR=%~dp0"
if "!SCRIPT_DIR:~-1!"=="\" set "SCRIPT_DIR=!SCRIPT_DIR:~0,-1!"
set "GRADLE_TASK=:TeamCode:installDebug"

set "INSTALL=1"
:parse_args
if "%~1"=="" goto args_done
if /i "%~1"=="--no-install" (
    set "INSTALL=0"
    shift
    goto parse_args
)
echo error: unknown argument '%~1'
exit /b 2

:args_done

call "!SCRIPT_DIR!\ensure-robot-connected.bat"
if errorlevel 1 exit /b 1

if "!INSTALL!"=="0" (
    echo Connection ready; skipping install ^(--no-install^).
    call "!SCRIPT_DIR!\ensure-robot-connected.bat" disconnect
    exit /b 0
)

echo Installing !GRADLE_TASK! ...
call "!SCRIPT_DIR!\gradlew.bat" -p "!SCRIPT_DIR!" "!GRADLE_TASK!"
set "RC=!errorlevel!"

call "!SCRIPT_DIR!\ensure-robot-connected.bat" disconnect

if not !RC!==0 (
    echo Install failed.
    exit /b !RC!
)
echo Done.
exit /b 0
