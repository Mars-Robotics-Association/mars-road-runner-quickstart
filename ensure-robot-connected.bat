@echo off
setlocal EnableDelayedExpansion

:: Shared adb connection helper for the robot tooling (deploy.bat).
::
:: Can be run directly to (re)establish the connection:
::   ensure-robot-connected.bat
::
:: Or called from another batch file:
::   call ensure-robot-connected.bat
::   if errorlevel 1 exit /b 1
::   ... work with adb ...
::   call ensure-robot-connected.bat disconnect
::
:: Behavior: if an adb device is already present (e.g. USB), it's used as-is.
:: Otherwise it tries `adb connect` to the hub, retrying a few times. The target
:: defaults to the Control Hub AP at 192.168.43.1:5555; override with ADB_HOST.
:: Set ADB_SERIAL to pin a specific device when several are attached.
::
:: Subcommands:
::   (none)       ensure connection
::   disconnect   tear down a network connection this script opened

if /i "%~1"=="disconnect" goto :disconnect

:: Default target: the Control Hub's own Wi-Fi AP. Override via ADB_HOST.
if defined ADB_HOST (
    set "ROBOT_HOST=!ADB_HOST!"
) else (
    set "ROBOT_HOST=192.168.43.1"
)
echo !ROBOT_HOST! | findstr ":" >nul 2>&1
if errorlevel 1 set "ROBOT_HOST=!ROBOT_HOST!:5555"

if not defined ROBOT_CONNECT_RETRIES set "ROBOT_CONNECT_RETRIES=5"
if not defined ROBOT_CONNECT_WAIT set "ROBOT_CONNECT_WAIT=3"

where adb >nul 2>&1
if errorlevel 1 (
    echo error: adb not found ^(add Android platform-tools to PATH^)
    exit /b 1
)

call :robot_is_connected
if not errorlevel 1 (
    if defined ADB_SERIAL (
        echo adb: connected ^(!ADB_SERIAL!^)
    ) else (
        echo adb: connected ^(usb^)
    )
    set "_ROBOT_NETWORK_CONNECTED=0"
    goto :export_and_exit_ok
)

set /a _ATTEMPT=1
:connect_loop
if !_ATTEMPT! gtr !ROBOT_CONNECT_RETRIES! goto :connect_failed

echo adb: not connected — connect attempt !_ATTEMPT!/!ROBOT_CONNECT_RETRIES! to !ROBOT_HOST! ...
:: Drop any stale offline entry before retrying.
adb disconnect "!ROBOT_HOST!" >nul 2>&1
adb connect "!ROBOT_HOST!" >nul 2>&1
:: Pin the serial to the host we just connected, so it isn't ambiguous when a
:: USB device is also attached.
if not defined ADB_SERIAL set "ADB_SERIAL=!ROBOT_HOST!"
call :robot_is_connected
if not errorlevel 1 (
    echo adb: connected to !ROBOT_HOST!
    :: Remember that we (not USB) opened this network link, so disconnect
    :: only tears down a connection we actually established.
    set "_ROBOT_NETWORK_CONNECTED=1"
    goto :export_and_exit_ok
)
timeout /t !ROBOT_CONNECT_WAIT! /nobreak >nul
set /a _ATTEMPT+=1
goto :connect_loop

:connect_failed
echo error: could not reach the hub at !ROBOT_HOST! after !ROBOT_CONNECT_RETRIES! attempts.
echo        Check the robot is on, you're on its Wi-Fi, and adb is enabled.
exit /b 1

:: Export key vars to the caller (after endlocal) and return success.
:: Capture delayed-expansion values into for-loop metavars so they survive endlocal.
:export_and_exit_ok
if defined ADB_SERIAL (
    for %%A in ("!ADB_SERIAL!") do for %%H in ("!ROBOT_HOST!") do for %%N in ("!_ROBOT_NETWORK_CONNECTED!") do (
        endlocal
        set "ADB_SERIAL=%%~A"
        set "ROBOT_HOST=%%~H"
        set "_ROBOT_NETWORK_CONNECTED=%%~N"
    )
) else (
    for %%H in ("!ROBOT_HOST!") do for %%N in ("!_ROBOT_NETWORK_CONNECTED!") do (
        endlocal
        set "ROBOT_HOST=%%~H"
        set "_ROBOT_NETWORK_CONNECTED=%%~N"
    )
)
exit /b 0

:disconnect
:: Tear down a network connection that ensure_robot_connected opened. No-op for a
:: USB session (we never connected over the network).
if not "%_ROBOT_NETWORK_CONNECTED%"=="1" (
    endlocal
    exit /b 0
)
if not defined ROBOT_HOST (
    if defined ADB_HOST (
        set "ROBOT_HOST=!ADB_HOST!"
    ) else (
        set "ROBOT_HOST=192.168.43.1:5555"
    )
)
echo !ROBOT_HOST! | findstr ":" >nul 2>&1
if errorlevel 1 set "ROBOT_HOST=!ROBOT_HOST!:5555"
echo adb: disconnecting !ROBOT_HOST!
adb disconnect "!ROBOT_HOST!" >nul 2>&1
endlocal
set "_ROBOT_NETWORK_CONNECTED=0"
exit /b 0

:robot_is_connected
:: True (exit 0) when adb reports a usable "device" state for our target.
set "_STATE="
if defined ADB_SERIAL (
    for /f "tokens=*" %%S in ('adb -s "!ADB_SERIAL!" get-state 2^>nul') do set "_STATE=%%S"
) else (
    for /f "tokens=*" %%S in ('adb get-state 2^>nul') do set "_STATE=%%S"
)
if /i "!_STATE!"=="device" exit /b 0
exit /b 1
