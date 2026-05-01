@echo off
setlocal EnableDelayedExpansion

echo Fetching tags from MarsCommonFtc...
cd /d "%~dp0MarsCommonFtc"
git fetch --prune --force --quiet origin "+refs/tags/*:refs/tags/*"
if errorlevel 1 (
    echo ERROR: Failed to fetch tags from remote "origin".
    echo        Check network access, git auth, and safe.directory settings.
    pause
    exit /b 1
)

:: Detect which tag is currently checked out (empty if HEAD is not on a tag)
set CURRENT_TAG=
for /f "tokens=*" %%C in ('git describe --tags --exact-match HEAD 2^>nul') do set CURRENT_TAG=%%C

:: Collect the 10 most recent tags with their commit dates
set COUNT=0
for /f "tokens=*" %%T in ('git tag --sort=-creatordate') do (
    if !COUNT! lss 10 (
        set /a COUNT+=1
        set TAG[!COUNT!]=%%T
        for /f "tokens=*" %%D in ('git log -1 --format^=%%ci "%%T" 2^>nul') do (
            set DATE[!COUNT!]=%%D
        )
    )
)

if %COUNT%==0 (
    echo No tags found in MarsCommonFtc.
    pause
    exit /b 1
)

echo.
echo Available MarsCommonFtc tags ^(newest first^):
echo -----------------------------------------------
for /l %%I in (1,1,%COUNT%) do (
    if "!TAG[%%I]!"=="!CURRENT_TAG!" (
        echo   %%I^) !TAG[%%I]!   !DATE[%%I]!  ^<-- current
    ) else (
        echo   %%I^) !TAG[%%I]!   !DATE[%%I]!
    )
)
echo.

:prompt
set CHOICE=
set /p CHOICE="Enter number (1-%COUNT%): "

:: Validate input is a number in range
set /a CHOICE_NUM=%CHOICE% 2>nul
if "%CHOICE_NUM%"=="" goto invalid
if %CHOICE_NUM% lss 1 goto invalid
if %CHOICE_NUM% gtr %COUNT% goto invalid

set SELECTED_TAG=!TAG[%CHOICE_NUM%]!
echo.
echo Checking out tag: %SELECTED_TAG%

git checkout "tags/%SELECTED_TAG%" --quiet
if errorlevel 1 (
    echo ERROR: Failed to checkout tag %SELECTED_TAG%.
    pause
    exit /b 1
)

cd /d "%~dp0"
echo Updating submodules recursively...
git submodule update --init --recursive
if errorlevel 1 (
    echo ERROR: Submodule update failed.
    pause
    exit /b 1
)

echo.
echo MarsCommonFtc updated to %SELECTED_TAG%.
pause
exit /b 0

:invalid
echo Invalid selection. Please enter a number between 1 and %COUNT%.
goto prompt
