@echo off
setlocal EnableDelayedExpansion

set "SCRIPT_DIR=%~dp0"

:: Collect submodule paths from .gitmodules
set SUB_COUNT=0
for /f "tokens=*" %%P in ('git config --file "%SCRIPT_DIR%.gitmodules" --get-regexp "submodule\..*.path"') do (
    for /f "tokens=2" %%V in ("%%P") do (
        set /a SUB_COUNT+=1
        set SUBMODULE[!SUB_COUNT!]=%%V
    )
)

if %SUB_COUNT%==0 (
    echo No submodules found in .gitmodules.
    pause
    exit /b 1
)

set SUB_PATH=
if not "%~1"=="" goto match_arg
goto list_submodules

:match_arg
set "REQUESTED=%~1"
for /l %%I in (1,1,%SUB_COUNT%) do (
    set "CANDIDATE=!SUBMODULE[%%I]!"
    for %%F in ("!CANDIDATE!") do set "CANDIDATE_BASE=%%~nxF"
    if /i "!CANDIDATE!"=="!REQUESTED!" set "SUB_PATH=!CANDIDATE!"
    if /i "!CANDIDATE_BASE!"=="!REQUESTED!" set "SUB_PATH=!CANDIDATE!"
)
if not "!SUB_PATH!"=="" goto submodule_selected
echo ERROR: No submodule matching "%~1" found.
echo Available submodules:
for /l %%I in (1,1,%SUB_COUNT%) do (
    echo   - !SUBMODULE[%%I]!
)
pause
exit /b 1

:list_submodules
echo Available submodules:
echo -----------------------------------------------
for /l %%I in (1,1,%SUB_COUNT%) do (
    echo   %%I^) !SUBMODULE[%%I]!
)
echo.

:prompt_submodule
set CHOICE=
set /p CHOICE="Select a submodule to update (1-%SUB_COUNT%): "
set /a CHOICE_NUM=%CHOICE% 2>nul
if "%CHOICE_NUM%"=="" goto invalid_submodule
if %CHOICE_NUM% lss 1 goto invalid_submodule
if %CHOICE_NUM% gtr %SUB_COUNT% goto invalid_submodule
set "SUB_PATH=!SUBMODULE[%CHOICE_NUM%]!"
goto submodule_selected

:invalid_submodule
echo Invalid selection. Please enter a number between 1 and %SUB_COUNT%.
goto prompt_submodule

:submodule_selected

echo.
echo Fetching tags from %SUB_PATH%...
cd /d "%SCRIPT_DIR%%SUB_PATH%"
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
    echo No tags found in %SUB_PATH%.
    pause
    exit /b 1
)

echo.
echo Available %SUB_PATH% tags ^(newest first^):
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

echo Updating nested submodules...
git submodule update --init --recursive
if errorlevel 1 (
    echo ERROR: Submodule update failed.
    pause
    exit /b 1
)
cd /d "%SCRIPT_DIR%"

echo Staging pointer update in parent repo...
git add "%SUB_PATH%"

echo.
echo %SUB_PATH% updated to %SELECTED_TAG%.
echo The submodule pointer is staged -- review and commit when ready.
pause
exit /b 0

:invalid
echo Invalid selection. Please enter a number between 1 and %COUNT%.
goto prompt
