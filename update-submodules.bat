@echo off
setlocal EnableDelayedExpansion

set "SCRIPT_DIR=%~dp0"
if "!SCRIPT_DIR:~-1!"=="\" set "SCRIPT_DIR=!SCRIPT_DIR:~0,-1!"

:: Collect submodule paths from .gitmodules
set SUB_COUNT=0
for /f "tokens=*" %%P in ('git config --file "!SCRIPT_DIR!\.gitmodules" --get-regexp "submodule\..*.path"') do (
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
cd /d "!SCRIPT_DIR!\!SUB_PATH!"
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
set "TARGET_SHA="
:: rev-list -n 1 peels annotated tags to the commit without bat caret-escaping pain.
for /f "tokens=*" %%H in ('git rev-list -n 1 "tags/!SELECTED_TAG!" 2^>nul') do set "TARGET_SHA=%%H"
if "!TARGET_SHA!"=="" (
    echo ERROR: Could not resolve tag !SELECTED_TAG! to a commit.
    pause
    exit /b 1
)

echo.
echo Checking out !SELECTED_TAG! ^(!TARGET_SHA!^) ...
:: --force resets the worktree to the tag so dirty files cannot leave sources
:: off the selected pin. Nested update --force does the same one level down.
git checkout --force "!TARGET_SHA!"
if errorlevel 1 (
    echo ERROR: Failed to checkout !SELECTED_TAG! ^(!TARGET_SHA!^).
    pause
    exit /b 1
)

echo Updating nested submodules to this pin's recorded hashes...
git submodule update --init --recursive --force
if errorlevel 1 (
    echo ERROR: Nested submodule update failed.
    pause
    exit /b 1
)

cd /d "!SCRIPT_DIR!"

echo Staging pointer update in parent repo...
git add "!SUB_PATH!"
if errorlevel 1 (
    echo ERROR: Failed to stage !SUB_PATH! in the parent repo.
    pause
    exit /b 1
)

:: Parent-side update uses the *index* pin we just staged — ensures this path
:: (and its nested submodules) really match that hash, not only the in-tree
:: checkout that preceded the stage.
echo Syncing parent view of !SUB_PATH! to the staged pin...
git submodule update --init --recursive --force -- "!SUB_PATH!"
if errorlevel 1 (
    echo ERROR: Parent submodule update for !SUB_PATH! failed.
    pause
    exit /b 1
)

:: Read pins from this directory (already cd'd to parent). Avoid git -C with a
:: trailing-backslash path, and avoid parsing ls-files columns — rev-parse
:: ":path" is the index object for that gitlink.
set "HEAD_SHA="
for /f "tokens=*" %%H in ('git -C "!SCRIPT_DIR!\!SUB_PATH!" rev-parse HEAD 2^>nul') do set "HEAD_SHA=%%H"
set "INDEX_SHA="
for /f "tokens=*" %%H in ('git rev-parse ":!SUB_PATH!" 2^>nul') do set "INDEX_SHA=%%H"
if /i not "!HEAD_SHA!"=="!TARGET_SHA!" (
    echo ERROR: !SUB_PATH! HEAD is !HEAD_SHA!, expected !TARGET_SHA!.
    pause
    exit /b 1
)
if /i not "!INDEX_SHA!"=="!TARGET_SHA!" (
    echo ERROR: parent index pin for !SUB_PATH! is !INDEX_SHA!, expected !TARGET_SHA!.
    pause
    exit /b 1
)

echo.
echo %SUB_PATH% is at !SELECTED_TAG! ^(!TARGET_SHA!^).
echo Worktree and nested submodules match that hash; parent pointer is staged.
echo Review and commit when ready.
pause
exit /b 0

:invalid
echo Invalid selection. Please enter a number between 1 and %COUNT%.
goto prompt
