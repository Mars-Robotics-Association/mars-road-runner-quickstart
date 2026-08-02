@echo off
setlocal EnableDelayedExpansion

:: Usage: commit-submodule.bat <tag> [commit-message]
:: Commits changes in MarsCommonFtc, tags, pushes, then updates the parent repo.

set "TAG=%~1"
set "MSG=%~2"

if "%TAG%"=="" (
    echo Usage: %~nx0 ^<tag^> [commit-message]
    exit /b 1
)

set "SUBMODULE=MarsCommonFtc"
set "PARENT_DIR=%~dp0"
if "!PARENT_DIR:~-1!"=="\" set "PARENT_DIR=!PARENT_DIR:~0,-1!"
set "SUB_DIR=!PARENT_DIR!\!SUBMODULE!"

:: --- Submodule ---
cd /d "!SUB_DIR!"
if errorlevel 1 (
    echo ERROR: Submodule directory not found: !SUB_DIR!
    exit /b 1
)

:: Submodules check out in detached HEAD; get the configured tracking branch and switch to it.
set "BRANCH="
for /f "tokens=*" %%B in ('git config -f "!PARENT_DIR!\.gitmodules" "submodule.!SUBMODULE!.branch" 2^>nul') do set "BRANCH=%%B"
if "!BRANCH!"=="" (
    for /f "tokens=*" %%B in ('git rev-parse --abbrev-ref HEAD 2^>nul') do set "BRANCH=%%B"
)
if "!BRANCH!"=="" (
    echo Cannot determine branch for !SUBMODULE! -- set 'branch' in .gitmodules or check out a branch manually.
    exit /b 1
)
if /i "!BRANCH!"=="HEAD" (
    echo Cannot determine branch for !SUBMODULE! -- set 'branch' in .gitmodules or check out a branch manually.
    exit /b 1
)

git checkout "!BRANCH!"
if errorlevel 1 (
    echo ERROR: Failed to checkout branch !BRANCH! in !SUBMODULE!.
    exit /b 1
)

:: Check for uncommitted changes
set "HAS_CHANGES=0"
for /f "tokens=*" %%S in ('git status --porcelain 2^>nul') do set "HAS_CHANGES=1"
if "!HAS_CHANGES!"=="0" (
    echo No changes in !SUBMODULE! to commit.
) else (
    if "!MSG!"=="" (
        set /p MSG="Commit message for !SUBMODULE!: "
    )
    if "!MSG!"=="" (
        echo ERROR: Commit message is required when there are changes.
        exit /b 1
    )
    git add -A
    git commit -m "!MSG!"
    if errorlevel 1 (
        echo ERROR: git commit failed in !SUBMODULE!.
        exit /b 1
    )
)

:: Tag if it does not already exist
git rev-parse "!TAG!" >nul 2>&1
if errorlevel 1 (
    git tag "!TAG!"
    if errorlevel 1 (
        echo ERROR: Failed to create tag !TAG!.
        exit /b 1
    )
)

git push origin "!BRANCH!" --tags
if errorlevel 1 (
    echo ERROR: Failed to push !SUBMODULE! branch '!BRANCH!' and tag '!TAG!'.
    exit /b 1
)
echo Pushed !SUBMODULE! branch '!BRANCH!' and tag '!TAG!'.

:: --- Parent repo ---
cd /d "!PARENT_DIR!"
git submodule update --remote -- "!SUBMODULE!"
if errorlevel 1 (
    echo ERROR: git submodule update --remote failed.
    exit /b 1
)
git add "!SUBMODULE!"

set "STAGED="
for /f "tokens=*" %%F in ('git diff --cached --name-only 2^>nul') do set "STAGED=1"
if "!STAGED!"=="" (
    echo Parent repo submodule pointer already up to date.
) else (
    git commit -m "Update !SUBMODULE! to !TAG!"
    if errorlevel 1 (
        echo ERROR: Parent repo commit failed.
        exit /b 1
    )
    echo Parent repo updated to !TAG!.
)

exit /b 0
