@echo off
setlocal EnableDelayedExpansion

:: Warm the Gradle/Android caches online so an offline install will work later.
::
:: Run this while you have a network connection. It does three things:
::   1. Ensure submodule sources match the pins in the parent *index* (committed
::      or staged-but-uncommitted). Unstaged pin drift fails fast — never reset
::      or stage. Nested/missing checkouts are filled from those pins. Offline
::      deploy has no network to fetch pins, so sources must be correct now.
::   2. Online build of the TeamCode debug APK — pulls the Gradle distribution,
::      every dependency artifact, and any missing Android SDK build-tools into
::      the local caches.
::   3. Offline verification — `clean` then rebuild with `--offline`, forcing a
::      full recompile/repackage using only what's now cached. If this succeeds,
::      you have real assurance that a later offline `deploy.bat` (which runs
::      installDebug) won't need the network for dependencies.
::
:: Usage:
::   prefetch.bat              # ensure pins, warm caches, verify offline
::   prefetch.bat --no-verify  # ensure pins + warm only; skip offline rebuild

set "SCRIPT_DIR=%~dp0"
if "!SCRIPT_DIR:~-1!"=="\" set "SCRIPT_DIR=!SCRIPT_DIR:~0,-1!"
set "TASK=:TeamCode:assembleDebug"

set "VERIFY=1"
:parse_args
if "%~1"=="" goto args_done
if /i "%~1"=="--no-verify" (
    set "VERIFY=0"
    shift
    goto parse_args
)
echo error: unknown argument '%~1'
exit /b 2

:args_done

echo ==^> Ensuring submodules match parent index pins ...
:: Fail-fast on top-level pin drift *before* submodule update. Update would
:: silently reset an unstaged intentional checkout back to the old index pin.
:: Intent is captured by update-submodules (stages the gitlink) or git add.
set "SUB_DRIFT=0"
for /f "tokens=1,2" %%S in ('git -C "!SCRIPT_DIR!" submodule status 2^>nul') do (
    set "SUB_HASH=%%S"
    set "SUB_PATH=%%T"
    set "SUB_FLAG=!SUB_HASH:~0,1!"
    if "!SUB_FLAG!"=="+" (
        echo FAIL: !SUB_PATH! is checked out at a different commit than the parent index pin.
        echo       If this pin is intentional:  git add !SUB_PATH!
        echo       ^(or re-run update-submodules.bat !SUB_PATH!^)
        echo       If not:                      git submodule update --init --recursive
        set "SUB_DRIFT=1"
    ) else if "!SUB_FLAG!"=="U" (
        echo FAIL: !SUB_PATH! has unresolved merge conflicts.
        set "SUB_DRIFT=1"
    )
)
if "!SUB_DRIFT!"=="1" exit /b 1

:: Index pins only (staged uncommitted gitlinks count). Fills missing clones and
:: nested pins; does not rewrite a top-level HEAD that already matches the index.
git -C "!SCRIPT_DIR!" submodule update --init --recursive
if errorlevel 1 (
    echo FAIL: could not check out submodules at the pins in the parent index.
    echo       Fix network/auth, or clean dirty submodule trees, then retry.
    exit /b 1
)

set "SUB_BAD=0"
for /f "tokens=1,2" %%S in ('git -C "!SCRIPT_DIR!" submodule status --recursive 2^>nul') do (
    set "SUB_HASH=%%S"
    set "SUB_PATH=%%T"
    set "SUB_FLAG=!SUB_HASH:~0,1!"
    if "!SUB_FLAG!"=="-" (
        echo FAIL: !SUB_PATH! is not initialized.
        set "SUB_BAD=1"
    ) else if "!SUB_FLAG!"=="+" (
        echo FAIL: !SUB_PATH! is not at the commit recorded in its parent pin.
        set "SUB_BAD=1"
    ) else if "!SUB_FLAG!"=="U" (
        echo FAIL: !SUB_PATH! has unresolved merge conflicts.
        set "SUB_BAD=1"
    )
)
if "!SUB_BAD!"=="1" (
    echo        Run: git submodule update --init --recursive
    exit /b 1
)

:: Dirty files on the correct commit still change what gets compiled into the
:: offline caches — refuse so the prefetched APK matches the pins exactly.
git -C "!SCRIPT_DIR!" submodule foreach --recursive "git diff --quiet && git diff --cached --quiet" >nul 2>&1
if errorlevel 1 (
    echo FAIL: one or more submodules have uncommitted local changes.
    echo       Commit, stash, or discard them so the prefetched build matches the pin.
    git -C "!SCRIPT_DIR!" submodule foreach --recursive "git status -s"
    exit /b 1
)

echo ==^> Warming caches with an online build of !TASK! ...
:: --refresh-dependencies makes Gradle re-resolve every dependency, so anything
:: missing from the cache is fetched now rather than silently relied on later.
call "!SCRIPT_DIR!\gradlew.bat" -p "!SCRIPT_DIR!" --refresh-dependencies "!TASK!"
if errorlevel 1 (
    echo FAIL: online build failed.
    exit /b 1
)

if "!VERIFY!"=="0" (
    echo Caches warmed; skipping offline verification ^(--no-verify^).
    exit /b 0
)

echo ==^> Verifying: clean + offline rebuild of !TASK! ^(no network^) ...
:: clean forces real compilation/packaging rather than up-to-date no-ops, so the
:: offline run genuinely exercises the cached dependencies.
call "!SCRIPT_DIR!\gradlew.bat" -p "!SCRIPT_DIR!" --offline clean "!TASK!"
if errorlevel 1 (
    echo.
    echo FAIL: offline build failed — something still needs the network.
    echo       Re-run prefetch.bat while online to fetch the missing pieces.
    exit /b 1
)

echo.
echo OK: offline build succeeded. Dependencies are cached — you can install
echo     offline with deploy.bat ^(or: gradlew.bat --offline :TeamCode:installDebug^).
exit /b 0
