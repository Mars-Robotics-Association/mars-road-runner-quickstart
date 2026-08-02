@echo off
setlocal EnableDelayedExpansion

:: Warm the Gradle/Android caches online so an offline install will work later.
::
:: Run this while you have a network connection. It does two things:
::   1. Online build of the TeamCode debug APK — pulls the Gradle distribution,
::      every dependency artifact, and any missing Android SDK build-tools into
::      the local caches.
::   2. Offline verification — `clean` then rebuild with `--offline`, forcing a
::      full recompile/repackage using only what's now cached. If this succeeds,
::      you have real assurance that a later offline `deploy.bat` (which runs
::      installDebug) won't need the network for dependencies.
::
:: Usage:
::   prefetch.bat              # warm caches, then verify the offline build
::   prefetch.bat --no-verify  # warm caches only, skip the offline rebuild

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
