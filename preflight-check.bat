@echo off
setlocal EnableDelayedExpansion

set ERRORS=0
set WARNINGS=0
set SCRIPT_DIR=%~dp0
if "!SCRIPT_DIR:~-1!"=="\" set SCRIPT_DIR=!SCRIPT_DIR:~0,-1!

echo =============================================
echo  FTC TeamCode Pre-Flight Check
echo =============================================
echo.

:: ============================================================
:: READ REQUIREMENTS VIA GRADLE
:: ============================================================
echo Reading build requirements via Gradle...
set REQ_JAVA=
set REQ_SDK=
set REQ_NDK_MAIN=
set REQ_NDK_RUCKIG=
set REQ_CMAKE=
set TEMP_VERSIONS=%TEMP%\preflight_%RANDOM%.txt

cd /d "!SCRIPT_DIR!"
call "!SCRIPT_DIR!\gradlew.bat" -q --warning-mode=none --init-script "!SCRIPT_DIR!\preflight-versions.gradle" printBuildVersions > "!TEMP_VERSIONS!" 2>nul

if not !errorlevel!==0 (
    del "!TEMP_VERSIONS!" 2>nul
    echo   [FAIL] Gradle query failed. Could not read build requirements.
    set /a ERRORS+=1
    goto summary
)

for /f "usebackq tokens=1,* delims==" %%A in ("!TEMP_VERSIONS!") do (
    if "%%A"=="PREFLIGHT_JAVA" set REQ_JAVA=%%B
    if "%%A"=="PREFLIGHT_COMPILE_SDK" set REQ_SDK=%%B
    if "%%A"=="PREFLIGHT_NDK_MAIN" set REQ_NDK_MAIN=%%B
    if "%%A"=="PREFLIGHT_NDK_RUCKIG" set REQ_NDK_RUCKIG=%%B
    if "%%A"=="PREFLIGHT_CMAKE_MIN" (
        set REQ_CMAKE=%%B
        set REQ_CMAKE=!REQ_CMAKE:+=!
    )
)
del "!TEMP_VERSIONS!" 2>nul

if "!REQ_JAVA!"=="" (
    echo   [FAIL] Missing PREFLIGHT_JAVA from Gradle output.
    set /a ERRORS+=1
)
if "!REQ_SDK!"=="" (
    echo   [FAIL] Missing PREFLIGHT_COMPILE_SDK from Gradle output.
    set /a ERRORS+=1
)
if "!REQ_NDK_MAIN!"=="" (
    echo   [FAIL] Missing PREFLIGHT_NDK_MAIN from Gradle output.
    set /a ERRORS+=1
)
if "!REQ_NDK_RUCKIG!"=="" (
    echo   [FAIL] Missing PREFLIGHT_NDK_RUCKIG from Gradle output.
    set /a ERRORS+=1
)
if "!REQ_CMAKE!"=="" (
    echo   [FAIL] Missing PREFLIGHT_CMAKE_MIN from Gradle output.
    set /a ERRORS+=1
)

if /i "!REQ_JAVA!"=="unknown" (
    echo   [FAIL] PREFLIGHT_JAVA returned 'unknown'.
    set /a ERRORS+=1
)
if /i "!REQ_SDK!"=="unknown" (
    echo   [FAIL] PREFLIGHT_COMPILE_SDK returned 'unknown'.
    set /a ERRORS+=1
)
if /i "!REQ_NDK_MAIN!"=="unknown" (
    echo   [FAIL] PREFLIGHT_NDK_MAIN returned 'unknown'.
    set /a ERRORS+=1
)
if /i "!REQ_NDK_RUCKIG!"=="unknown" (
    echo   [FAIL] PREFLIGHT_NDK_RUCKIG returned 'unknown'.
    set /a ERRORS+=1
)
if /i "!REQ_CMAKE!"=="unknown" (
    echo   [FAIL] PREFLIGHT_CMAKE_MIN returned 'unknown'.
    set /a ERRORS+=1
)

if !ERRORS! GTR 0 (
    goto summary
)

echo   Java      : !REQ_JAVA!
echo   API level : android-!REQ_SDK!
echo   NDK main  : !REQ_NDK_MAIN!
echo   NDK C++20 : !REQ_NDK_RUCKIG!
echo   CMake min : !REQ_CMAKE!
echo.

:: Pre-compute cmake major/minor for the version comparison in check 7
set REQ_CMAKE_MAJOR=3
set REQ_CMAKE_MINOR=22
for /f "tokens=1,2 delims=." %%A in ("!REQ_CMAKE!") do (
    set REQ_CMAKE_MAJOR=%%A
    set REQ_CMAKE_MINOR=%%B
)

:: ============================================================
:: CHECK 1/10: JDK version
:: ============================================================
echo [1/10] JDK version...
set JAVA_VER_LINE=
set JAVA_FOUND_RAW=
set JAVA_FOUND_NORM=
for /f "tokens=*" %%L in ('java -version 2^>^&1') do (
    if "!JAVA_VER_LINE!"=="" set JAVA_VER_LINE=%%L
)
if "!JAVA_VER_LINE!"=="" (
    echo       [FAIL] java not found on PATH
    echo              Install JDK !REQ_JAVA! and add it to PATH ^(or set JAVA_HOME^)
    set /a ERRORS+=1
) else (
    set JAVA_MAJOR=
    for /f "tokens=3 delims= " %%V in ("!JAVA_VER_LINE!") do (
        set _TEMP=%%~V
        set _TEMP=!_TEMP:"=!
        set JAVA_FOUND_RAW=!_TEMP!
        for /f "tokens=1 delims=." %%M in ("!_TEMP!") do set JAVA_MAJOR=%%M
    )
    set _JAVA_PARSE_INPUT=!JAVA_FOUND_RAW!
    call :normalize_jdk_version _JAVA_PARSE_INPUT JAVA_FOUND_NORM
    if "!JAVA_MAJOR!"=="!REQ_JAVA!" (
        echo       [OK]   !JAVA_VER_LINE!
    ) else (
        echo       [FAIL] JDK !REQ_JAVA! required. Found: !JAVA_VER_LINE!
        set /a ERRORS+=1
    )
)

:: ============================================================
:: CHECK 2/10: JDK security baseline via OpenJDK advisory
:: ============================================================
echo [2/10] JDK security baseline ^(OpenJDK advisory^)...
if "!JAVA_FOUND_NORM!"=="" (
    echo       [SKIP] JDK version unknown
) else (
    set ADVISORY_INDEX_URL=https://openjdk.org/groups/vulnerability/advisories/
    set ADVISORY_INDEX_FILE=%TEMP%\preflight_advisory_index_%RANDOM%.txt
    set ADVISORY_BODY_FILE=%TEMP%\preflight_advisory_body_%RANDOM%.txt
    set LATEST_ADVISORY_PATH=
    set AFFECTED_LINE=
    set REQUIRED_FOR_MAJOR=
    set REQUIRED_NORM=

    set FETCH_OK=0
    set FETCH_ERR=
    where curl.exe >nul 2>&1
    if !errorlevel!==0 (
        rem curl unavailable; fall back to PowerShell
    ) else (
        curl.exe -fsSL "!ADVISORY_INDEX_URL!" -o "!ADVISORY_INDEX_FILE!" >nul 2>&1
        if !errorlevel!==0 (
            for /f "usebackq delims=" %%E in (`curl.exe -fsSL "!ADVISORY_INDEX_URL!" -o "!ADVISORY_INDEX_FILE!" 2^>^&1`) do (
                if "!FETCH_ERR!"=="" set FETCH_ERR=%%E
            )
        ) else (
            set FETCH_OK=1
        )
    )
    if !FETCH_OK!==0 (
        for /f "usebackq delims=" %%E in (`powershell -NoProfile -ExecutionPolicy Bypass -Command "$ErrorActionPreference='Stop'; try { [Net.ServicePointManager]::SecurityProtocol=[Net.SecurityProtocolType]::Tls12; (Invoke-WebRequest -UseBasicParsing '!ADVISORY_INDEX_URL!').Content | Out-File -FilePath '!ADVISORY_INDEX_FILE!' -Encoding utf8; 'OK' } catch { $_.Exception.Message; exit 1 }"`) do (
            if "%%E"=="OK" (
                set FETCH_OK=1
            ) else if "!FETCH_ERR!"=="" (
                set FETCH_ERR=%%E
            )
        )
    )
    if !FETCH_OK!==0 (
        echo       [WARN] Could not reach !ADVISORY_INDEX_URL!
        if not "!FETCH_ERR!"=="" echo              Reason: !FETCH_ERR!
        set /a WARNINGS+=1
    ) else (
        for /f "usebackq delims=" %%P in (`powershell -NoProfile -ExecutionPolicy Bypass -Command "$c=Get-Content -Raw '!ADVISORY_INDEX_FILE!'; $m=[regex]::Match($c,'\d{4}-\d{2}-\d{2}'); if($m.Success){$m.Value}"`) do (
            if "!LATEST_ADVISORY_PATH!"=="" set LATEST_ADVISORY_PATH=%%P
        )
        if "!LATEST_ADVISORY_PATH!"=="" (
            echo       [WARN] Could not parse latest advisory URL from OpenJDK
            set /a WARNINGS+=1
        ) else (
            set LATEST_ADVISORY_URL=https://openjdk.org/groups/vulnerability/advisories/!LATEST_ADVISORY_PATH!
            set FETCH_OK=0
            set FETCH_ERR=
            where curl.exe >nul 2>&1
            if !errorlevel!==0 (
                rem curl unavailable; fall back to PowerShell
            ) else (
                curl.exe -fsSL "!LATEST_ADVISORY_URL!" -o "!ADVISORY_BODY_FILE!" >nul 2>&1
                if !errorlevel!==0 (
                    for /f "usebackq delims=" %%E in (`curl.exe -fsSL "!LATEST_ADVISORY_URL!" -o "!ADVISORY_BODY_FILE!" 2^>^&1`) do (
                        if "!FETCH_ERR!"=="" set FETCH_ERR=%%E
                    )
                ) else (
                    set FETCH_OK=1
                )
            )
            if !FETCH_OK!==0 (
                for /f "usebackq delims=" %%E in (`powershell -NoProfile -ExecutionPolicy Bypass -Command "$ErrorActionPreference='Stop'; try { [Net.ServicePointManager]::SecurityProtocol=[Net.SecurityProtocolType]::Tls12; (Invoke-WebRequest -UseBasicParsing '!LATEST_ADVISORY_URL!').Content | Out-File -FilePath '!ADVISORY_BODY_FILE!' -Encoding utf8; 'OK' } catch { $_.Exception.Message; exit 1 }"`) do (
                    if "%%E"=="OK" (
                        set FETCH_OK=1
                    ) else if "!FETCH_ERR!"=="" (
                        set FETCH_ERR=%%E
                    )
                )
            )
            if !FETCH_OK!==0 (
                echo       [WARN] Could not reach !LATEST_ADVISORY_URL!
                if not "!FETCH_ERR!"=="" echo              Reason: !FETCH_ERR!
                set /a WARNINGS+=1
            ) else (
                for /f "usebackq delims=" %%R in (`powershell -NoProfile -ExecutionPolicy Bypass -Command "$c=Get-Content -Raw '!ADVISORY_BODY_FILE!'; $re=[regex]::Match($c,'!REQ_JAVA!([.]\d+[.]\d+|u\d+)'); if($re.Success){$re.Value}"`) do (
                    if "!REQUIRED_FOR_MAJOR!"=="" set REQUIRED_FOR_MAJOR=%%R
                )
                if "!REQUIRED_FOR_MAJOR!"=="" (
                    echo       [WARN] Could not find JDK !REQ_JAVA! baseline in latest advisory
                    set /a WARNINGS+=1
                ) else (
                    set _REQ_PARSE_INPUT=!REQUIRED_FOR_MAJOR!
                    call :normalize_jdk_version _REQ_PARSE_INPUT REQUIRED_NORM
                    powershell -NoProfile -ExecutionPolicy Bypass -Command "$a=[version]'!JAVA_FOUND_NORM!'; $b=[version]'!REQUIRED_NORM!'; if($a -lt $b){ exit 10 } else { exit 0 }" >nul 2>&1
                    if !errorlevel!==10 (
                        echo       [FAIL] Installed JDK !JAVA_FOUND_RAW! is older than security baseline !REQUIRED_FOR_MAJOR!
                        echo              Advisory: !LATEST_ADVISORY_URL!
                        set /a ERRORS+=1
                    ) else (
                        echo       [OK]   JDK !JAVA_FOUND_RAW! meets or exceeds baseline !REQUIRED_FOR_MAJOR!
                    )
                )
            )
        )
    )

    del "!ADVISORY_INDEX_FILE!" 2>nul
    del "!ADVISORY_BODY_FILE!" 2>nul
)

:: ============================================================
:: CHECK 3/10: local.properties and sdk.dir
:: ============================================================
echo [3/10] local.properties...
set SDK_DIR=
if not exist "!SCRIPT_DIR!\local.properties" (
    echo       [FAIL] local.properties not found
    echo              Open the project once in Android Studio to generate it
    set /a ERRORS+=1
) else (
    for /f "usebackq tokens=1,* delims==" %%A in ("!SCRIPT_DIR!\local.properties") do (
        if "%%A"=="sdk.dir" set SDK_DIR=%%B
    )
    if "!SDK_DIR!"=="" (
        echo       [FAIL] sdk.dir not set in local.properties
        set /a ERRORS+=1
    ) else (
        :: Unescape Java properties format: C\:\\ -> C:\
        set SDK_DIR=!SDK_DIR:\:=:!
        set SDK_DIR=!SDK_DIR:\\=\!
        echo       [OK]   local.properties present
    )
)

:: ============================================================
:: CHECK 4/10: Android SDK directory
:: ============================================================
echo [4/10] Android SDK directory...
if "!SDK_DIR!"=="" (
    echo       [SKIP] SDK path unknown
) else if not exist "!SDK_DIR!" (
    echo       [FAIL] Android SDK not found at: !SDK_DIR!
    set /a ERRORS+=1
    set SDK_DIR=
) else (
    echo       [OK]   !SDK_DIR!
)

:: ============================================================
:: CHECK 5/10: Android SDK platform
:: ============================================================
echo [5/10] Android SDK platform android-!REQ_SDK!...
if "!SDK_DIR!"=="" (
    echo       [SKIP] SDK path unknown
) else if exist "!SDK_DIR!\platforms\android-!REQ_SDK!" (
    echo       [OK]   android-!REQ_SDK! installed
) else (
    echo       [FAIL] android-!REQ_SDK! not installed
    echo              SDK Manager -^> SDK Platforms -^> Android !REQ_SDK!
    set /a ERRORS+=1
)

:: ============================================================
:: CHECK 6/10: Android NDK main build
:: ============================================================
echo [6/10] Android NDK !REQ_NDK_MAIN! ^(main build^)...
if "!SDK_DIR!"=="" (
    echo       [SKIP] SDK path unknown
) else if exist "!SDK_DIR!\ndk\!REQ_NDK_MAIN!" (
    echo       [OK]   NDK !REQ_NDK_MAIN! found
) else (
    if /i "!REQ_NDK_MAIN!"=="!REQ_NDK_RUCKIG!" (
        echo       [WARN] NDK !REQ_NDK_MAIN! not found
        echo              SDK Manager -^> SDK Tools -^> NDK ^(Side by side^) -^> !REQ_NDK_MAIN!
        set /a WARNINGS+=1
    ) else (
        echo       [INFO] NDK !REQ_NDK_MAIN! not found ^(separate NDK is configured for native modules^)
        echo              Optional unless another module explicitly requires !REQ_NDK_MAIN!
    )
)

:: ============================================================
:: CHECK 7/10: Android NDK Ruckig / C++20
:: ============================================================
echo [7/10] Android NDK !REQ_NDK_RUCKIG! ^(RuckigNative / C++20^)...
if "!SDK_DIR!"=="" (
    echo       [SKIP] SDK path unknown
) else if exist "!SDK_DIR!\ndk\!REQ_NDK_RUCKIG!" (
    echo       [OK]   NDK !REQ_NDK_RUCKIG! found
) else (
    echo       [WARN] NDK !REQ_NDK_RUCKIG! not found -- RuckigNative will not compile
    echo              SDK Manager -^> SDK Tools -^> NDK ^(Side by side^) -^> !REQ_NDK_RUCKIG!
    set /a WARNINGS+=1
)

:: ============================================================
:: CHECK 8/10: CMake
:: ============================================================
echo [8/10] CMake ^>= !REQ_CMAKE! ^(RuckigNative native build^)...
set CMAKE_FOUND=0
set CMAKE_VER=

cmake --version >nul 2>&1
if !errorlevel!==0 (
    for /f "tokens=3" %%V in ('cmake --version 2^>^&1 ^| findstr /i "cmake version"') do (
        if "!CMAKE_VER!"=="" set CMAKE_VER=%%V
    )
    set CMAKE_FOUND=1
)

if !CMAKE_FOUND!==0 if not "!SDK_DIR!"=="" (
    for /d %%D in ("!SDK_DIR!\cmake\*") do (
        if exist "%%D\bin\cmake.exe" if !CMAKE_FOUND!==0 (
            for /f "tokens=3" %%V in ('"%%D\bin\cmake.exe" --version 2^>^&1 ^| findstr /i "cmake version"') do (
                if "!CMAKE_VER!"=="" set CMAKE_VER=%%V
            )
            set CMAKE_FOUND=1
        )
    )
)

if !CMAKE_FOUND!==0 (
    echo       [WARN] CMake not found on PATH or in Android SDK
    echo              SDK Manager -> SDK Tools -> CMake
    set /a WARNINGS+=1
) else (
    set FOUND_CMAKE_MAJOR=0
    set FOUND_CMAKE_MINOR=0
    for /f "tokens=1,2 delims=." %%A in ("!CMAKE_VER!") do (
        set FOUND_CMAKE_MAJOR=%%A
        set FOUND_CMAKE_MINOR=%%B
    )
    set CMAKE_OK=0
    if !FOUND_CMAKE_MAJOR! GTR !REQ_CMAKE_MAJOR! set CMAKE_OK=1
    if !FOUND_CMAKE_MAJOR! EQU !REQ_CMAKE_MAJOR! if !FOUND_CMAKE_MINOR! GEQ !REQ_CMAKE_MINOR! set CMAKE_OK=1
    if !CMAKE_OK!==1 (
        echo       [OK]   CMake !CMAKE_VER!
    ) else (
        echo       [WARN] CMake !CMAKE_VER! found but ^>= !REQ_CMAKE! required
        echo              Update CMake via SDK Manager -^> SDK Tools -^> CMake
        set /a WARNINGS+=1
    )
)

:: ============================================================
:: CHECK 9/10: Git submodules
:: ============================================================
echo [9/10] Git submodules...

git rev-parse --git-dir >nul 2>&1
if !errorlevel! neq 0 (
    echo       [FAIL] Not in a git repository or git not found
    set /a ERRORS+=1
    goto summary
)

set SUB_COUNT=0
for /f "tokens=1,2" %%S in ('git submodule status --recursive 2^>nul') do (
    set /a SUB_COUNT+=1
    set SUB_HASH=%%S
    set SUB_PATH=%%T
    set SUB_FLAG=!SUB_HASH:~0,1!

    if "!SUB_FLAG!"=="-" (
        echo       [FAIL] !SUB_PATH! -- not initialized
        echo              Run: git submodule update --init --recursive
        echo              Or run: update-MarsCommonFtc
        set /a ERRORS+=1
    ) else if "!SUB_FLAG!"=="+" (
        echo       [WARN] !SUB_PATH! -- at a different commit than the repo expects
        echo              Run: git submodule update --recursive
        set /a WARNINGS+=1
    ) else if "!SUB_FLAG!"=="U" (
        echo       [FAIL] !SUB_PATH! -- has unresolved merge conflicts
        set /a ERRORS+=1
    ) else (
        echo       [OK]   !SUB_PATH!
    )
)
if !SUB_COUNT!==0 (
    echo       [WARN] No submodule status returned -- submodules may not be initialized
    echo              Run: git submodule update --init --recursive
    echo              Or run: update-MarsCommonFtc
    set /a WARNINGS+=1
)
git config core.hooksPath .githooks >nul 2>&1
git update-index --chmod=+x .githooks/pre-commit >nul 2>&1
if not exist "!USERPROFILE!\.githooks\google-java-format.jar" (
    echo       [WARN] google-java-format.jar not found at !USERPROFILE!\.githooks\google-java-format.jar
    set DL_CHOICE=
    set /p DL_CHOICE="             Download latest release now? [Y/N] "
    if /i "!DL_CHOICE!"=="Y" (
        if not exist "!USERPROFILE!\.githooks" mkdir "!USERPROFILE!\.githooks"
        echo              Fetching latest release info...
        set GJF_JAR=!USERPROFILE!\.githooks\google-java-format.jar
        set DL_URL=
        set DL_OK=0
        for /f "usebackq delims=" %%U in (`powershell -NoProfile -ExecutionPolicy Bypass -Command "$r=Invoke-RestMethod 'https://api.github.com/repos/google/google-java-format/releases/latest'; ($r.assets | Where-Object { $_.name -like '*all-deps*' }).browser_download_url"`) do (
            if "!DL_URL!"=="" set DL_URL=%%U
        )
        if not "!DL_URL!"=="" (
            powershell -NoProfile -ExecutionPolicy Bypass -Command "Invoke-WebRequest -Uri '!DL_URL!' -OutFile '!GJF_JAR!'" >nul 2>&1
            if exist "!GJF_JAR!" (
                echo       [OK]   Downloaded google-java-format.jar
                set DL_OK=1
            )
        )
        if !DL_OK!==0 (
            echo       [WARN] Download failed -- install manually from https://github.com/google/google-java-format/releases/latest
            set /a WARNINGS+=1
        )
    ) else (
        echo              Pre-commit Java formatting will be skipped until installed
        set /a WARNINGS+=1
    )
)

:: ============================================================
:: CHECK 10/10: Gradle offline mode
:: ============================================================
echo [10/10] Gradle offline mode...
set OFFLINE_FOUND=0

if exist "!SCRIPT_DIR!\gradle.properties" (
    call :check_gradle_offline_file "!SCRIPT_DIR!\gradle.properties"
)

if defined USERPROFILE if exist "!USERPROFILE!\.gradle\gradle.properties" (
    call :check_gradle_offline_file "!USERPROFILE!\.gradle\gradle.properties"
)

if !OFFLINE_FOUND!==0 (
    echo       [OK]   Gradle offline mode not forced
) else (
    echo       [WARN] Gradle offline mode is enabled ^(org.gradle.offline=true^)
    echo              This can cause missing dependency errors on clean machines
    set /a WARNINGS+=1
)

:: ============================================================
:: SUMMARY
:: ============================================================
:summary
echo.
echo =============================================
if !ERRORS!==0 if !WARNINGS!==0 (
    echo  Result: All checks passed
) else (
    if !ERRORS! GTR 0 echo  Errors  : !ERRORS!  ^(build will fail^)
    if !WARNINGS! GTR 0 echo  Warnings: !WARNINGS!  ^(some features may not build^)
)
echo =============================================
echo.

if !ERRORS! GTR 0 (
    echo Fix the errors above before building.
    if /i not "%~1"=="Y" if /i not "%~1"=="N" pause
    exit /b 1
)

:: ============================================================
:: OFFER GRADLE BUILD
:: ============================================================
set BUILD_CHOICE=
if not "%~1"=="" set BUILD_CHOICE=%~1
if /i not "!BUILD_CHOICE!"=="Y" if /i not "!BUILD_CHOICE!"=="N" (
    set /p BUILD_CHOICE="All critical checks passed. Run a full Gradle build now? [Y/N] "
)
if /i "!BUILD_CHOICE!"=="Y" (
    echo.
    echo Running: gradlew.bat assembleDebug
    echo This may take several minutes on first run...
    echo.
    call "!SCRIPT_DIR!\gradlew.bat" assembleDebug
    if !errorlevel!==0 (
        echo.
        echo Build succeeded.
    ) else (
        echo.
        echo Build failed. Check the output above for errors.
    )
)

echo.
if /i not "%~1"=="Y" if /i not "%~1"=="N" pause
exit /b 0

:normalize_jdk_version
setlocal EnableDelayedExpansion
set "_v=!%~1!"
set "_v=!_v:"=!"
for /f "tokens=1 delims=+-" %%A in ("!_v!") do set "_v=%%A"
set "_norm=!_v!"
echo !_v! | findstr /r "^[0-9][0-9]*u[0-9][0-9]*$" >nul
if !errorlevel!==0 (
    for /f "tokens=1,2 delims=u" %%A in ("!_v!") do set "_norm=%%A.0.%%B"
)
endlocal & set "%~2=%_norm%"
exit /b 0

:check_gradle_offline_file
set "_OFFLINE_FILE=%~1"
for /f "usebackq tokens=1,* delims==" %%A in ("%_OFFLINE_FILE%") do (
    set "_K=%%A"
    set "_V=%%B"
    set "_K=!_K: =!"
    set "_V=!_V: =!"
    if /i "!_K!"=="org.gradle.offline" (
        if /i "!_V!"=="true" (
            echo       [WARN] org.gradle.offline=true set in: %_OFFLINE_FILE%
            set OFFLINE_FOUND=1
        )
    )
)
exit /b 0
