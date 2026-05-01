#!/usr/bin/env bash
set -u

ERRORS=0
WARNINGS=0
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

normalize_jdk_version() {
  local raw="${1:-}"
  raw="${raw%%+*}"
  raw="${raw%%-*}"
  if [[ "$raw" =~ ^([0-9]+)u([0-9]+)$ ]]; then
    echo "${BASH_REMATCH[1]}.0.${BASH_REMATCH[2]}"
    return 0
  fi
  echo "$raw" | sed -E 's/^([0-9]+)\.([0-9]+)\.([0-9]+).*$/\1.\2.\3/'
}

printf '=============================================\n'
printf ' FTC TeamCode Pre-Flight Check\n'
printf '=============================================\n\n'

echo "Reading build requirements via Gradle..."
REQ_JAVA=""
REQ_SDK=""
REQ_NDK_MAIN=""
REQ_NDK_RUCKIG=""
REQ_CMAKE=""
TEMP_VERSIONS="${TMPDIR:-/tmp}/preflight_$RANDOM.txt"

cd "$SCRIPT_DIR" || exit 1
if ! ./gradlew -q --warning-mode=none --init-script "$SCRIPT_DIR/preflight-versions.gradle" printBuildVersions > "$TEMP_VERSIONS" 2>/dev/null; then
  rm -f "$TEMP_VERSIONS"
  echo "  [FAIL] Gradle query failed. Could not read build requirements."
  ERRORS=$((ERRORS+1))
  goto_summary=1
else
  goto_summary=0
fi

if [[ $goto_summary -eq 0 ]]; then
  while IFS='=' read -r key val; do
    case "$key" in
      PREFLIGHT_JAVA) REQ_JAVA="$val" ;;
      PREFLIGHT_COMPILE_SDK) REQ_SDK="$val" ;;
      PREFLIGHT_NDK_MAIN) REQ_NDK_MAIN="$val" ;;
      PREFLIGHT_NDK_RUCKIG) REQ_NDK_RUCKIG="$val" ;;
      PREFLIGHT_CMAKE_MIN) REQ_CMAKE="${val/+}" ;;
    esac
  done < "$TEMP_VERSIONS"
  rm -f "$TEMP_VERSIONS"

  for k in REQ_JAVA REQ_SDK REQ_NDK_MAIN REQ_NDK_RUCKIG REQ_CMAKE; do
    v="${!k}"
    if [[ -z "$v" ]]; then
      echo "  [FAIL] Missing ${k#REQ_} from Gradle output."
      ERRORS=$((ERRORS+1))
    elif [[ "${v,,}" == "unknown" ]]; then
      echo "  [FAIL] ${k#REQ_} returned 'unknown'."
      ERRORS=$((ERRORS+1))
    fi
  done
fi

if [[ $ERRORS -gt 0 ]]; then
  goto_summary=1
else
  echo "  Java      : $REQ_JAVA"
  echo "  API level : android-$REQ_SDK"
  echo "  NDK main  : $REQ_NDK_MAIN"
  echo "  NDK C++20 : $REQ_NDK_RUCKIG"
  echo "  CMake min : $REQ_CMAKE"
  echo
  REQ_CMAKE_MAJOR="${REQ_CMAKE%%.*}"
  REQ_CMAKE_MINOR_TMP="${REQ_CMAKE#*.}"
  REQ_CMAKE_MINOR="${REQ_CMAKE_MINOR_TMP%%.*}"
fi

if [[ $goto_summary -eq 0 ]]; then
  echo "[1/10] JDK version..."
  JAVA_VER_LINE="$(java -version 2>&1 | head -n1)"
  JAVA_FOUND_RAW=""
  JAVA_FOUND_NORM=""
  if [[ -z "$JAVA_VER_LINE" ]]; then
    echo "      [FAIL] java not found on PATH"
    echo "             Install JDK $REQ_JAVA and add it to PATH (or set JAVA_HOME)"
    ERRORS=$((ERRORS+1))
  else
    JAVA_MAJOR="$(echo "$JAVA_VER_LINE" | sed -E 's/.*\"([0-9]+).*/\1/')"
    JAVA_FOUND_RAW="$(echo "$JAVA_VER_LINE" | sed -E 's/.*\"([^\"]+)\".*/\1/')"
    JAVA_FOUND_NORM="$(normalize_jdk_version "$JAVA_FOUND_RAW")"
    if [[ "$JAVA_MAJOR" == "$REQ_JAVA" ]]; then
      echo "      [OK]   $JAVA_VER_LINE"
    else
      echo "      [FAIL] JDK $REQ_JAVA required. Found: $JAVA_VER_LINE"
      ERRORS=$((ERRORS+1))
    fi
  fi

  echo "[2/10] JDK security baseline (OpenJDK advisory)..."
  if [[ -z "$JAVA_FOUND_NORM" ]]; then
    echo "      [SKIP] JDK version unknown"
  elif ! command -v curl >/dev/null 2>&1; then
    echo "      [WARN] curl not found; cannot check latest OpenJDK advisory online"
    WARNINGS=$((WARNINGS+1))
  else
    ADVISORY_INDEX_URL="https://openjdk.org/groups/vulnerability/advisories/"
    ADVISORY_INDEX_HTML="$(curl -fsSL "$ADVISORY_INDEX_URL" 2>/dev/null || true)"
    if [[ -z "$ADVISORY_INDEX_HTML" ]]; then
      echo "      [WARN] Could not reach $ADVISORY_INDEX_URL"
      WARNINGS=$((WARNINGS+1))
    else
      LATEST_PATH="$(echo "$ADVISORY_INDEX_HTML" | grep -Eo 'href="[0-9]{4}-[0-9]{2}-[0-9]{2}"' | head -n1 || true)"
      if [[ -z "$LATEST_PATH" ]]; then
        echo "      [WARN] Could not parse latest advisory URL from OpenJDK"
        WARNINGS=$((WARNINGS+1))
      else
        LATEST_PATH="${LATEST_PATH#href=\"}"
        LATEST_PATH="${LATEST_PATH%\"}"
        LATEST_URL="https://openjdk.org/groups/vulnerability/advisories/${LATEST_PATH}"
        LATEST_HTML="$(curl -fsSL "$LATEST_URL" 2>/dev/null || true)"
        AFFECTED_LINE="$(echo "$LATEST_HTML" | grep -Eo 'affected versions are [^<]+' | head -n1 || true)"
        REQUIRED_FOR_MAJOR=""
        if [[ -n "$AFFECTED_LINE" ]]; then
          REQUIRED_FOR_MAJOR="$(echo "$AFFECTED_LINE" | grep -Eo "${REQ_JAVA}([.][0-9]+[.][0-9]+|u[0-9]+)" | head -n1 || true)"
        fi
        if [[ -z "$REQUIRED_FOR_MAJOR" ]]; then
          echo "      [WARN] Could not find JDK $REQ_JAVA baseline in latest advisory"
          WARNINGS=$((WARNINGS+1))
        else
          REQUIRED_NORM="$(normalize_jdk_version "$REQUIRED_FOR_MAJOR")"
          LOWEST="$(printf "%s\n%s\n" "$REQUIRED_NORM" "$JAVA_FOUND_NORM" | sort -V | head -n1)"
          if [[ "$LOWEST" == "$REQUIRED_NORM" && "$JAVA_FOUND_NORM" != "$REQUIRED_NORM" ]]; then
            echo "      [FAIL] Installed JDK $JAVA_FOUND_RAW is older than security baseline $REQUIRED_FOR_MAJOR"
            echo "             Advisory: $LATEST_URL"
            ERRORS=$((ERRORS+1))
          else
            echo "      [OK]   JDK $JAVA_FOUND_RAW meets or exceeds baseline $REQUIRED_FOR_MAJOR"
          fi
        fi
      fi
    fi
  fi

  echo "[3/10] local.properties..."
  SDK_DIR=""
  if [[ ! -f "$SCRIPT_DIR/local.properties" ]]; then
    echo "      [FAIL] local.properties not found"
    ERRORS=$((ERRORS+1))
  else
    SDK_DIR_LINE="$(grep -E '^sdk\.dir=' "$SCRIPT_DIR/local.properties" | head -n1 || true)"
    SDK_DIR="${SDK_DIR_LINE#sdk.dir=}"
    SDK_DIR="${SDK_DIR//\\:/:}"
    SDK_DIR="${SDK_DIR//\\//}"
    if [[ -z "$SDK_DIR" ]]; then
      echo "      [FAIL] sdk.dir not set in local.properties"
      ERRORS=$((ERRORS+1))
    else
      echo "      [OK]   local.properties present"
    fi
  fi

  echo "[4/10] Android SDK directory..."
  if [[ -z "$SDK_DIR" ]]; then
    echo "      [SKIP] SDK path unknown"
  elif [[ ! -d "$SDK_DIR" ]]; then
    echo "      [FAIL] Android SDK not found at: $SDK_DIR"
    ERRORS=$((ERRORS+1))
    SDK_DIR=""
  else
    echo "      [OK]   $SDK_DIR"
  fi

  echo "[5/10] Android SDK platform android-$REQ_SDK..."
  if [[ -z "$SDK_DIR" ]]; then
    echo "      [SKIP] SDK path unknown"
  elif [[ -d "$SDK_DIR/platforms/android-$REQ_SDK" ]]; then
    echo "      [OK]   android-$REQ_SDK installed"
  else
    echo "      [FAIL] android-$REQ_SDK not installed"
    ERRORS=$((ERRORS+1))
  fi

  echo "[6/10] Android NDK $REQ_NDK_MAIN (main build)..."
  if [[ -z "$SDK_DIR" ]]; then
    echo "      [SKIP] SDK path unknown"
  elif [[ -d "$SDK_DIR/ndk/$REQ_NDK_MAIN" ]]; then
    echo "      [OK]   NDK $REQ_NDK_MAIN found"
  else
    if [[ "${REQ_NDK_MAIN,,}" == "${REQ_NDK_RUCKIG,,}" ]]; then
      echo "      [WARN] NDK $REQ_NDK_MAIN not found"
      WARNINGS=$((WARNINGS+1))
    else
      echo "      [INFO] NDK $REQ_NDK_MAIN not found (separate NDK is configured for native modules)"
      echo "             Optional unless another module explicitly requires $REQ_NDK_MAIN"
    fi
  fi

  echo "[7/10] Android NDK $REQ_NDK_RUCKIG (RuckigNative / C++20)..."
  if [[ -z "$SDK_DIR" ]]; then
    echo "      [SKIP] SDK path unknown"
  elif [[ -d "$SDK_DIR/ndk/$REQ_NDK_RUCKIG" ]]; then
    echo "      [OK]   NDK $REQ_NDK_RUCKIG found"
  else
    echo "      [WARN] NDK $REQ_NDK_RUCKIG not found -- RuckigNative will not compile"
    WARNINGS=$((WARNINGS+1))
  fi

  echo "[8/10] CMake >= $REQ_CMAKE (RuckigNative native build)..."
  CMAKE_FOUND=0
  CMAKE_VER=""
  if command -v cmake >/dev/null 2>&1; then
    CMAKE_VER="$(cmake --version 2>/dev/null | head -n1 | awk '{print $3}')"
    CMAKE_FOUND=1
  fi
  if [[ $CMAKE_FOUND -eq 0 && -n "$SDK_DIR" && -d "$SDK_DIR/cmake" ]]; then
    while IFS= read -r -d '' c; do
      CMAKE_VER="$($c --version 2>/dev/null | head -n1 | awk '{print $3}')"
      CMAKE_FOUND=1
      break
    done < <(find "$SDK_DIR/cmake" -type f -path '*/bin/cmake' -print0 2>/dev/null)
  fi
  if [[ $CMAKE_FOUND -eq 0 ]]; then
    echo "      [WARN] CMake not found on PATH or in Android SDK"
    WARNINGS=$((WARNINGS+1))
  else
    FOUND_CMAKE_MAJOR="${CMAKE_VER%%.*}"
    FOUND_CMAKE_MINOR_TMP="${CMAKE_VER#*.}"
    FOUND_CMAKE_MINOR="${FOUND_CMAKE_MINOR_TMP%%.*}"
    CMAKE_OK=0
    if (( FOUND_CMAKE_MAJOR > REQ_CMAKE_MAJOR )); then CMAKE_OK=1; fi
    if (( FOUND_CMAKE_MAJOR == REQ_CMAKE_MAJOR )) && (( FOUND_CMAKE_MINOR >= REQ_CMAKE_MINOR )); then CMAKE_OK=1; fi
    if [[ $CMAKE_OK -eq 1 ]]; then
      echo "      [OK]   CMake $CMAKE_VER"
    else
      echo "      [WARN] CMake $CMAKE_VER found but >= $REQ_CMAKE required"
      WARNINGS=$((WARNINGS+1))
    fi
  fi

  echo "[9/10] Git submodules..."
  if ! git rev-parse --git-dir >/dev/null 2>&1; then
    echo "      [FAIL] Not in a git repository or git not found"
    ERRORS=$((ERRORS+1))
  else
    SUB_COUNT=0
    while read -r hash path _; do
      [[ -z "${hash:-}" ]] && continue
      SUB_COUNT=$((SUB_COUNT+1))
      flag="${hash:0:1}"
        case "$flag" in
          -)
            echo "      [FAIL] $path -- not initialized"
            echo "             Run: git submodule update --init --recursive"
            echo "             Or run: ./update-MarsCommonFtc.sh"
            ERRORS=$((ERRORS+1))
            ;;
        +)
          echo "      [WARN] $path -- at a different commit than expected"
          WARNINGS=$((WARNINGS+1))
          ;;
        U)
          echo "      [FAIL] $path -- has unresolved merge conflicts"
          ERRORS=$((ERRORS+1))
          ;;
        *)
          echo "      [OK]   $path"
          ;;
      esac
    done < <(git submodule status --recursive 2>/dev/null)
    if [[ $SUB_COUNT -eq 0 ]]; then
      echo "      [WARN] No submodule status returned -- submodules may not be initialized"
      echo "             Run: git submodule update --init --recursive"
      echo "             Or run: ./update-MarsCommonFtc.sh"
      WARNINGS=$((WARNINGS+1))
    fi
  fi

  echo "[10/10] Gradle offline mode..."
  OFFLINE_FOUND=0
  check_offline_file() {
    local f="$1"
    [[ -f "$f" ]] || return 0
    if grep -E '^[[:space:]]*org\.gradle\.offline[[:space:]]*=[[:space:]]*true[[:space:]]*$' "$f" >/dev/null 2>&1; then
      echo "      [WARN] org.gradle.offline=true set in: $f"
      OFFLINE_FOUND=1
    fi
  }
  check_offline_file "$SCRIPT_DIR/gradle.properties"
  check_offline_file "$HOME/.gradle/gradle.properties"
  if [[ $OFFLINE_FOUND -eq 0 ]]; then
    echo "      [OK]   Gradle offline mode not forced"
  else
    echo "      [WARN] Gradle offline mode is enabled (org.gradle.offline=true)"
    echo "             This can cause missing dependency errors on clean machines"
    WARNINGS=$((WARNINGS+1))
  fi
fi

echo
printf '=============================================\n'
if [[ $ERRORS -eq 0 && $WARNINGS -eq 0 ]]; then
  echo " Result: All checks passed"
else
  [[ $ERRORS -gt 0 ]] && echo " Errors  : $ERRORS  (build will fail)"
  [[ $WARNINGS -gt 0 ]] && echo " Warnings: $WARNINGS  (some features may not build)"
fi
printf '=============================================\n\n'

if [[ $ERRORS -gt 0 ]]; then
  echo "Fix the errors above before building."
  exit 1
fi

BUILD_CHOICE="${1:-}"
if [[ "${BUILD_CHOICE^^}" != "Y" && "${BUILD_CHOICE^^}" != "N" ]]; then
  read -r -p "All critical checks passed. Run a full Gradle build now? [Y/N] " BUILD_CHOICE
fi
if [[ "${BUILD_CHOICE^^}" == "Y" ]]; then
  echo
  echo "Running: ./gradlew assembleDebug"
  echo "This may take several minutes on first run..."
  echo
  if ./gradlew assembleDebug; then
    echo
    echo "Build succeeded."
  else
    echo
    echo "Build failed. Check the output above for errors."
  fi
fi

exit 0
