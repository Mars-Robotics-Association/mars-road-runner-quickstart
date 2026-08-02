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

# Returns the JDK home path configured as Android Studio's Gradle JDK, or exits non-zero.
resolve_as_jdk_home() {
  local idea_gradle="$SCRIPT_DIR/.idea/gradle.xml"
  [[ -f "$idea_gradle" ]] || return 1

  local gradle_jvm
  gradle_jvm="$(grep -o 'gradleJvm" value="[^"]*"' "$idea_gradle" \
    | sed 's/.*value="\([^"]*\)".*/\1/' | head -1)"
  [[ -z "$gradle_jvm" ]] && return 1

  if [[ "$gradle_jvm" == "#JAVA_HOME" ]]; then
    [[ -n "${JAVA_HOME:-}" && -x "$JAVA_HOME/bin/java" ]] && echo "$JAVA_HOME" && return 0
    return 1
  fi

  # Android Studio "Gradle local JDK" -- the path lives in .gradle/config.properties.
  if [[ "$gradle_jvm" == "#GRADLE_LOCAL_JAVA_HOME" ]]; then
    local cfg="$SCRIPT_DIR/.gradle/config.properties"
    [[ -f "$cfg" ]] || return 1
    local jh
    jh="$(grep -E '^java\.home=' "$cfg" | head -1)"
    jh="${jh#java.home=}"
    # Unescape Java .properties (\: -> :) then normalize all backslashes to / for bash.
    jh="${jh//\\:/:}"
    jh="${jh//\\//}"
    [[ -n "$jh" && -x "$jh/bin/java" ]] && echo "$jh" && return 0
    return 1
  fi

  # Absolute path (Windows C:\... or Unix /)
  if [[ "$gradle_jvm" =~ ^[A-Za-z]:\\ || "$gradle_jvm" == /* ]]; then
    echo "$gradle_jvm" && return 0
  fi

  # Named JDK -- search jdk.table.xml files, newest AS version first
  local appdata="${APPDATA:-}"
  [[ -z "$appdata" ]] && return 1
  if command -v cygpath >/dev/null 2>&1; then
    appdata="$(cygpath -u "$appdata")"
  fi

  local jdk_table result
  for jdk_table in $(ls -r "$appdata"/Google/AndroidStudio*/options/jdk.table.xml 2>/dev/null); do
    [[ -f "$jdk_table" ]] || continue
    result="$(awk -v target="$gradle_jvm" '
      /<jdk / { in_jdk=1; name=""; path="" }
      in_jdk {
        line = $0
        if (line ~ /name value=/) {
          sub(/.*name value="/, "", line); sub(/".*/, "", line); name = line
        }
        if (line ~ /homePath value=/) {
          sub(/.*homePath value="/, "", line); sub(/".*/, "", line); path = line
        }
        if (line ~ /<\/jdk>/) {
          if (name == target && path != "") print path
          in_jdk = 0
        }
      }
    ' "$jdk_table")"
    if [[ -n "$result" ]]; then
      result="${result/\$USER_HOME\$/$HOME}"
      echo "$result"
      return 0
    fi
  done
  return 1
}

printf '=============================================\n'
printf ' FTC TeamCode Pre-Flight Check\n'
printf '=============================================\n\n'

# Find the JDK that Gradle/Android Studio actually builds with (fall back to PATH),
# then run the build probe WITH that JDK. The probe both reads the project's version
# requirements and proves the JDK can configure the build (Gradle + AGP) -- the real
# "is my JDK healthy" question, independent of the source language level.
REQ_JAVA=""
REQ_SDK=""
goto_summary=0

cd "$SCRIPT_DIR" || exit 1

AS_JDK_HOME="$(resolve_as_jdk_home 2>/dev/null || true)"
if [[ -n "$AS_JDK_HOME" ]]; then
  JAVA_BIN="$AS_JDK_HOME/bin/java"
  GRADLE_JVM_LABEL="$(grep -o 'gradleJvm" value="[^"]*"' "$SCRIPT_DIR/.idea/gradle.xml" \
    | sed 's/.*value="\([^"]*\)".*/\1/' | head -1)"
  JDK_SOURCE="Android Studio ($GRADLE_JVM_LABEL)"
  PROBE_JAVA_HOME="$AS_JDK_HOME"
else
  JAVA_BIN="java"
  JDK_SOURCE="system PATH / JAVA_HOME"
  PROBE_JAVA_HOME="${JAVA_HOME:-}"
fi

JAVA_VER_LINE="$("$JAVA_BIN" -version 2>&1 | head -n1)"
JAVA_FOUND_RAW="$(echo "$JAVA_VER_LINE" | sed -E 's/.*"([^"]+)".*/\1/')"
JAVA_FOUND_NORM="$(normalize_jdk_version "$JAVA_FOUND_RAW")"
JDK_MAJOR="$(echo "$JAVA_FOUND_RAW" | cut -d. -f1)"

echo "[1/8] Build configuration with Gradle JDK..."
if [[ -n "$PROBE_JAVA_HOME" ]]; then
  PROBE_OUT="$(JAVA_HOME="$PROBE_JAVA_HOME" ./gradlew -q --warning-mode=none --init-script "$SCRIPT_DIR/preflight-versions.gradle" printBuildVersions 2>&1)"
  PROBE_RC=$?
else
  PROBE_OUT="$(./gradlew -q --warning-mode=none --init-script "$SCRIPT_DIR/preflight-versions.gradle" printBuildVersions 2>&1)"
  PROBE_RC=$?
fi
if [[ $PROBE_RC -ne 0 ]]; then
  echo "      [FAIL] Gradle could not configure the build with this JDK  [$JDK_SOURCE]"
  [[ -n "$JAVA_VER_LINE" ]] && echo "             JDK: $JAVA_VER_LINE"
  echo "             ---- gradle output (tail) ----"
  echo "$PROBE_OUT" | tail -n 12 | sed 's/^/             /'
  ERRORS=$((ERRORS+1))
  goto_summary=1
else
  while IFS='=' read -r key val; do
    val="${val%$'\r'}"
    case "$key" in
      PREFLIGHT_JAVA) REQ_JAVA="$val" ;;
      PREFLIGHT_COMPILE_SDK) REQ_SDK="$val" ;;
    esac
  done <<< "$PROBE_OUT"
  if [[ -z "$REQ_SDK" || "$(echo "$REQ_SDK" | tr '[:upper:]' '[:lower:]')" == "unknown" ]]; then
    echo "      [FAIL] Build configured, but compileSdk could not be read from Gradle"
    ERRORS=$((ERRORS+1))
    goto_summary=1
  else
    echo "      [OK]   ${JAVA_VER_LINE:-JDK present} configures the build  [$JDK_SOURCE]"
    echo "             Source level: Java ${REQ_JAVA:-unknown}    API level: android-$REQ_SDK"
  fi
fi

if [[ $goto_summary -eq 0 ]]; then
  echo "[2/8] JDK security baseline (OpenJDK advisory)..."
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
      LATEST_PATH="$(echo "$ADVISORY_INDEX_HTML" | grep -Eo '[0-9]{4}-[0-9]{2}-[0-9]{2}' | head -n1 || true)"
      if [[ -z "$LATEST_PATH" ]]; then
        echo "      [WARN] Could not parse latest advisory URL from OpenJDK"
        WARNINGS=$((WARNINGS+1))
      else
        LATEST_PATH="${LATEST_PATH#href=\"}"
        LATEST_PATH="${LATEST_PATH%\"}"
        LATEST_URL="https://openjdk.org/groups/vulnerability/advisories/${LATEST_PATH}"
        LATEST_HTML="$(curl -fsSL "$LATEST_URL" 2>/dev/null || true)"
        REQUIRED_FOR_MAJOR="$(echo "$LATEST_HTML" | grep -Eo "${JDK_MAJOR}([.][0-9]+[.][0-9]+|u[0-9]+)" | head -n1 || true)"
        if [[ -z "$REQUIRED_FOR_MAJOR" ]]; then
          echo "      [WARN] Could not find JDK $JDK_MAJOR baseline in latest advisory"
          WARNINGS=$((WARNINGS+1))
        else
          REQUIRED_NORM="$(normalize_jdk_version "$REQUIRED_FOR_MAJOR")"
          # FAIL when the installed JDK sorts lowest (older) than the baseline.
          LOWEST="$(printf "%s\n%s\n" "$REQUIRED_NORM" "$JAVA_FOUND_NORM" | sort -V | head -n1)"
          if [[ "$LOWEST" == "$JAVA_FOUND_NORM" && "$JAVA_FOUND_NORM" != "$REQUIRED_NORM" ]]; then
            echo "      [WARN] Installed JDK $JAVA_FOUND_RAW is older than security baseline $REQUIRED_FOR_MAJOR"
            echo "             Advisory: $LATEST_URL"
            WARNINGS=$((WARNINGS+1))
          else
            echo "      [OK]   JDK $JAVA_FOUND_RAW meets or exceeds baseline $REQUIRED_FOR_MAJOR"
          fi
        fi
      fi
    fi
  fi

  echo "[3/8] local.properties..."
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

  echo "[4/8] Android SDK directory..."
  if [[ -z "$SDK_DIR" ]]; then
    echo "      [SKIP] SDK path unknown"
  elif [[ ! -d "$SDK_DIR" ]]; then
    echo "      [FAIL] Android SDK not found at: $SDK_DIR"
    ERRORS=$((ERRORS+1))
    SDK_DIR=""
  else
    echo "      [OK]   $SDK_DIR"
  fi

  echo "[5/8] Android SDK platform android-$REQ_SDK..."
  if [[ -z "$SDK_DIR" ]]; then
    echo "      [SKIP] SDK path unknown"
  elif [[ -d "$SDK_DIR/platforms/android-$REQ_SDK" ]]; then
    echo "      [OK]   android-$REQ_SDK installed"
  else
    echo "      [FAIL] android-$REQ_SDK not installed"
    ERRORS=$((ERRORS+1))
  fi

  echo "[6/8] Git submodules..."
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
            echo "             Or run: ./update-submodules.sh"
            ERRORS=$((ERRORS+1))
            ;;
        +)
          CLEAN_HASH="${hash:1}"
          PARENT_HASH="$(git ls-files -s "$path" | awk '{print $2}')"
          if [[ -n "$PARENT_HASH" ]] && git -C "$path" merge-base --is-ancestor "$PARENT_HASH" "$CLEAN_HASH" 2>/dev/null; then
            echo "      [WARN] $path -- submodule has new commits not yet staged in the parent repo"
            echo "             Stage and commit: git add $path && git commit"
            echo "             Or discard:       git submodule update --recursive"
          else
            echo "      [WARN] $path -- submodule is out of sync; at a different commit than the parent repo expects"
            echo "             Run: git submodule update --recursive"
          fi
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
      echo "             Or run: ./update-submodules.sh"
      WARNINGS=$((WARNINGS+1))
    fi
    # Detect staged-but-uncommitted submodule pointer changes in the parent repo.
    # git submodule status reports clean when index == submodule HEAD, so this case
    # is invisible to the loop above even though the parent still needs a commit.
    while IFS= read -r sub_path; do
      [[ -z "$sub_path" ]] && continue
      if ! git diff --cached --quiet -- "$sub_path" 2>/dev/null; then
        echo "      [WARN] $sub_path -- parent repo has a staged submodule pointer update that has not been committed"
        echo "             Commit now to record the submodule update: git commit"
        WARNINGS=$((WARNINGS+1))
      fi
    done < <(git config --file .gitmodules --get-regexp 'submodule\..*\.path' 2>/dev/null | awk '{print $2}')

  echo "[7/8] Git hooks / Java formatter..."
    # Pin GJF to a release that runs on JDK 17 (project sourceCompatibility). 1.29+
    # references JCTree$JCAnyPattern and fails on Java 17 with NoClassDefFoundError.
    GJF_VERSION="1.28.0"
    GJF_JAR="$HOME/.githooks/google-java-format.jar"
    GJF_URL="https://github.com/google/google-java-format/releases/download/v${GJF_VERSION}/google-java-format-${GJF_VERSION}-all-deps.jar"
    git config core.hooksPath .githooks
    git update-index --chmod=+x .githooks/pre-commit 2>/dev/null || true
    chmod +x .githooks/pre-commit 2>/dev/null || true

    gjf_probe_ok() {
      local jar="$1"
      [[ -f "$jar" ]] || return 1
      command -v java >/dev/null 2>&1 || return 1
      echo 'class _GjfProbe { void m() { int x = 1; } }' \
        | java -jar "$jar" --aosp - >/dev/null 2>&1
    }

    need_gjf_install=0
    if [[ ! -f "$GJF_JAR" ]]; then
      echo "      [WARN] google-java-format.jar not found at $GJF_JAR"
      need_gjf_install=1
    elif ! gjf_probe_ok "$GJF_JAR"; then
      echo "      [WARN] $GJF_JAR is incompatible with the current Java runtime"
      echo "             (common with GJF 1.29+ on JDK 17). Will offer pinned v${GJF_VERSION}."
      need_gjf_install=1
    else
      echo "      [OK]   pre-commit hook configured (google-java-format works with this Java)"
    fi

    if [[ "$need_gjf_install" -eq 1 ]]; then
      if command -v curl >/dev/null 2>&1; then
        read -r -p "             Download google-java-format ${GJF_VERSION} now? [Y/N] " DL_CHOICE
        if [[ "$DL_CHOICE" == "Y" || "$DL_CHOICE" == "y" ]]; then
          mkdir -p "$HOME/.githooks"
          if curl -fsSL -o "$GJF_JAR" "$GJF_URL"; then
            if gjf_probe_ok "$GJF_JAR"; then
              echo "      [OK]   Installed google-java-format ${GJF_VERSION}"
            else
              echo "      [WARN] Downloaded jar still fails under $(java -version 2>&1 | head -1)"
              echo "             Need Java 17+ on PATH for the pre-commit hook."
              WARNINGS=$((WARNINGS+1))
            fi
          else
            echo "      [WARN] Download failed -- install manually:"
            echo "             $GJF_URL"
            WARNINGS=$((WARNINGS+1))
          fi
        else
          echo "             Pre-commit Java formatting will fail until a compatible jar is installed"
          WARNINGS=$((WARNINGS+1))
        fi
      else
        echo "             Install curl, or download manually:"
        echo "             $GJF_URL"
        WARNINGS=$((WARNINGS+1))
      fi
    fi
  fi

  echo "[8/8] Gradle offline mode..."
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
if [[ "$BUILD_CHOICE" != "Y" && "$BUILD_CHOICE" != "y" && "$BUILD_CHOICE" != "N" && "$BUILD_CHOICE" != "n" ]]; then
  read -r -p "All critical checks passed. Run a full Gradle build now? [Y/N] " BUILD_CHOICE
fi
if [[ "$BUILD_CHOICE" == "Y" || "$BUILD_CHOICE" == "y" ]]; then
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



