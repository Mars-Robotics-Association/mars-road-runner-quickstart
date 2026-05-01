#!/bin/bash
# Format Java files with google-java-format (AOSP style) after Claude writes/edits them.
# JAR location: $HOME/.claude/google-java-format.jar
# Download: https://github.com/google/google-java-format/releases/latest

JAR="$HOME/.claude/google-java-format.jar"

# Find a Java 21+ binary. Tries in order:
#   1. $JAVA_HOME  2. PATH  3. Common Windows dirs  4. macOS  5. Linux
find_java21() {
    is_java21() {
        local v
        v=$("$1" -version 2>&1 | awk -F'"' '/version/{print $2}' | cut -d. -f1)
        [[ "$v" =~ ^[0-9]+$ ]] && (( v >= 21 ))
    }

    # 1. JAVA_HOME
    if [[ -n "$JAVA_HOME" && -x "$JAVA_HOME/bin/java" ]]; then
        is_java21 "$JAVA_HOME/bin/java" && echo "$JAVA_HOME/bin/java" && return
    fi

    # 2. java on PATH
    if command -v java &>/dev/null; then
        is_java21 "$(command -v java)" && echo "$(command -v java)" && return
    fi

    # 3. Common Windows install dirs (Git Bash paths)
    local dir
    for dir in \
        "/c/Program Files/Eclipse Adoptium" \
        "/c/Program Files/Java" \
        "/c/Program Files/Microsoft" \
        "/c/Program Files/BellSoft" \
        "$HOME/.jdks"; do
        [[ -d "$dir" ]] || continue
        local java_bin
        for java_bin in "$dir"/jdk-2[1-9]*/bin/java "$dir"/jdk-2[1-9]*/bin/java.exe; do
            [[ -x "$java_bin" ]] && is_java21 "$java_bin" && echo "$java_bin" && return
        done
    done

    # 4. macOS
    if [[ -x /usr/libexec/java_home ]]; then
        local jhome
        jhome=$(/usr/libexec/java_home -v 21 2>/dev/null) && echo "$jhome/bin/java" && return
    fi

    # 5. Linux
    for java_bin in /usr/lib/jvm/java-2[1-9]-*/bin/java /usr/lib/jvm/temurin-2[1-9]-*/bin/java; do
        [[ -x "$java_bin" ]] && is_java21 "$java_bin" && echo "$java_bin" && return
    done

    return 1
}

INPUT=$(cat)
FILE_PATH=$(echo "$INPUT" | jq -r '.tool_input.file_path // empty')

# Only process Java files
if [[ -z "$FILE_PATH" || ! "$FILE_PATH" =~ \.java$ ]]; then
    exit 0
fi

# Skip silently if JAR not found
if [[ ! -f "$JAR" ]]; then
    exit 0
fi

JAVA=$(find_java21) || JAVA=java  # fall back to PATH java (will error if too old)

"$JAVA" -jar "$JAR" --aosp --skip-removing-unused-imports --replace "$FILE_PATH"

