#!/bin/bash
# Format Java files with google-java-format (AOSP style) after Claude writes/edits them.
# JAR location: $HOME/.claude/google-java-format.jar
# Download: https://github.com/google/google-java-format/releases/latest

JAR="$HOME/.claude/google-java-format.jar"

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

java -jar "$JAR" --aosp --replace "$FILE_PATH"
