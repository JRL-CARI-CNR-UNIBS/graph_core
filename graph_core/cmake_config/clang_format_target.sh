#!/bin/bash

# Directory base del progetto (una directory sopra lo script)
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "Checking for clang-format in $PROJECT_DIR..."

if ! command -v clang-format > /dev/null 2>&1; then
    echo "clang-format not found, skipping format target"
    exit 0 
fi

echo "Searching for .clang-format file..."

# Cerca il file .clang-format nella directory del progetto
CLANG_FORMAT_FILE=$(find "$PROJECT_DIR" -name ".clang-format" -print -quit)
if [ -z "$CLANG_FORMAT_FILE" ]; then
    echo "No .clang-format file found in $PROJECT_DIR, skipping format target"
    exit 0
fi

echo "Found .clang-format file at $CLANG_FORMAT_FILE"

echo "Running clang-format on the following files..."

# Cerca i file .cpp e .h nella directory del progetto
FILES=$(find "$PROJECT_DIR" -type f \( -name "*.cpp" -o -name "*.h" \))

if [ -z "$FILES" ]; then
    echo "No source files found in $PROJECT_DIR, skipping format target"
    exit 0
fi

echo "$FILES"

# Esegui clang-format su ciascun file
for FILE in $FILES; do
    clang-format -i --style=file --assume-filename="$CLANG_FORMAT_FILE" "$FILE"
done

echo "clang-format completed successfully."
