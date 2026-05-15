#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_LIBAVFILTER="$SCRIPT_DIR/libavfilter"

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 /path/to/ffmpeg"
    exit 1
fi

FFMPEG_DIR="$1"
TARGET_LIBAVFILTER="$FFMPEG_DIR/libavfilter"
TARGET_X86="$TARGET_LIBAVFILTER/x86"

# Validate FFmpeg directory
if [[ ! -d "$FFMPEG_DIR" ]]; then
    echo "ERROR: FFmpeg directory does not exist:"
    echo "  $FFMPEG_DIR"
    exit 1
fi

if [[ ! -d "$TARGET_LIBAVFILTER" ]]; then
    echo "ERROR: Missing directory:"
    echo "  $TARGET_LIBAVFILTER"
    exit 1
fi

if [[ ! -d "$TARGET_X86" ]]; then
    echo "ERROR: Missing directory:"
    echo "  $TARGET_X86"
    exit 1
fi

# Validate source files
REQUIRED_FILES=(
    "$SOURCE_LIBAVFILTER/x86/vf_vca.asm"
    "$SOURCE_LIBAVFILTER/x86/vf_vca_init.c"
    "$SOURCE_LIBAVFILTER/vca_dct.h"
    "$SOURCE_LIBAVFILTER/vca_dct.c"
    "$SOURCE_LIBAVFILTER/vf_vca.c"
)

for file in "${REQUIRED_FILES[@]}"; do
    if [[ ! -f "$file" ]]; then
        echo "ERROR: Missing source file:"
        echo "  $file"
        exit 1
    fi
done

# Copy files
cp "$SOURCE_LIBAVFILTER/x86/vf_vca.asm"       "$TARGET_X86/"
echo "Created: libavfilter/x86/vf_vca.asm"
cp "$SOURCE_LIBAVFILTER/x86/vf_vca_init.c"    "$TARGET_X86/"
echo "Created: libavfilter/x86/vf_vca_init.c"
cp "$SOURCE_LIBAVFILTER/vca_dct.h"            "$TARGET_LIBAVFILTER/"
echo "Created: libavfilter/vca_dct.h"
cp "$SOURCE_LIBAVFILTER/vca_dct.c"            "$TARGET_LIBAVFILTER/"
echo "Created: libavfilter/vca_dct.c"
cp "$SOURCE_LIBAVFILTER/vf_vca.c"             "$TARGET_LIBAVFILTER/"
echo "Created: libavfilter/vf_vca.c"
insert_after_last_match() {
    local file="$1"
    local regex="$2"
    local newline="$3"

    if [[ ! -f "$file" ]]; then
        echo "ERROR: Missing file:"
        echo "  $file"
        exit 1
    fi

    # Skip if already present
    if grep -Fqx "$newline" "$file"; then
    echo "Already exists in ${file#$FFMPEG_DIR/}"
        return
    fi

    # Find last matching line number
    local last_line
    last_line=$(grep -nE "$regex" "$file" | tail -n1 | cut -d: -f1)

    if [[ -z "$last_line" ]]; then
        echo "ERROR: Could not find insertion point in:"
        echo "  $file"
        echo "Regex:"
        echo "  $regex"
        exit 1
    fi

    # Handle multiline continuations ending with '\'
    local insert_line="$last_line"
    local total_lines
    total_lines=$(wc -l < "$file")

    while [[ "$insert_line" -lt "$total_lines" ]]; do
        local current_line
        current_line=$(sed -n "${insert_line}p" "$file")

        # Remove trailing whitespace
        current_line="${current_line%"${current_line##*[![:space:]]}"}"

        # Continue while line ends with '\'
        if [[ "$current_line" == *\\ ]]; then
            ((insert_line++))
        else
            break
        fi
    done

if [[ "$OSTYPE" == "darwin"* ]]; then
    sed -i '' "${insert_line}a\\
$newline
" "$file"
else
    sed -i "${insert_line}a\\
$newline
" "$file"
fi

    echo "Updated: ${file#$FFMPEG_DIR/}"
}

# Appends inside Makefiles and allfilters.c after the last filter entry in each file
insert_after_last_match \
    "$TARGET_X86/Makefile" \
    '^X86ASM-OBJS-\$\(CONFIG_.*_FILTER\)' \
    'X86ASM-OBJS-$(CONFIG_VCA_FILTER)             += x86/vf_vca.o x86/vf_vca_init.o'

insert_after_last_match \
    "$TARGET_LIBAVFILTER/Makefile" \
    '^OBJS-\$\(CONFIG_.*_FILTER\)' \
    'OBJS-$(CONFIG_VCA_FILTER)                    += vf_vca.o vca_dct.o'

insert_after_last_match \
    "$TARGET_LIBAVFILTER/allfilters.c" \
    '^extern const FFFilter ff_vf_.*;' \
    'extern const FFFilter ff_vf_vca;'

echo
echo "VCA filter installation completed successfully."