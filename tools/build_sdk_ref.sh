#!/bin/bash
# Reference build of an AC695N soundbox SDK tree with JieLi's Windows toolchain (C:\JL\pi32), driven by WSL's make.
# Usage (in WSL): bash build_sdk_ref.sh /mnt/c/.../ac695n_sdk_3.1.0 LOGFILE
# Builds only pre_build + the ELF; the post-build script (packers / USB downloader) is not run.
set -u
SDK="$1"; LOG="$2"
FORCE=(-B)
# Only use incremental after source-only edits. Header/configuration changes
# require the default full rebuild because Windows .d files are not consumed.
if [ "${3:-}" = incremental ]; then FORCE=(); fi
TC=/mnt/c/JL/pi32/bin
cd "$SDK" || exit 2
ulimit -n 8192 2>/dev/null
{
  echo "start $(date)"
  # Windows clang emits dependency files with drive colons/backslashes that
  # WSL make cannot parse. Always rebuild; do not consume those .d files.
  make "${FORCE[@]}" -j8 pre_build cpu/br23/tools/sdk.elf DEP_FILES= \
    CC="$TC/clang.exe" CXX="$TC/clang.exe" LD="$TC/pi32v2-lto-wrapper.exe" AR="$TC/pi32v2-lto-ar.exe" \
    SYS_LIB_DIR=C:/JL/pi32/pi32v2-lib/r3 SYS_INC_DIR=C:/JL/pi32/pi32v2-include \
    MKDIR="mkdir -p" FIXBAT=true EXT_CFLAGS=
  result=$?
  echo "make exit $result"
  echo "end $(date)"
} > "$LOG" 2>&1
exit "$result"
