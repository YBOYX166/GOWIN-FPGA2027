#!/bin/bash
# 运行 Gowin gw_sh（命令行工具链），自动修正 macOS 下的动态库路径。
# 用法: ./gowin_run.sh <tcl脚本>
set -e
IDE="/Applications/GowinIDE.app/Contents/Resources/Gowin_EDA/IDE"
WORK="/tmp/gowin_cli"
if [ ! -x "$WORK/bin/gw_sh" ]; then
    mkdir -p "$WORK/bin"
    cp "$IDE/bin/gw_sh" "$WORK/bin/gw_sh"
    install_name_tool -change "/Library/Frameworks/Tcl.framework/Versions/8.6/Tcl" "$IDE/lib/Tcl.framework/Versions/8.6/Tcl" "$WORK/bin/gw_sh"
    install_name_tool -change "/opt/homebrew/opt/openssl@3/lib/libcrypto.3.dylib" "$IDE/lib/libcrypto.3.dylib" "$WORK/bin/gw_sh"
    install_name_tool -add_rpath "$IDE/lib" "$WORK/bin/gw_sh"
    codesign -f -s - "$WORK/bin/gw_sh" >/dev/null 2>&1 || true
    for f in "$IDE/bin"/*; do
        b=$(basename "$f")
        [ "$b" != "gw_sh" ] && ln -sfn "$f" "$WORK/bin/$b"
    done
    for d in data share ipcore simlib doc plugins; do
        ln -sfn "$IDE/$d" "$WORK/$d"
    done
fi
export DYLD_LIBRARY_PATH="$IDE/lib"
export DYLD_FRAMEWORK_PATH="$IDE/lib"
export QT_PLUGIN_PATH="$IDE/plugins/qt"
exec "$WORK/bin/gw_sh" "$@"
