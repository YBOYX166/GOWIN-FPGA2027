#!/bin/bash
# =====================================================================
# build_fs.sh —— 一条命令把 4K 工程编成可直接烧录的 .fs，放进 4k/fs/
#
#   ./tools/build_fs.sh                 # 生成 fs/servo_serial.fs
#   ./tools/build_fs.sh my_test         # 生成 fs/my_test.fs
#
# 设计要点：**文件列表直接从 fpga_project.gprj 读**，不写死在脚本里。
# 所以你以后往工程里加新 .v（或在 IDE 里拖动顺序），这里会自动跟上，
# 不会出现「IDE 编的是新代码、脚本编的是老代码」这种岔子。
#
# 后台是 Gowin EDA 的命令行工具 gw_sh。macOS 上它需要两个 DYLD 变量
# 才能找到自带的库，脚本已经处理。
# =====================================================================
set -euo pipefail

GW="/Applications/GowinIDE.app/Contents/Resources/Gowin_EDA/IDE"
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"                 # -> 4k/
GPRJ="$ROOT/fpga_project/fpga_project.gprj"
SRC="$ROOT/fpga_project/src"
OUT="$ROOT/fs"
NAME="${1:-servo_serial}"
WORK="/tmp/4k_build_$$"

if [ ! -x "$GW/bin/gw_sh" ]; then
  echo "找不到 Gowin EDA：$GW" >&2; exit 1
fi
if [ ! -f "$GPRJ" ]; then
  echo "找不到工程文件：$GPRJ" >&2; exit 1
fi

# ---- 从 .gprj 提取器件和文件列表 ----
DEV_NAME=$(sed -n 's/.*<Device name="\([^"]*\)".*/\1/p' "$GPRJ")
DEV_PN=$(sed -n 's/.*pn="\([^"]*\)".*/\1/p'     "$GPRJ")
if [ -z "$DEV_NAME" ] || [ -z "$DEV_PN" ]; then
  echo "从 .gprj 读器件失败（name/pn 是不是空的？）" >&2; exit 1
fi

# 所有 enable="1" 的源文件
# 注意：不能用 mapfile —— macOS 自带的是 bash 3.2，没这个命令。
# 也不要把 [ -n ] && cmd 放在循环体末尾，配 set -e 会在空行上直接退出。
FILES=()
while IFS= read -r line; do
  if [ -n "$line" ]; then
    FILES+=("$line")
  fi
done < <(grep -o 'path="[^"]*"[^>]*enable="1"' "$GPRJ" | sed 's/path="//; s/".*//')

NF=${#FILES[@]}
if [ "$NF" -eq 0 ]; then
  echo "从 .gprj 读文件列表失败（FileList 是空的？）" >&2; exit 1
fi

echo "器件   : $DEV_NAME  $DEV_PN"
echo "文件   : $NF 个"
for f in "${FILES[@]}"; do echo "         $f"; done
echo

# ---- 组装临时工作目录 + tcl 脚本 ----
rm -rf "$WORK"; mkdir -p "$WORK/src"
for f in "${FILES[@]}"; do
  [ -f "$ROOT/fpga_project/$f" ] && cp "$ROOT/fpga_project/$f" "$WORK/$f"
done

{
  echo "set_device -name $DEV_NAME $DEV_PN"
  for f in "${FILES[@]}"; do echo "add_file $f"; done
  echo "set_option -top_module top"
  # 引脚 10 是 MODE 专用脚，不放开就报 dedicated pin (MODE)
  echo "set_option -use_mode_as_gpio 1"
  # 如果以后加了 src/pnr_dual_purpose.tcl 里的其它选项，在这里补
  echo "run all"
} > "$WORK/build.tcl"

# ---- 编译 ----
cd "$WORK"
DYLD_LIBRARY_PATH="$GW/lib" DYLD_FRAMEWORK_PATH="$GW/lib" \
  "$GW/bin/gw_sh" build.tcl 2>&1 \
  | grep -viE "OpenGL|Chromium|Hardware accel|alog|license|GOWIN Tcl" \
  | grep -iE "^ERROR|Bitstream generation completed|Placement and routing completed" || true

if [ ! -f "$WORK/impl/pnr/project.fs" ]; then
  echo; echo "❌ 没有生成 project.fs，编译失败了" >&2
  rm -rf "$WORK"; exit 1
fi

# ---- 落地 ----
# 注意：Gowin 生成的 .fs / .bin 权限是 555（只读），cp 会把这个权限带过去，
#       于是第二次编译时覆盖就报 Permission denied。所以先删再拷，最后改回 644。
mkdir -p "$OUT"
rm -f "$OUT/$NAME.fs" "$OUT/$NAME.bin"
cp "$WORK/impl/pnr/project.fs"  "$OUT/$NAME.fs"
cp "$WORK/impl/pnr/project.bin" "$OUT/$NAME.bin"
chmod 644 "$OUT/$NAME.fs" "$OUT/$NAME.bin"

echo
echo "✅ 已生成"
ls -l "$OUT/$NAME.fs" "$OUT/$NAME.bin" | awk '{printf "   %-28s %10s bytes\n", $NF, $5}'
echo "   sha256: $(shasum -a 256 "$OUT/$NAME.fs" | cut -d' ' -f1)"

# ---- 打印时序和资源摘要，方便一眼看出有没有变差 ----
if [ -f "$WORK/impl/pnr/project.rpt.txt" ]; then
  echo
  echo "资源："
  grep -aE "^  (Logic|Register|CLS|DSP|BSRAM) " "$WORK/impl/pnr/project.rpt.txt" | sed 's/^/   /'
fi
if [ -f "$WORK/impl/pnr/project_tr_content.html" ]; then
  # 时序报告是 HTML 表格，剥标签后 "Actual Fmax" 和数值还落在不同行，
  # 所以换行也必须压平（第一次只压了空格，结果永远取不到）。
  # 那一行压平后长这样：
  #   Actual Fmax Logic Level Entity 1 clk27 27.000(MHz) 82.375(MHz) 7 TOP
  # 第一个 MHz 是约束值，第二个才是实测 Fmax。
  # 管道的 grep 找不到会返回 1，配 set -e 会把整个脚本干掉，所以都要 || true。
  TXT=$(sed 's/<[^>]*>/ /g' "$WORK/impl/pnr/project_tr_content.html" \
        | tr -s ' \n' ' ' || true)
  FMAX=$(printf '%s\n' "$TXT" | grep -o 'Actual Fmax.\{0,160\}' | head -1 \
         | grep -oE '[0-9]+\.[0-9]+\(MHz\)' | sed -n '2p' | tr -d '(MHz)' || true)
  TNS=$(printf '%s\n' "$TXT" | grep -o 'Total Negative Slack Summary.\{0,110\}' \
        | head -1 || true)
  echo
  echo "时序："
  echo "   实测 Fmax = ${FMAX:-未取到} MHz"
  echo "   ${TNS:-未取到}"
fi

# 顺手把 .fs 头里的 CheckSum 打出来 —— 这才是比特流本身的标识
echo
echo "比特流标识（.fs 文本头）："
head -c 700 "$OUT/$NAME.fs" | strings \
  | grep -aE "^//(Part Number|CheckSum|Created Time)" | sed 's/^/   /'

rm -rf "$WORK"
