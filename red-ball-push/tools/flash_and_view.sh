#!/bin/zsh
# Flash this camera diagnostic, then immediately open its Mac preview page.
# Run through the VS Code task "Camera: Flash and Open Preview".

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SERIAL_PORT="${1:-/dev/cu.usbserial-0001}"
IDF_PATH="/Users/liupeng/.espressif/v5.4.4/esp-idf"
IDF_TOOLS_PATH="/Users/liupeng/.espressif"
IDF_PYTHON="/Users/liupeng/.espressif/tools/python/v5.4.4/venv/bin/python"
NINJA_DIR="/Users/liupeng/.espressif/tools/ninja/1.12.1"
XTENSA_S3_BIN="/Users/liupeng/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20260121/xtensa-esp-elf/bin"
VENV_DIR="$PROJECT_DIR/.venv"

if [[ ! -e "$SERIAL_PORT" ]]; then
    print "找不到串口：$SERIAL_PORT"
    print "请重新插拔开发板，或在 .vscode/settings.json 中更新 idf.port。"
    exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
    print "未找到 python3，无法启动电脑端图像查看器。"
    exit 1
fi

if [[ ! -x "$IDF_PYTHON" ]]; then
    print "找不到 ESP-IDF 5.4.4 的 Python 环境：$IDF_PYTHON"
    print "请先在 VS Code 的 ESP-IDF 面板完成一次安装配置。"
    exit 1
fi

print "[1/2] 正在烧录摄像头固件…"
export IDF_PATH
export IDF_TOOLS_PATH
export IDF_PYTHON_ENV_PATH="/Users/liupeng/.espressif/tools/python/v5.4.4/venv"
export IDF_PYTHON_CHECK_CONSTRAINTS=0
export PYTHON="$IDF_PYTHON"
export PATH="$XTENSA_S3_BIN:$NINJA_DIR:$PATH"
# Do not source export.sh here: it searches for an older Python environment
# that is not present on this Mac. The VS Code installation uses this v5.4.4
# environment, which is also the one used to compile this project. The local
# installation has no downloaded constraints file, so disable only that
# offline dependency check; the selected virtual environment is already known
# to build this project successfully.
"$IDF_PYTHON" "$IDF_PATH/tools/idf.py" -C "$PROJECT_DIR" -G Ninja -p "$SERIAL_PORT" flash

print "[2/2] 正在启动图像查看器并打开浏览器…"
if [[ ! -x "$VENV_DIR/bin/python" ]]; then
    python3 -m venv "$VENV_DIR"
    "$VENV_DIR/bin/python" -m pip install -r "$PROJECT_DIR/tools/requirements.txt"
fi

# Keep the task terminal as the viewer's status panel. Wait for the local web
# server to be ready before asking macOS to open it: opening after a fixed
# delay could leave the browser on a failed initial connection.
"$VENV_DIR/bin/python" "$PROJECT_DIR/tools/color_viewer.py" "$SERIAL_PORT" --http-port 8767 &
VIEWER_PID=$!
trap 'kill "$VIEWER_PID" 2>/dev/null || true' EXIT INT TERM
VIEWER_URL="http://127.0.0.1:8767/"
VIEWER_READY=0
for attempt in {1..50}; do
    if /usr/bin/curl --silent --fail --max-time 1 "$VIEWER_URL" >/dev/null 2>&1; then
        VIEWER_READY=1
        break
    fi
    sleep 0.1
done

if [[ "$VIEWER_READY" -ne 1 ]]; then
    print "图像查看器未能在 5 秒内启动，未打开浏览器。"
    exit 1
fi

# This Mac has Safari installed. Naming the app explicitly avoids VS Code task
# shells silently handing the URL to a non-interactive/default handler.
/usr/bin/open -a "Safari" "$VIEWER_URL"
print "已自动在 Safari 打开图像页：$VIEWER_URL"
wait "$VIEWER_PID"
