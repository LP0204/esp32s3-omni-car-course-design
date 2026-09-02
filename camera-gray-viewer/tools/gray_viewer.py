#!/usr/bin/env python3
"""Display ESP32-S3 camera gray frames in a local browser.

The firmware sends a CRC-protected packet:
    "GRAY" + uint16 width + uint16 height + uint16 threshold
           + width*height gray bytes + uint16 CRC-16/XMODEM.

The visualization follows the supplied reference: enlarged side-by-side
gray/binary views, a triangular road ROI, virtual channel guides, and live
threshold adjustment. It deliberately uses a browser rather than Tk because
the Python currently installed on this Mac has no Tk GUI module.
"""

from __future__ import annotations

import argparse
import json
import struct
import threading
import time
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        "缺少 pyserial。请先运行：python3 -m pip install -r tools/requirements.txt"
    ) from exc


MAGIC = b"GRAY"
HEADER_BYTES = 10
MAX_WIDTH = 320
MAX_HEIGHT = 180


def crc16_xmodem(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class ViewerState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.gray: Optional[bytes] = None
        self.width = 80
        self.height = 60
        self.threshold = 100
        self.frames = 0
        self.started = time.monotonic()
        self.status = "等待 ESP32 图像帧…"

    def update_frame(self, gray: bytes, width: int, height: int, threshold: int) -> None:
        with self.lock:
            self.gray = gray
            self.width = width
            self.height = height
            self.threshold = threshold
            self.frames += 1
            elapsed = max(0.001, time.monotonic() - self.started)
            self.status = (
                f"{width}×{height} | {self.frames / elapsed:.1f} FPS | "
                f"阈值 {threshold}（灰阶小于阈值显示为黑）"
            )

    def set_status(self, status: str) -> None:
        with self.lock:
            self.status = status

    def snapshot(self) -> tuple[Optional[bytes], int, int, int, str]:
        with self.lock:
            return self.gray, self.width, self.height, self.threshold, self.status


STATE = ViewerState()


class SerialLink:
    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.lock = threading.Lock()
        self.port_handle: Optional[serial.Serial] = None
        self.stop_event = threading.Event()

    def send(self, character: bytes) -> None:
        with self.lock:
            if self.port_handle is None:
                return
            try:
                self.port_handle.write(character)
                self.port_handle.flush()
            except (serial.SerialException, OSError) as exc:
                STATE.set_status(f"串口发送失败：{exc}")

    def run(self) -> None:
        try:
            with serial.Serial(self.port, self.baud, timeout=0.1, rtscts=False, dsrdtr=False) as port:
                self.port_handle = port
                # Avoid a DTR/RTS-triggered reset after the port has opened.
                try:
                    port.dtr = False
                    port.rts = False
                except (serial.SerialException, OSError):
                    pass
                STATE.set_status("串口已连接，等待摄像头帧…")
                buffer = bytearray()
                last_start_command = 0.0
                while not self.stop_event.is_set():
                    now = time.monotonic()
                    # The first command can be lost if the board resets when the
                    # USB-UART port opens, so send v again every second.
                    if now - last_start_command >= 1.0:
                        self.send(b"v")
                        last_start_command = now
                    chunk = port.read(4096)
                    if chunk:
                        buffer.extend(chunk)
                        parse_frames(buffer)
        except (serial.SerialException, OSError) as exc:
            STATE.set_status(f"串口打开失败：{exc}")
        finally:
            self.port_handle = None

    def close(self) -> None:
        self.send(b"x")
        self.stop_event.set()


def parse_frames(buffer: bytearray) -> None:
    while True:
        start = buffer.find(MAGIC)
        if start < 0:
            if len(buffer) > len(MAGIC) - 1:
                del buffer[: -(len(MAGIC) - 1)]
            return
        if start:
            del buffer[:start]
        if len(buffer) < HEADER_BYTES:
            return

        width, height, threshold = struct.unpack_from("<HHH", buffer, 4)
        if not (0 < width <= MAX_WIDTH and 0 < height <= MAX_HEIGHT):
            del buffer[0]
            continue
        pixels = width * height
        total = HEADER_BYTES + pixels + 2
        if len(buffer) < total:
            return
        payload = bytes(buffer[HEADER_BYTES:HEADER_BYTES + pixels])
        received_crc = struct.unpack_from("<H", buffer, HEADER_BYTES + pixels)[0]
        del buffer[:total]
        if crc16_xmodem(payload) != received_crc:
            continue
        STATE.update_frame(payload, width, height, threshold)


HTML = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><title>ESP32-S3 摄像头灰度图</title>
<style>
body{margin:0;background:#202124;color:#eee;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;text-align:center}
h2{font-size:21px;font-weight:500;margin:18px 0 6px}#status{color:#b9c1cc;margin:0 0 14px}
.views{display:flex;justify-content:center;gap:18px;flex-wrap:wrap}.view{display:flex;flex-direction:column;gap:8px}
canvas{width:min(720px,46vw);height:auto;max-width:94vw;max-height:72vh;background:#111;image-rendering:pixelated;border:1px solid #4a4f57}
button{margin:16px 5px;padding:8px 16px;font-size:15px}.note{color:#9aa4b2;font-size:13px;margin:0 0 16px}
</style></head><body>
<h2>ESP32-S3 摄像头实时灰度图</h2><div id="status">等待图像帧…</div>
<div class="views"><div class="view"><span>灰度图</span><canvas id="gray"></canvas></div>
<div class="view"><span>二值预览</span><canvas id="binary"></canvas></div></div>
<div><button onclick="cmd('n')">阈值 −5</button><button onclick="cmd('m')">阈值 +5</button></div>
<p class="note">二值预览仅用于检查黑白对比；本工程不控制电机。</p>
<script>
const grayCanvas=document.getElementById('gray'), binCanvas=document.getElementById('binary');
const grayCtx=grayCanvas.getContext('2d'), binCtx=binCanvas.getContext('2d');
const status=document.getElementById('status'); let lastFrames=0;
function drawGuides(ctx,w,h,threshold,binary){
  ctx.save();ctx.lineWidth=Math.max(0.5,w/320);
  ctx.setLineDash([Math.max(1,w/40),Math.max(1,w/40)]);ctx.strokeStyle='#00e5ff';
  for(const ratio of [25/80,35/80,40/80,45/80,50/80]){
    const x=ratio*w;ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,h);ctx.stroke();
  }
  ctx.setLineDash([]);ctx.strokeStyle='#ffe600';ctx.beginPath();
  ctx.moveTo(w/2,h*14/60);ctx.lineTo(0,h-1);ctx.lineTo(w-1,h-1);ctx.closePath();ctx.stroke();
  if(binary){ctx.fillStyle='#ff3030';ctx.font=`${Math.max(5,h/10)}px monospace`;ctx.fillText(`TH=${threshold}`,2,Math.max(6,h/10));}
  ctx.restore();
}
function draw(canvas,ctx,pixels,w,h,threshold,binary){
  if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h;}
  const image=ctx.createImageData(w,h); for(let i=0;i<pixels.length;i++){
    const v=binary?(pixels[i]<threshold?0:255):pixels[i]; const j=i*4;
    image.data[j]=v;image.data[j+1]=v;image.data[j+2]=v;image.data[j+3]=255;
  } ctx.putImageData(image,0,0);drawGuides(ctx,w,h,threshold,binary);
}
async function update(){try{
  const s=await fetch('/status',{cache:'no-store'});const info=await s.json();status.textContent=info.status;
  if(!info.ready||info.frames===lastFrames)return;lastFrames=info.frames;
  const r=await fetch('/frame',{cache:'no-store'});if(!r.ok)return;
  const w=Number(r.headers.get('X-Width')),h=Number(r.headers.get('X-Height')),t=Number(r.headers.get('X-Threshold'));
  const pixels=new Uint8Array(await r.arrayBuffer());draw(grayCanvas,grayCtx,pixels,w,h,t,false);draw(binCanvas,binCtx,pixels,w,h,t,true);
}catch(e){status.textContent='查看器连接异常：'+e;}}
async function cmd(c){await fetch('/command/'+c,{cache:'no-store'});}
setInterval(update,50);update();
</script></body></html>"""


class ViewerHandler(BaseHTTPRequestHandler):
    link: SerialLink

    def log_message(self, _format: str, *_args: object) -> None:
        pass

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        if self.path == "/" or self.path.startswith("/index.html"):
            body = HTML.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path.startswith("/status"):
            gray, width, height, threshold, status = STATE.snapshot()
            body = json.dumps({"ready": gray is not None, "width": width, "height": height,
                               "threshold": threshold, "frames": STATE.frames, "status": status},
                              ensure_ascii=False).encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path.startswith("/frame"):
            gray, width, height, threshold, _status = STATE.snapshot()
            if gray is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "尚未收到图像帧")
                return
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Width", str(width))
            self.send_header("X-Height", str(height))
            self.send_header("X-Threshold", str(threshold))
            self.send_header("Content-Length", str(len(gray)))
            self.end_headers()
            self.wfile.write(gray)
            return
        if self.path.startswith("/command/") and len(self.path) == len("/command/") + 1:
            character = self.path[-1]
            if character in "mnvxMN VX".replace(" ", ""):
                self.link.send(character.encode("ascii"))
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return
        self.send_error(HTTPStatus.NOT_FOUND)


def main() -> None:
    parser = argparse.ArgumentParser(description="在浏览器查看 ESP32-S3 摄像头灰度画面")
    parser.add_argument("port", nargs="?", default="/dev/cu.usbserial-0001", help="macOS 串口路径")
    parser.add_argument("--baud", type=int, default=921600, help="串口波特率")
    parser.add_argument("--http-port", type=int, default=8765, help="本地浏览器端口")
    parser.add_argument("--no-browser", action="store_true", help="不自动打开浏览器")
    args = parser.parse_args()

    link = SerialLink(args.port, args.baud)
    ViewerHandler.link = link
    threading.Thread(target=link.run, daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), ViewerHandler)
    url = f"http://127.0.0.1:{args.http_port}/"
    print(f"浏览器地址：{url}")
    print("按 Ctrl+C 停止。运行查看器时不要打开 VS Code Monitor Device。")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        server.server_close()


if __name__ == "__main__":
    main()
