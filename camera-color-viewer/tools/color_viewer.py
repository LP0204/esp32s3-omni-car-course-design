#!/usr/bin/env python3
"""Show 80x60 RGB565 frames from the ESP32-S3 camera in a local browser."""

from __future__ import annotations

import argparse
import json
import struct
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

try:
    import serial
except ImportError as exc:
    raise SystemExit("缺少 pyserial。请运行：python3 -m pip install -r tools/requirements.txt") from exc


MAGIC = b"RGB5"
HEADER_BYTES = 8
MAX_WIDTH = 160
MAX_HEIGHT = 120


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
        self.rgb565: Optional[bytes] = None
        self.width, self.height = 80, 60
        self.frames = 0
        self.started = time.monotonic()
        self.status = "等待 ESP32 彩色图像帧…"

    def update(self, frame: bytes, width: int, height: int) -> None:
        with self.lock:
            self.rgb565, self.width, self.height = frame, width, height
            self.frames += 1
            elapsed = max(0.001, time.monotonic() - self.started)
            self.status = f"{width}×{height} RGB 彩色 | {self.frames / elapsed:.1f} FPS"

    def set_status(self, status: str) -> None:
        with self.lock:
            self.status = status

    def snapshot(self) -> tuple[Optional[bytes], int, int, int, str]:
        with self.lock:
            return self.rgb565, self.width, self.height, self.frames, self.status


STATE = ViewerState()


class SerialLink:
    def __init__(self, port: str, baud: int) -> None:
        self.port, self.baud = port, baud
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
                try:
                    port.dtr = False
                    port.rts = False
                except (serial.SerialException, OSError):
                    pass
                STATE.set_status("串口已连接，等待摄像头彩色帧…")
                buffer = bytearray()
                last_start = 0.0
                while not self.stop_event.is_set():
                    now = time.monotonic()
                    if now - last_start >= 1.0:
                        self.send(b"v")
                        last_start = now
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
        width, height = struct.unpack_from("<HH", buffer, 4)
        if not (0 < width <= MAX_WIDTH and 0 < height <= MAX_HEIGHT):
            del buffer[0]
            continue
        payload_size = width * height * 2
        total = HEADER_BYTES + payload_size + 2
        if len(buffer) < total:
            return
        payload = bytes(buffer[HEADER_BYTES:HEADER_BYTES + payload_size])
        received_crc = struct.unpack_from("<H", buffer, HEADER_BYTES + payload_size)[0]
        del buffer[:total]
        if crc16_xmodem(payload) == received_crc:
            STATE.update(payload, width, height)


HTML = """<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<title>ESP32-S3 摄像头彩色分类</title><style>
body{margin:0;background:#202124;color:#eee;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;text-align:center}
h2{font-size:21px;font-weight:500;margin:18px 0 6px}#status{color:#b9c1cc;margin:0 0 14px}
.views{display:flex;justify-content:center;gap:18px;flex-wrap:wrap}.view{display:flex;flex-direction:column;gap:8px}
canvas{width:min(680px,46vw);height:auto;max-width:94vw;max-height:70vh;background:#111;border:1px solid #4a4f57;image-rendering:pixelated}
.controls{display:flex;justify-content:center;gap:16px;flex-wrap:wrap;margin:0 auto 14px}.controls label{display:flex;align-items:center;gap:7px;color:#d7dce3;font-size:14px}.controls input{width:130px}.controls output{min-width:25px;text-align:left;color:#8dd8ff}
p{color:#9aa4b2;font-size:13px}</style></head><body><h2>ESP32-S3 摄像头实时彩色分类</h2>
<div id="status">等待图像帧…</div><div class="controls"><label>黑色亮度 <input id="gray" type="range" min="0" max="160" value="55"><output id="grayValue">55</output></label><label>红度 <input id="red" type="range" min="0" max="160" value="50"><output id="redValue">50</output></label><label>绿度 <input id="green" type="range" min="0" max="160" value="42"><output id="greenValue">42</output></label></div><div class="views"><div class="view"><span>原始彩色图</span><canvas id="image"></canvas></div><div class="view"><span>红 / 绿 / 黑 / 白 分类图</span><canvas id="classify"></canvas></div></div>
<p>黑色优先；红、绿按颜色优势判定；其余像素默认为白色。本工程不控制电机。</p>
<script>const canvas=document.getElementById('image'),ctx=canvas.getContext('2d'),classCanvas=document.getElementById('classify'),classCtx=classCanvas.getContext('2d'),status=document.getElementById('status'),gray=document.getElementById('gray'),red=document.getElementById('red'),green=document.getElementById('green');let last=-1,lastPixels=null,lastWidth=0,lastHeight=0;
function rgb565(p,i){const v=p[i*2]|(p[i*2+1]<<8);return [((v>>11)&31)*255/31,((v>>5)&63)*255/63,(v&31)*255/31]}
function classify(r,g,b){const brightness=(r+g+b)/3,redness=r-(g+b)/2,greenness=g-(r+b)/2;if(brightness<Number(gray.value))return [0,0,0];if(redness>Number(red.value)&&r>90)return [230,45,45];if(greenness>Number(green.value)&&g>75)return [35,190,70];return [255,255,255]}
function draw(p,w,h){lastPixels=p;lastWidth=w;lastHeight=h;if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h;classCanvas.width=w;classCanvas.height=h}const image=ctx.createImageData(w,h),classes=classCtx.createImageData(w,h);for(let i=0;i<w*h;i++){const [r,g,b]=rgb565(p,i),j=i*4,c=classify(r,g,b);image.data[j]=r;image.data[j+1]=g;image.data[j+2]=b;image.data[j+3]=255;classes.data[j]=c[0];classes.data[j+1]=c[1];classes.data[j+2]=c[2];classes.data[j+3]=255}ctx.putImageData(image,0,0);classCtx.putImageData(classes,0,0)}
function updateControls(){document.getElementById('grayValue').textContent=gray.value;document.getElementById('redValue').textContent=red.value;document.getElementById('greenValue').textContent=green.value;if(lastPixels)draw(lastPixels,lastWidth,lastHeight)}[gray,red,green].forEach(control=>control.addEventListener('input',updateControls));
async function update(){try{const s=await fetch('/status',{cache:'no-store'}),info=await s.json();status.textContent=info.status;if(!info.ready||info.frames===last)return;last=info.frames;const r=await fetch('/frame',{cache:'no-store'});if(!r.ok)return;draw(new Uint8Array(await r.arrayBuffer()),Number(r.headers.get('X-Width')),Number(r.headers.get('X-Height')))}catch(e){status.textContent='查看器连接异常：'+e}}setInterval(update,40);update();</script></body></html>"""


class ViewerHandler(BaseHTTPRequestHandler):
    link: SerialLink

    def log_message(self, _format: str, *_args: object) -> None:
        pass

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/" or self.path.startswith("/index.html"):
            body = HTML.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path.startswith("/status"):
            frame, width, height, frames, status = STATE.snapshot()
            body = json.dumps({"ready": frame is not None, "width": width, "height": height,
                               "frames": frames, "status": status}, ensure_ascii=False).encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path.startswith("/frame"):
            frame, width, height, _frames, _status = STATE.snapshot()
            if frame is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "尚未收到图像帧")
                return
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Width", str(width))
            self.send_header("X-Height", str(height))
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)
            return
        self.send_error(HTTPStatus.NOT_FOUND)


def main() -> None:
    parser = argparse.ArgumentParser(description="在浏览器查看 ESP32-S3 摄像头 RGB 彩色画面")
    parser.add_argument("port", nargs="?", default="/dev/cu.usbserial-0001")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--http-port", type=int, default=8766)
    args = parser.parse_args()
    link = SerialLink(args.port, args.baud)
    ViewerHandler.link = link
    threading.Thread(target=link.run, daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), ViewerHandler)
    print(f"浏览器地址：http://127.0.0.1:{args.http_port}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        server.server_close()


if __name__ == "__main__":
    main()
