#!/usr/bin/env python3
"""Show the 640x480 camera frame and the ESP32 green mask."""

from __future__ import annotations

import argparse
import binascii
import json
import struct
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional
from urllib.parse import parse_qs, urlparse

try:
    import serial
except ImportError as exc:
    raise SystemExit("缺少 pyserial。请运行：python3 -m pip install -r tools/requirements.txt") from exc


JPEG_MAGIC = b"JPG4"
MAP_MAGIC = b"GMAP"
JPEG_HEADER_BYTES = 12  # magic + width + height + JPEG length
MAP_HEADER_BYTES = 8    # magic + width + height
MAX_JPEG_BYTES = 512 * 1024
MAX_WIDTH = 1920
MAX_HEIGHT = 1080
GREEN_THRESHOLD_DEFAULT = 42
GREEN_THRESHOLD_MIN = 0
GREEN_THRESHOLD_MAX = 160


def crc16_xmodem(data: bytes) -> int:
    return binascii.crc_hqx(data, 0)


class ViewerState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.jpeg: Optional[bytes] = None
        self.width, self.height = 640, 480
        self.green_map: Optional[bytes] = None
        self.map_width, self.map_height = 80, 60
        self.frames = 0
        self.received_bytes = 0
        self.bad_frames = 0
        self.last_frame_time = 0.0
        self.map_frames = 0
        self.green_pixels = 0
        self.green_threshold = GREEN_THRESHOLD_DEFAULT
        self.started = time.monotonic()
        self.status = "等待 ESP32 绿球工程图像帧…"

    def update(self, frame: bytes, width: int, height: int) -> None:
        with self.lock:
            self.jpeg, self.width, self.height = frame, width, height
            self.frames += 1
            self.last_frame_time = time.monotonic()
            elapsed = max(0.001, time.monotonic() - self.started)
            self.status = f"{width}×{height} JPEG | {self.frames / elapsed:.1f} FPS"

    def update_map(self, mask: bytes, width: int, height: int) -> None:
        with self.lock:
            self.green_map, self.map_width, self.map_height = mask, width, height
            self.map_frames += 1
            self.green_pixels = sum(1 for value in mask if value)

    def snapshot_map(self) -> tuple[Optional[bytes], int, int, int]:
        with self.lock:
            return self.green_map, self.map_width, self.map_height, self.map_frames

    def set_status(self, status: str) -> None:
        with self.lock:
            self.status = status

    def set_green_threshold(self, value: int) -> None:
        with self.lock:
            self.green_threshold = value

    def get_green_threshold(self) -> int:
        with self.lock:
            return self.green_threshold

    def snapshot(self) -> tuple[Optional[bytes], int, int, int, str]:
        with self.lock:
            status = (self.status + f" | 绿图 {self.map_frames} 帧 | 绿点 {self.green_pixels}"
                      f" | 接收 {self.received_bytes} 字节 | 校验失败 {self.bad_frames}")
            if self.last_frame_time and time.monotonic() - self.last_frame_time > 3:
                status += " | 图像已停更"
            return self.jpeg, self.width, self.height, self.frames, status


STATE = ViewerState()


class SerialLink:
    def __init__(self, port: str, baud: int) -> None:
        self.port, self.baud = port, baud
        self.lock = threading.Lock()
        self.port_handle: Optional[serial.Serial] = None
        self.stop_event = threading.Event()
        self.green_threshold = GREEN_THRESHOLD_DEFAULT

    def send(self, character: bytes) -> None:
        with self.lock:
            if self.port_handle is None:
                return
            try:
                self.port_handle.write(character)
                self.port_handle.flush()
            except (serial.SerialException, OSError) as exc:
                STATE.set_status(f"串口发送失败：{exc}")

    def set_green_threshold(self, value: int) -> int:
        value = max(GREEN_THRESHOLD_MIN, min(GREEN_THRESHOLD_MAX, int(value)))
        self.green_threshold = value
        STATE.set_green_threshold(value)
        self.send(f"g{value}\n".encode("ascii"))
        return value

    def run(self) -> None:
        try:
            with serial.Serial(self.port, self.baud, timeout=0.1, rtscts=False, dsrdtr=False) as port:
                self.port_handle = port
                try:
                    port.dtr = False
                    port.rts = False
                except (serial.SerialException, OSError):
                    pass
                STATE.set_status("串口已连接，等待摄像头 JPEG 帧…")
                # Apply the slider's current value after every reconnect.
                self.set_green_threshold(self.green_threshold)
                buffer = bytearray()
                last_start = 0.0
                while not self.stop_event.is_set():
                    now = time.monotonic()
                    if now - last_start >= 1.0:
                        self.send(b"v")
                        last_start = now
                    chunk = port.read(min(65536, max(1, port.in_waiting)))
                    if chunk:
                        with STATE.lock:
                            STATE.received_bytes += len(chunk)
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
        starts = [index for magic in (JPEG_MAGIC, MAP_MAGIC)
                  for index in [buffer.find(magic)] if index >= 0]
        start = min(starts) if starts else -1
        if start < 0:
            keep = max(len(JPEG_MAGIC), len(MAP_MAGIC)) - 1
            if len(buffer) > keep:
                del buffer[: -keep]
            return
        if start:
            del buffer[:start]
        if buffer.startswith(JPEG_MAGIC):
            header_bytes = JPEG_HEADER_BYTES
        elif buffer.startswith(MAP_MAGIC):
            header_bytes = MAP_HEADER_BYTES
        else:
            del buffer[0]
            continue
        if len(buffer) < header_bytes:
            return

        is_jpeg = buffer.startswith(JPEG_MAGIC)
        if is_jpeg:
            width, height, payload_size = struct.unpack_from("<HHI", buffer, 4)
            valid = (0 < width <= MAX_WIDTH and 0 < height <= MAX_HEIGHT and
                     4 <= payload_size <= MAX_JPEG_BYTES)
        else:
            width, height = struct.unpack_from("<HH", buffer, 4)
            payload_size = width * height
            valid = (0 < width <= 160 and 0 < height <= 120 and
                     payload_size <= 160 * 120)
        if not valid:
            del buffer[0]
            continue
        total = header_bytes + payload_size + 2
        if len(buffer) < total:
            return
        payload = bytes(buffer[header_bytes:header_bytes + payload_size])
        received_crc = struct.unpack_from("<H", buffer, header_bytes + payload_size)[0]
        if crc16_xmodem(payload) == received_crc:
            del buffer[:total]
            if is_jpeg:
                STATE.update(payload, width, height)
            else:
                STATE.update_map(payload, width, height)
        else:
            with STATE.lock:
                STATE.bad_frames += 1
            del buffer[0]


HTML = """<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<title>ESP32-S3 绿球识别 480p 预览</title><style>
body{margin:0;background:#202124;color:#eee;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;text-align:center}
h2{font-size:22px;font-weight:500;margin:18px 0 6px}#status{color:#b9c1cc;margin:0 0 14px}
.controls{display:flex;justify-content:center;align-items:center;gap:10px;margin:0 auto 14px;color:#d7dce3;font-size:15px}.controls input{width:220px}.controls output{display:inline-block;min-width:28px;color:#8dd8ff;text-align:left}
.views{display:flex;justify-content:center;gap:20px;flex-wrap:wrap}.frame{display:flex;flex-direction:column;gap:8px;max-width:96vw}.frame span{font-size:16px;color:#d7dce3}
img,canvas{display:block;width:min(640px,46vw);height:auto;background:#111;border:1px solid #4a4f57;transform:rotate(180deg)}
canvas{image-rendering:pixelated}p{color:#9aa4b2;font-size:13px}code{color:#8dd8ff}</style></head><body>
<h2>ESP32-S3 摄像头实时绿球识别</h2><div id="status">等待图像帧…</div>
<div class="controls"><label for="threshold">绿度阈值</label><input id="threshold" type="range" min="0" max="160" value="42" step="1"><output id="thresholdValue">42</output></div>
<div class="views"><div class="frame"><span>摄像头原始图像（目标 640×480，已旋转 180°）</span><img id="preview" alt="等待摄像头图像"></div>
<div class="frame"><span>单片机实际绿度二值图（80×60，已旋转 180°）<b id="mapInfo">绿点：等待</b></span><canvas id="greenMap" width="80" height="60"></canvas></div></div>
<p>判绿条件：G &gt; 75 且 G-(R+B)/2 &gt; <span id="thresholdHint">42</span>；拖动滑块实时更新单片机阈值。绿色像素显示为绿色，其余显示为白色。按 BOOT 开始或停止。</p>
<script>const preview=document.getElementById('preview'),mapCanvas=document.getElementById('greenMap'),mapCtx=mapCanvas.getContext('2d'),mapInfo=document.getElementById('mapInfo'),status=document.getElementById('status'),threshold=document.getElementById('threshold'),thresholdValue=document.getElementById('thresholdValue'),thresholdHint=document.getElementById('thresholdHint');let last=-1,lastMap=-1,url=null,thresholdTimer=null;
function drawMap(mask,w,h){if(mapCanvas.width!==w||mapCanvas.height!==h){mapCanvas.width=w;mapCanvas.height=h}let count=0;const image=mapCtx.createImageData(w,h);for(let i=0;i<w*h;i++){if(mask[i])count++;const c=mask[i]? [35,205,75]:[255,255,255],j=i*4;image.data[j]=c[0];image.data[j+1]=c[1];image.data[j+2]=c[2];image.data[j+3]=255}mapCtx.putImageData(image,0,0);mapInfo.textContent='绿点：'+count}
function applyThreshold(){const value=Number(threshold.value);thresholdValue.textContent=value;thresholdHint.textContent=value;clearTimeout(thresholdTimer);thresholdTimer=setTimeout(()=>fetch('/set-threshold?value='+value,{cache:'no-store'}).catch(()=>{}),80)}
threshold.addEventListener('input',applyThreshold);
async function update(){try{const s=await fetch('/status',{cache:'no-store'}),info=await s.json();status.textContent=info.status;if(document.activeElement!==threshold&&Number.isInteger(info.threshold)){threshold.value=info.threshold;thresholdValue.textContent=info.threshold;thresholdHint.textContent=info.threshold}if(info.ready&&info.frames!==last){last=info.frames;const r=await fetch('/frame',{cache:'no-store'});if(r.ok){const blob=await r.blob();if(url)URL.revokeObjectURL(url);url=URL.createObjectURL(blob);preview.src=url}}if(info.mapFrames!==lastMap){lastMap=info.mapFrames;const m=await fetch('/green-map',{cache:'no-store'});if(m.ok)drawMap(new Uint8Array(await m.arrayBuffer()),Number(m.headers.get('X-Width')),Number(m.headers.get('X-Height')))}}catch(e){status.textContent='查看器连接异常：'+e}}setInterval(update,100);applyThreshold();update();</script></body></html>"""


class ViewerHandler(BaseHTTPRequestHandler):
    link: SerialLink

    def log_message(self, _format: str, *_args: object) -> None:
        pass

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/" or self.path.startswith("/index.html"):
            body = HTML.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            # The page contains the latest protocol/UI definition. Do not let
            # Safari keep an older page that has no GMAP panel.
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path.startswith("/status"):
            frame, width, height, frames, status = STATE.snapshot()
            _mask, _map_width, _map_height, map_frames = STATE.snapshot_map()
            body = json.dumps({"ready": frame is not None, "width": width, "height": height,
                               "frames": frames, "mapFrames": map_frames,
                               "greenPixels": STATE.green_pixels,
                               "threshold": STATE.get_green_threshold(),
                               "status": status}, ensure_ascii=False).encode("utf-8")
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
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "尚未收到 JPEG 图像帧")
                return
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Width", str(width))
            self.send_header("X-Height", str(height))
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)
            return
        if self.path.startswith("/green-map"):
            mask, width, height, _map_frames = STATE.snapshot_map()
            if mask is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "尚未收到绿度图")
                return
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Width", str(width))
            self.send_header("X-Height", str(height))
            self.send_header("Content-Length", str(len(mask)))
            self.end_headers()
            self.wfile.write(mask)
            return
        if self.path.startswith("/set-threshold"):
            query = parse_qs(urlparse(self.path).query)
            try:
                value = int(query.get("value", [GREEN_THRESHOLD_DEFAULT])[0])
            except (TypeError, ValueError):
                self.send_error(HTTPStatus.BAD_REQUEST, "阈值必须是整数")
                return
            value = self.link.set_green_threshold(value)
            body = json.dumps({"threshold": value}).encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(HTTPStatus.NOT_FOUND)


def main() -> None:
    parser = argparse.ArgumentParser(description="在浏览器查看 ESP32-S3 640x480 JPEG 图像")
    parser.add_argument("port", nargs="?", default="/dev/cu.usbserial-0001")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--http-port", type=int, default=8767)
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
