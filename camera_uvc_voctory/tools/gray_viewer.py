#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32-S3 摄像头 80x60 灰度图/二值图实时查看器

配合 camera_uvc_test 固件使用：
  1. 固件串口波特率是 921600（VS Code monitor 也要选 921600）。
  2. 运行本脚本，会自动向串口发送 'v' 开启灰度流（每 2 秒重发一次，
     防止打开串口瞬间开发板被 DTR 复位导致第一条命令丢失）；
     关闭窗口或按 Esc 时自动发送 'x' 关闭。
  3. 打开串口后立即释放 DTR/RTS（避免触发开发板自动复位）。
  4. 窗口并排显示：左侧原灰度图、右侧阈值二值图（灰阶 < 阈值 = 黑），
     均按 SCALE 倍放大。
  5. 窗口内按 + / -（或 m / n）可实时调节固件阈值（±5），二值图即时变化，
     不用重新烧录。

用法：
  python3 gray_viewer.py /dev/cu.usbserial-0001 [波特率] [放大倍数]
  示例：
  python3 gray_viewer.py /dev/cu.usbserial-0001 921600 9

依赖：
  pip install pyserial
"""

import struct
import sys
import threading
import time

import serial
import tkinter as tk

MAGIC = b"GRAY"
W = 80
H = 60
SCALE = 9                     # 放大倍数（画幅 = 80*SCALE x 60*SCALE）
THRESHOLD = 110               # 与固件默认阈值一致；每帧图像帧头会携带最新值
FRAME_BYTES = W * H
FRAME_TOTAL = 4 + 4 + 2 + 2 + FRAME_BYTES + 2  # magic + w/h + th + near/far + data + crc16

# 关注区域（80x60 坐标）：底边 80、高 45 的等腰三角形，顶点 (40,14)
ROI_APEX_X = 40
ROI_APEX_Y = 14


def crc16_xmodem(data):
    crc = 0
    for byte in data:
        crc ^= (byte << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class GrayViewer:
    def __init__(self, port_name, baud):
        self.port_name = port_name
        self.baud = baud
        self.ser = serial.Serial(port_name, baud, timeout=0.1)
        # 打开后立刻释放 DTR/RTS：避免触发 ESP32 开发板自动复位
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except Exception:
            pass
        self.running = True
        self.frames = 0
        self.fps_frames = 0
        self.t0 = time.time()
        self.bytes_rx = 0
        self.last_frame_time = time.time()
        self.near_bits = 0
        self.far_bits = 0

        self.root = tk.Tk()
        self.root.title("ESP32-S3 Camera 80x60 (Gray | Binary)")

        # 原始 80x60 帧缓冲
        self.frame_gray = tk.PhotoImage(width=W, height=H)
        self.frame_bin = tk.PhotoImage(width=W, height=H)
        # 放大后的显示图（保持引用防止被回收）
        self.gray_zoom = self.frame_gray.zoom(SCALE, SCALE)
        self.bin_zoom = self.frame_bin.zoom(SCALE, SCALE)

        top = tk.Frame(self.root)
        top.pack()
        self.canvas_gray = tk.Canvas(top, width=W * SCALE, height=H * SCALE,
                                     highlightthickness=0)
        self.canvas_bin = tk.Canvas(top, width=W * SCALE, height=H * SCALE,
                                    highlightthickness=0)
        self.canvas_gray.pack(side="left", padx=4)
        self.canvas_bin.pack(side="left", padx=4)
        self.item_gray = self.canvas_gray.create_image(0, 0, anchor="nw",
                                                       image=self.gray_zoom)
        self.item_bin = self.canvas_bin.create_image(0, 0, anchor="nw",
                                                     image=self.bin_zoom)
        # 二值图左上角叠印当前阈值
        self.th_overlay = self.canvas_bin.create_text(
            8, 6, anchor="nw", text="TH=%d" % THRESHOLD,
            fill="red", font=("Menlo", 14))
        # 关注三角形（真线大概率在此区域内）
        tri = [(ROI_APEX_X * SCALE, ROI_APEX_Y * SCALE),
               (0, (H - 1) * SCALE),
               ((W - 1) * SCALE, (H - 1) * SCALE)]
        self.tri_gray = self.canvas_gray.create_polygon(tri, outline="yellow",
                                                        fill="")
        self.tri_bin = self.canvas_bin.create_polygon(tri, outline="yellow",
                                                      fill="")
        # 4 路虚拟红外的分区边界（80x60 流坐标 x=26/33/40/47/54，
        # 即检测图 bit3[52,66) bit2[66,80) bit1[80,94) bit0[94,108) 的边界）
        for zx in (26, 33, 40, 47, 54):
            x = zx * SCALE
            self.canvas_gray.create_line(x, 0, x, H * SCALE,
                                         fill="cyan", dash=(4, 4))
            self.canvas_bin.create_line(x, 0, x, H * SCALE,
                                        fill="cyan", dash=(4, 4))

        lab = tk.Frame(self.root)
        lab.pack()
        tk.Label(lab, text="灰度图", font=("Menlo", 12)).pack(side="left", padx=80)
        self.bin_label = tk.Label(lab, text="二值图 (灰阶<%d=黑)" % THRESHOLD,
                                  font=("Menlo", 12))
        self.bin_label.pack(side="left", padx=80)

        self.th_label = tk.Label(self.root, text="当前阈值 TH = %d" % THRESHOLD,
                                 font=("Menlo", 18), fg="red")
        self.th_label.pack()

        zones = tk.Frame(self.root)
        zones.pack()
        tk.Label(zones, text="四路判定(near):", font=("Menlo", 12)).pack(side="left")
        self.zone_labels = []
        for _ in range(4):
            lb = tk.Label(zones, text="W", width=3, font=("Menlo", 16),
                          relief="solid", bd=1)
            lb.pack(side="left", padx=3)
            self.zone_labels.append(lb)
        self.far_label = tk.Label(self.root, text="far=0b0000",
                                  font=("Menlo", 12))
        self.far_label.pack()

        btns = tk.Frame(self.root)
        btns.pack(pady=4)
        tk.Button(btns, text="阈值 -5 (n)",
                  command=lambda: self.send_cmd(b"n"),
                  font=("Menlo", 12)).pack(side="left", padx=8)
        tk.Button(btns, text="阈值 +5 (m)",
                  command=lambda: self.send_cmd(b"m"),
                  font=("Menlo", 12)).pack(side="left", padx=8)

        self.label = tk.Label(self.root, text="waiting for frames ...",
                              font=("Menlo", 12), justify="left")
        self.label.pack()

        self.root.bind("<Escape>", lambda e: self.stop())
        self.root.bind("+", lambda e: self.send_cmd(b"m"))
        self.root.bind("-", lambda e: self.send_cmd(b"n"))
        self.root.bind("m", lambda e: self.send_cmd(b"m"))
        self.root.bind("n", lambda e: self.send_cmd(b"n"))
        self.root.protocol("WM_DELETE_WINDOW", self.stop)
        self.root.after(200, self.poll_status)
        self.root.after(2000, self.resend_v)

    def show(self, gray):
        gray_rows = []
        bin_rows = []
        for y in range(H):
            gs = gray[y * W:(y + 1) * W]
            gray_rows.append("{" + " ".join(
                "#%02x%02x%02x" % (g, g, g) for g in gs) + "}")
            bin_rows.append("{" + " ".join(
                "#000000" if g < THRESHOLD else "#ffffff" for g in gs) + "}")
        self.frame_gray.put(" ".join(gray_rows), to=(0, 0, W, H))
        self.frame_bin.put(" ".join(bin_rows), to=(0, 0, W, H))

        self.gray_zoom = self.frame_gray.zoom(SCALE, SCALE)
        self.bin_zoom = self.frame_bin.zoom(SCALE, SCALE)
        self.canvas_gray.itemconfig(self.item_gray, image=self.gray_zoom)
        self.canvas_bin.itemconfig(self.item_bin, image=self.bin_zoom)

        self.frames += 1
        self.fps_frames += 1
        self.last_frame_time = time.time()

    def update_zones(self):
        b = self.near_bits
        for i in range(4):
            bit = (b >> (3 - i)) & 1
            lb = self.zone_labels[i]
            lb.config(text="B" if bit else "W",
                      bg="#000000" if bit else "#ffffff",
                      fg="#ffffff" if bit else "#000000")
        self.far_label.config(
            text="near=0b%d%d%d%d  far=0b%d%d%d%d" % (
                (b >> 3) & 1, (b >> 2) & 1, (b >> 1) & 1, b & 1,
                (self.far_bits >> 3) & 1, (self.far_bits >> 2) & 1,
                (self.far_bits >> 1) & 1, self.far_bits & 1))

    def resend_v(self):
        if not self.running:
            return
        try:
            self.ser.write(b"v")
        except Exception:
            pass
        self.root.after(2000, self.resend_v)

    def send_cmd(self, cmd):
        if self.running:
            try:
                self.ser.write(cmd)
            except Exception:
                pass

    def poll_status(self):
        if not self.running:
            return
        now = time.time()
        fps = 0.0
        if now - self.t0 >= 1.0:
            fps = self.fps_frames / (now - self.t0)
            self.fps_frames = 0
            self.t0 = now
        text = "bytes: %d   frames: %d   fps: %.1f   TH=%d   (+/- 调阈值, Esc 退出)" % (
            self.bytes_rx, self.frames, fps, THRESHOLD)
        self.update_zones()
        self.th_label.config(text="当前阈值 TH = %d" % THRESHOLD)
        self.bin_label.config(text="二值图 (灰阶<%d=黑)" % THRESHOLD)
        self.canvas_bin.itemconfig(self.th_overlay, text="TH=%d" % THRESHOLD)
        if self.frames == 0 and now - self.last_frame_time > 5:
            text += ("\n未收到图像帧，请检查：\n"
                     " 1) 串口监视器/screen 是否已关闭（同一串口只能一个程序占用）\n"
                     " 2) 固件是否最新（串口菜单应显示 v=gray stream ON）\n"
                     " 3) 摄像头是否出图（日志有 LINE ... 行）")
        self.label.config(text=text)
        self.root.after(200, self.poll_status)

    def reader(self):
        buf = b""
        try:
            while self.running:
                chunk = self.ser.read(4096)
                if not chunk:
                    continue
                self.bytes_rx += len(chunk)
                buf += chunk
                while True:
                    idx = buf.find(MAGIC)
                    if idx < 0:
                        buf = buf[-8:]
                        break
                    if len(buf) - idx < FRAME_TOTAL:
                        buf = buf[idx:]
                        break
                    w, h = struct.unpack("<HH", buf[idx + 4:idx + 8])
                    th = struct.unpack("<H", buf[idx + 8:idx + 10])[0]
                    self.near_bits = buf[idx + 10]
                    self.far_bits = buf[idx + 11]
                    payload = buf[idx + 12:idx + 12 + FRAME_BYTES]
                    crc = struct.unpack("<H", buf[idx + 12 + FRAME_BYTES:
                                                   idx + FRAME_TOTAL])[0]
                    buf = buf[idx + FRAME_TOTAL:]
                    if w == W and h == H and crc16_xmodem(payload) == crc:
                        global THRESHOLD
                        if THRESHOLD != th:
                            THRESHOLD = th
                            print("TH from frame:", th, file=sys.stderr)
                        self.root.after(0, lambda p=payload: self.show(p))
        except Exception as exc:
            if self.running:
                print("reader error:", exc)

    def stop(self):
        if not self.running:
            return
        self.running = False
        try:
            self.ser.write(b"x")
            self.ser.flush()
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        self.root.destroy()

    def run(self):
        self.ser.write(b"v")
        threading.Thread(target=self.reader, daemon=True).start()
        self.root.mainloop()


def main():
    if len(sys.argv) < 2:
        print("用法: python3 gray_viewer.py <串口> [波特率] [放大倍数]")
        print("示例: python3 gray_viewer.py /dev/cu.usbserial-0001 921600 9")
        sys.exit(1)
    global SCALE
    port_name = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 921600
    if len(sys.argv) > 3:
        SCALE = int(sys.argv[3])
    try:
        viewer = GrayViewer(port_name, baud)
    except serial.SerialException as exc:
        print("打开串口失败（可能被其他程序占用，例如 VS Code 监视器/screen）:")
        print(" ", exc)
        sys.exit(1)
    viewer.run()


if __name__ == "__main__":
    main()
