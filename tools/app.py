"""GUI for real residual-error images over ESP32-C5 raw 802.11 frames."""
import json
import math
import os
from pathlib import Path
import queue
import secrets
import threading
import time
import struct
from concurrent.futures import ThreadPoolExecutor
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from PIL import Image, ImageTk
import serial
from serial.tools import list_ports

from build_support import ROOT, load_settings, run_build
from protocol import CHUNK, FIRMWARE, ImageRun, command_record, parse_record, prepare_image
from sender import transmit
from rssi import RssiTracker
from archive import save_round


class ImageApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Y极化 · ESP32-C5实时误码传图")
        self.geometry("1240x900")
        self.settings = load_settings()
        self.events = queue.Queue(maxsize=10000)
        self.ports = {}
        self.info = {}
        self.generation = 0
        self.connection_log = None
        self.capture_log = None
        self.building = False
        self.active = False
        self.loop_enabled = False
        self.loop_round = 0
        self.loop_after = None
        self.run = None
        self.draining = {}
        self.saver = ThreadPoolExecutor(max_workers=1, thread_name_prefix="image-save")
        self.saves = []
        self.live_rssi = RssiTracker()
        self.next_run_id = secrets.randbelow(0x7FFFFFFE) + 1
        self.session = None
        self.selected_path = None
        self.source_image = None
        self.photos = {}
        self.stop_event = threading.Event()
        self.acks = queue.Queue()
        self.sender_thread = None
        self.finish_at = None
        self.finish_reason = ""
        self.serial_errors = 0
        self.host_drops = 0
        self.serial_baseline = 0
        self.host_baseline = 0
        self.transport_broken = False
        self.board_stats = [0] * 6
        self.drop_baseline = 0
        self.final_summary = None
        self.last_render = 0
        self.first_tx_time = None
        self.last_tx_time = None
        self.last_flush = 0
        self.dirty = False
        self.rssi = RssiTracker()
        self.last_rssi_render = 0
        self.rssi_text = tk.StringVar(value="Bob RSSI：— · 尚无本轮可识别图像块")
        self.status = tk.StringVar(value="先烧录本项目两端固件，再连接并选择图片。")
        self.metrics = tk.StringVar(value="尚未传输")
        self.image_note = tk.StringVar(value="请选择JPG、PNG或其他可读取的图片")
        ttk.Label(self, text="Y通道实时误码传图", font=("Microsoft YaHei", 20)).pack(anchor="w", padx=16, pady=10)
        ttk.Label(self, text="5.200 GHz / 信道40 · 首次接收保留 · 载荷错误直接显示 · 缺失区域为深灰色 · 当前为明文").pack(anchor="w", padx=16)
        row = ttk.Frame(self); row.pack(fill="x", padx=16, pady=10)
        self.port_vars = {}; self.port_boxes = []
        for role in ("Alice", "Bob"):
            ttk.Label(row, text=role + "串口").pack(side="left")
            var = tk.StringVar(value=self.settings.get(role.lower() + "_port", ""))
            self.port_vars[role] = var
            box = ttk.Combobox(row, textvariable=var, width=9, state="readonly")
            box.pack(side="left", padx=6); self.port_boxes.append(box)
        for label, action in (("刷新", self.refresh), ("连接两端", self.connect), ("断开", self.disconnect),
                              ("连接日志", lambda: self.open_folder("connection_logs"))):
            ttk.Button(row, text=label, command=action).pack(side="left", padx=4)
        row = ttk.Frame(self); row.pack(fill="x", padx=16, pady=4)
        for role in ("Bob", "Alice"):
            ttk.Button(row, text="编译并烧录" + role, command=lambda role=role: self.flash(role)).pack(side="left", padx=4)
        ttk.Button(row, text="仅编译两端", command=lambda: self.flash(None)).pack(side="left", padx=4)
        row = ttk.Frame(self); row.pack(fill="x", padx=16, pady=10)
        ttk.Button(row, text="选择图片", command=self.choose_image).pack(side="left", padx=4)
        ttk.Label(row, text="尺寸上限").pack(side="left")
        self.limit = tk.StringVar(value="256")
        box = ttk.Combobox(row, textvariable=self.limit, values=(160, 256, 320, 640), width=6, state="readonly")
        box.pack(side="left", padx=5)
        box.bind("<<ComboboxSelected>>", lambda event: self.resize_source())
        self.interval = tk.StringVar(value="0")
        self.power = tk.StringVar(value="8")
        for label, variable in (("块间隔(ms)", self.interval), ("功率上限(dBm)", self.power)):
            ttk.Label(row, text=label).pack(side="left")
            ttk.Entry(row, textvariable=variable, width=6).pack(side="left", padx=5)
        ttk.Button(row, text="发送一次", command=self.start).pack(side="left", padx=5)
        ttk.Button(row, text="循环发送", command=self.start_loop).pack(side="left", padx=5)
        ttk.Button(row, text="停止", command=self.stop).pack(side="left", padx=5)
        ttk.Button(row, text="结果目录", command=lambda: self.open_folder("results")).pack(side="left", padx=5)
        ttk.Label(self, textvariable=self.image_note).pack(anchor="w", padx=16)
        ttk.Label(self, textvariable=self.status, wraplength=1180).pack(anchor="w", padx=16, pady=6)
        self.progress = ttk.Progressbar(self, maximum=100)
        self.progress.pack(fill="x", padx=16, pady=4)
        ttk.Label(self, textvariable=self.metrics, justify="left", wraplength=1180).pack(anchor="w", padx=16, pady=6)
        ttk.Label(self, textvariable=self.rssi_text, font=("Microsoft YaHei", 12), wraplength=1180).pack(anchor="w", padx=16, pady=3)
        row = ttk.Frame(self); row.pack(fill="both", expand=True, padx=16, pady=5)
        self.previews = {}
        for key, title in (("source", "Alice发送图像"), ("received", "Bob实际收到的像素")):
            group = ttk.LabelFrame(row, text=title)
            group.pack(side="left", fill="both", expand=True, padx=5)
            label = ttk.Label(group, text="等待图像", anchor="center")
            label.pack(fill="both", expand=True)
            self.previews[key] = label
        ttk.Label(self, text="条件BER只统计已通过头校验的已收到载荷，丢包与坏头不计入；请以空口丢包率、本端FCS失败率、头损坏率为主判据。"
                  "错误像素保留，缺失块不用原图补齐。结果另存缺失掩码、错误位置图，以及FCS失败帧的原始误码图 raw_error.png。",
                  wraplength=1180).pack(anchor="w", padx=16, pady=5)
        self.logbox = tk.Text(self, height=9, wrap="word")
        self.logbox.pack(fill="x", padx=16, pady=8)
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh()
        self.poll_after = self.after(5, self.poll)

    def log(self, text):
        self.logbox.insert("end", time.strftime("%H:%M:%S ") + text + "\n")
        if int(self.logbox.index("end-1c").split(".")[0]) > 1200:
            self.logbox.delete("1.0", "300.0")
        self.logbox.see("end")

    def show_image(self, key, image):
        display = image.copy()
        display.thumbnail((560, 350), Image.Resampling.NEAREST)
        # Integer nearest-neighbor enlargement keeps small errors visible.
        scale = max(1, min(560 // display.width, 350 // display.height))
        if scale > 1:
            display = display.resize((display.width * scale, display.height * scale), Image.Resampling.NEAREST)
        self.photos[key] = ImageTk.PhotoImage(display)
        self.previews[key].configure(image=self.photos[key], text="")

    def choose_image(self):
        if self.active:
            return
        path = filedialog.askopenfilename(filetypes=[("图片", "*.png *.jpg *.jpeg *.bmp *.webp *.ppm"), ("所有文件", "*.*")])
        if path:
            self.selected_path = Path(path)
            self.resize_source()

    def resize_source(self):
        if self.active or not self.selected_path:
            return
        try:
            image, pixels = prepare_image(self.selected_path, int(self.limit.get()))
            self.source_image = image
            self.show_image("source", image)
            blocks = (len(pixels) + CHUNK - 1) // CHUNK
            self.image_note.set(f"{self.selected_path.name} · RGB {image.width}×{image.height} · {len(pixels):,}字节 · {blocks}块")
        except Exception as exc:
            self.source_image = None
            messagebox.showerror("图片读取失败", str(exc))

    def refresh(self):
        names = [port.device for port in list_ports.comports()]
        for box in self.port_boxes:
            box.configure(values=names)

    def connect(self):
        if self.active or self.building:
            return
        self.disconnect()
        names = {role: var.get() for role, var in self.port_vars.items()}
        if not all(names.values()) or names["Alice"] == names["Bob"]:
            messagebox.showerror("串口", "请选择两个不同的实际串口。")
            return
        try:
            folder = ROOT / "connection_logs"; folder.mkdir(exist_ok=True)
            path = folder / (time.strftime("%Y%m%d_%H%M%S_") + str(time.time_ns()) + ".jsonl")
            self.connection_log = path.open("w", encoding="utf-8")
            self.log("连接日志 " + str(path))
            for role, name in names.items():
                port = serial.Serial(port=None, baudrate=921600, timeout=0.2, write_timeout=2)
                port.dtr = False; port.rts = False; port.port = name; port.open()
                self.ports[role] = port
                threading.Thread(target=self.reader, args=(role, port, self.generation), daemon=True).start()
            generation = self.generation
            self.after(1000, lambda: self.query_info(generation))
            self.status.set("正在查询固件身份；如无回应，请复位两块板。")
        except Exception as exc:
            self.disconnect()
            messagebox.showerror("连接失败", str(exc))

    def query_info(self, generation):
        if generation != self.generation or self.active:
            return
        for role, port in self.ports.items():
            try:
                port.write(command_record("INFO"))
            except Exception as exc:
                self.log(role + " " + str(exc))

    def reader(self, role, port, generation):
        pending = bytearray()
        try:
            while generation == self.generation and port.is_open:
                pending.extend(port.read(max(1, min(port.in_waiting, 4096))))
                while b"\n" in pending:
                    line, _, pending = pending.partition(b"\n")
                    try:
                        self.events.put_nowait((generation, role, bytes(line).rstrip(b"\r").decode("ascii", errors="replace")))
                    except queue.Full:
                        self.host_drops += 1
                if len(pending) > 8192:
                    raise ValueError("串口记录超长或缺少换行")
        except Exception as exc:
            self.events.put((generation, "PORT_ERROR", role + " " + str(exc)))

    def disconnect(self):
        self.cancel_loop()
        self.stop_event.set()
        if self.active:
            self.transport_broken = True
            self.finish("连接断开，保存部分结果")
        for run_id in list(self.draining):
            self.draining[run_id]['result']['transport_broken'] = True
            self.close_draining(run_id)
        self.generation += 1
        for port in self.ports.values():
            try:
                port.close()
            except Exception:
                pass
        self.ports.clear(); self.info.clear()
        self.rssi = RssiTracker()
        self.live_rssi = RssiTracker()
        self.rssi_text.set("Bob RSSI：— · 已断开")
        if self.connection_log:
            self.connection_log.close(); self.connection_log = None
        self.status.set("已断开")

    def flash(self, role):
        if self.active or self.building:
            return
        port = self.port_vars[role].get() if role else None
        if role and not port:
            messagebox.showerror("串口", "先选择对应开发板串口。")
            return
        self.disconnect(); self.building = True
        self.status.set("正在编译" + ("并烧录" + role if role else "两端"))
        def worker():
            try:
                for target in ((role,) if role else ("Bob", "Alice")):
                    run_build(self.settings, target, port, log=lambda text: self.events.put((-1, "BUILD", text)))
                self.events.put((-1, "BUILD_DONE", "完成，请连接两端。"))
            except Exception as exc:
                self.events.put((-1, "BUILD_DONE", "构建或烧录失败：" + str(exc)))
        threading.Thread(target=worker, daemon=True).start()

    def cancel_loop(self):
        self.loop_enabled = False
        if self.loop_after is not None:
            self.after_cancel(self.loop_after)
            self.loop_after = None

    def start_loop(self):
        if self.active or self.building or self.loop_enabled:
            return
        self.loop_enabled = True
        self.loop_round = 0
        self.start(loop_continuation=True)

    def next_loop_round(self):
        self.loop_after = None
        if not self.loop_enabled:
            return
        if len(self.draining) >= 32 or len(self.saves) >= 8:
            self.status.set("后台收尾或保存积压，暂缓下一轮以限制内存；正在继续接收")
            self.loop_after = self.after(50, self.next_loop_round)
            return
        if self.sender_thread and self.sender_thread.is_alive():
            self.loop_after = self.after(50, self.next_loop_round)
            return
        self.start(loop_continuation=True)

    def start(self, loop_continuation=False):
        if self.active or self.building or (self.sender_thread and self.sender_thread.is_alive()):
            return
        if not loop_continuation:
            self.cancel_loop()
        try:
            for role in ("Alice", "Bob"):
                info = self.info.get(role, [])
                if len(info) < 5 or info[0] != role or info[1] != "40" or info[-1] != FIRMWARE:
                    raise ValueError("请连接并确认两端均运行raw_image_v1固件、角色正确、信道40。")
            if self.source_image is None:
                raise ValueError("请先选择图片。")
            if not loop_continuation and self.selected_path and self.selected_path.exists():
                self.resize_source()
                if self.source_image is None:
                    raise ValueError("图片读取失败，请重新选择。")
            interval = int(self.interval.get()); power = float(self.power.get())
            if not 0 <= interval <= 2000 or not math.isfinite(power) or not 2 <= power <= 10:
                raise ValueError("块间隔0～2000 ms（0为四块流水发送），功率上限2～10 dBm。")
            run_id = self.next_run_id
            self.next_run_id = self.next_run_id % 0xFFFFFFFF + 1
            if self.loop_enabled:
                self.loop_round += 1
            image = self.source_image.copy()
            self.run = ImageRun(run_id, self.info["Alice"][2], image)
            self.rssi = RssiTracker()
            if not loop_continuation or self.loop_round == 1:
                self.live_rssi = RssiTracker()
            self.rssi_text.set(self.live_rssi.label(time.monotonic(), True))
            self.session = ROOT / "results" / (time.strftime("%Y%m%d_%H%M%S_") + str(run_id))
            self.session.mkdir(parents=True)
            (self.session / "settings.json").write_text(json.dumps(dict(run=run_id, image=str(self.selected_path),
                size=image.size, interval_ms=interval, requested_power_dbm=power, devices=self.info,
                loop_mode=self.loop_enabled, loop_round=self.loop_round if self.loop_enabled else None), indent=2), encoding="utf-8")
            self.capture_log = (self.session / "records.jsonl").open("w", encoding="utf-8")
            self.acks = queue.Queue(); self.stop_event = threading.Event()
            self.serial_baseline = self.serial_errors
            self.host_baseline = self.host_drops
            self.transport_broken = False
            self.drop_baseline = self.board_stats[3]
            self.final_summary = None; self.finish_at = None; self.finish_reason = ""
            self.first_tx_time = self.last_tx_time = None
            self.active = True; self.dirty = False; self.progress["value"] = 0
            self.previews["received"].configure(image="", text="等待本轮无线数据；尚未取得有效图像头")
            self.photos.pop("received", None)
            self.status.set("正在准备两端，随后逐块发送；无遮挡时先确认Bob画面出现。")
            generation = self.generation
            self.sender_thread = threading.Thread(target=transmit, args=(run_id, image.size, image.tobytes(),
                self.info["Alice"][2], interval, round(power*4), dict(self.ports), self.acks, self.stop_event,
                lambda complete, reason: self.events.put((generation, "SENDER_DONE", (run_id, complete, reason))),
                not loop_continuation or self.loop_round == 1), daemon=True)
            self.sender_thread.start()
            if self.loop_enabled:
                self.log(f"循环发送：第{self.loop_round}轮，会话{run_id}")
        except Exception as exc:
            self.cancel_loop()
            if self.active:
                self.finish("启动失败")
            elif self.capture_log:
                self.capture_log.close(); self.capture_log = None
            messagebox.showerror("无法开始", str(exc))

    def stop(self):
        if self.transport_broken:
            for context in self.draining.values():
                context['result']['transport_broken'] = True
        was_loop = self.loop_enabled
        self.cancel_loop()
        if self.active:
            self.stop_event.set()
            self.status.set("正在停止，等待剩余串口数据后保存部分图像。")
        elif was_loop:
            self.status.set("循环已停止")

    def handle(self, role, line):
        record = json.dumps(dict(time=time.time(), role=role, line=line), ensure_ascii=False) + "\n"
        if self.connection_log:
            self.connection_log.write(record)
        # Route by session hint, then let ImageReceiver validate CRC/MAC/header.
        # A forged or corrupt hint can never bypass receiver validation.
        target = None
        if role == "Bob" and line.startswith("!RX "):
            try:
                fields_hint = parse_record(line)
                # Field 9 is the hex payload in both the old 10-field record
                # and the new record with trailing PHY metadata. Using [-1]
                # would silently break multi-round routing with the new format.
                if len(fields_hint) >= 10:
                    raw = bytes.fromhex(fields_hint[9])
                    if len(raw) >= 36:
                        target = self.draining.get(struct.unpack_from('<I', raw, 32)[0])
            except (ValueError, IndexError):
                pass
        logfile = target['log'] if target else self.capture_log
        if logfile:
            logfile.write(record)
        try:
            fields = parse_record(line)
        except (ValueError, UnicodeError):
            self.serial_errors += 1
            self.log(role + "串口记录校验失败，排除本条记录")
            return
        if not fields:
            if line:
                self.log(role + " " + line)
            if any(mark in line for mark in ("rst:", "ESP-ROM:", "Guru Meditation", "ESP_ERROR_CHECK failed", "abort() was called")):
                self.info.pop(role, None)
                self.status.set(role + "重启或启动异常，请查看连接日志")
                if self.active or self.loop_enabled or self.draining:
                    self.transport_broken = True; self.stop()
            return
        tag = fields[0]
        if target and tag == "RX":
            item = target['run'].receive(fields)
            if item:
                target['rssi'].add(item['rssi'], time.monotonic())
                self.live_rssi.add(item['rssi'], time.monotonic())
            return
        if tag == "INFO":
            self.info[role] = fields[1:]
            self.log(role + " " + " ".join(fields))
            if self.active or self.loop_enabled or self.draining:
                self.transport_broken = True; self.stop()
            elif all(self.info.get(name, []) and self.info[name][-1] == FIRMWARE for name in ("Alice", "Bob")):
                self.status.set("两端固件已就绪，选择图片后点击发送一次。")
        elif tag == "BOOT":
            self.log(role + "初始化 " + " ".join(fields[1:]))
        elif tag == "STATS" and role == "Bob":
            values = list(map(int, fields[1:]))
            # Old firmware reports 5 counters, new firmware 6 (adds the count
            # of filtered foreign frames). Keep index 3 = dropped in both.
            self.board_stats = values + [0] * (6 - len(values))
        elif tag == "ERROR":
            self.log(role + " " + " ".join(fields))
            self.status.set(role + " " + " ".join(fields))
            if "BOOT_FAILED" in fields:
                self.info.pop(role, None)
            if self.active:
                self.transport_broken = True
                self.acks.put((role, fields))
                self.stop()
            elif self.loop_enabled or self.draining:
                self.transport_broken = True
                self.stop()
        elif self.active:
            if tag == "START" and role == "Alice" and int(fields[1]) == self.run.run:
                self.run.started = True
                self.status.set("传输中。Bob按实际收到的像素刷新，深灰色表示尚未收到。")
                self.log(f"发送开始，实际功率上限报告{int(fields[4])/4:g} dBm")
            elif tag == "TX" and role == "Alice" and int(fields[1]) == self.run.run:
                sequence, error = map(int, fields[2:])
                if not 0 <= sequence < self.run.count or sequence in self.run.attempted:
                    raise ValueError("发送确认序号重复或越界")
                self.run.attempted.add(sequence)
                self.last_tx_time = time.monotonic()
                if self.first_tx_time is None:
                    self.first_tx_time = self.last_tx_time
                if error == 0:
                    self.run.submitted.add(sequence)
                else:
                    self.run.tx_errors += 1
            elif tag == "RX" and role == "Bob":
                item = self.run.receive(fields)
                if item:
                    self.rssi.add(item["rssi"], time.monotonic())
                    self.live_rssi.add(item["rssi"], time.monotonic())
                    self.dirty = True
                    error = self.run.comparisons.get(item["sequence"], {}).get("bit_errors", 0)
                    if error:
                        self.log(f"第{item['sequence']}块实际错误{error}位，RSSI {item['rssi']} dBm，接收状态{item['state']}")
            if tag in ("START", "TX", "DONE", "STOPPED", "PEER_OK"):
                self.acks.put((role, fields))

    def snapshot(self):
        result = self.run.summary()
        result.update(self.rssi.summary())
        result.update(serial_record_errors=self.serial_errors-self.serial_baseline, host_queue_drops=self.host_drops-self.host_baseline,
            bob_queue_drops=max(0, self.board_stats[3]-self.drop_baseline),
            foreign_frames_filtered_lifetime=self.board_stats[5],
            transport_broken=self.transport_broken, bob_lifetime_counters=list(self.board_stats))
        duration = (self.last_tx_time - self.first_tx_time
                    if self.first_tx_time is not None and self.last_tx_time is not None else 0)
        result["tx_ack_span_seconds"] = duration
        accepted_bytes = sum(min(CHUNK, len(self.run.source)-seq*CHUNK) for seq in self.run.submitted)
        result["approx_payload_bps"] = accepted_bytes*8/duration if duration > 0 else None
        return result

    def render_received(self):
        image = self.run.receiver.image() if self.run else None
        if image is not None:
            self.show_image("received", image)
        self.dirty = False; self.last_render = time.monotonic()

    def queue_save(self, run, session, result):
        self.saves.append((self.saver.submit(save_round, run, session, result), session))

    def close_draining(self, run_id):
        context = self.draining.pop(run_id)
        context['log'].close()
        result = context['result']
        result.update(context['run'].summary())
        result.update(context['rssi'].summary())
        # Snapshot the counters at this round's close instead of leaving the
        # detach-time values in place. Loop rounds use overlapping observation
        # windows, so the value is explicitly marked and must not be summed.
        at_close = list(self.board_stats)
        result['bob_counters_at_close'] = at_close
        result['bob_queue_drops'] = max(0, at_close[3] - context['drop_baseline'])
        result['bob_queue_drops_overlapping_window'] = True
        result['foreign_frames_filtered_lifetime'] = at_close[5]
        result['serial_record_errors'] = self.serial_errors - context['serial_baseline']
        result['host_queue_drops'] = self.host_drops - context['host_baseline']
        result['background_tail_seconds'] = 3
        self.queue_save(context['run'], context['session'], result)
        self.rssi_text.set(self.live_rssi.label(time.monotonic(), self.active or self.loop_enabled or bool(self.draining)))

    def detach_round(self, reason):
        result = self.snapshot()
        result['finish_reason'] = reason
        self.draining[self.run.run] = dict(run=self.run, session=self.session,
            log=self.capture_log, rssi=self.rssi, result=result,
            deadline=time.monotonic()+3, drop_baseline=self.drop_baseline,
            serial_baseline=self.serial_baseline, host_baseline=self.host_baseline)
        self.capture_log = None
        self.active = False
        self.finish_at = None
        self.final_summary = result
        self.loop_after = self.after(1, self.next_loop_round)

    def finish(self, reason):
        if not self.active:
            return
        self.active = False; self.finish_at = None
        self.rssi_text.set(self.live_rssi.label(time.monotonic(), bool(self.draining)))
        self.render_received()
        result = self.snapshot()
        result["finish_reason"] = reason
        self.final_summary = result
        if self.capture_log:
            self.capture_log.close(); self.capture_log = None
        self.queue_save(self.run, self.session, result)
        if result["serial_record_errors"] or result["host_queue_drops"] or result["bob_queue_drops"] or result["transport_broken"] or result["dimension_mismatch"]:
            note = "本地链路或数据有异常，本轮需复核"
        elif not result["unique_received"]:
            note = "未取得有效图像载荷，无法绘制接收图像"
        else:
            note = f"收到{result['unique_received']}/{result['total_blocks']}块，错误像素{result['pixel_errors']}，缺失块{result['missing_blocks']}"
        if result.get("local_fcs_fail_frames"):
            note += f"；本端FCS失败帧{result['local_fcs_fail_frames']}，原始误码块{result.get('raw_fcs_error_blocks', 0)}（见 raw_error.png）"
        if result["zero_state_payload_error_frames"]:
            note += "；存在接收状态正常但载荷不一致的块，请复核日志，不能直接归因于无线误码"
        self.status.set(reason + " · " + note)
        if self.loop_enabled:
            if result["sender_complete"] and not self.stop_event.is_set() and not result["transport_broken"]:
                self.status.set(f"第{self.loop_round}轮完成 · {note} · 即将开始下一轮")
                self.loop_after = self.after(100, self.next_loop_round)
            else:
                self.cancel_loop()

    def poll(self):
        for _ in range(300):
            try:
                generation, role, value = self.events.get_nowait()
            except queue.Empty:
                break
            if role == "BUILD":
                self.log(value); continue
            if role == "BUILD_DONE":
                self.building = False; self.status.set(value); self.log(value); continue
            if generation != self.generation:
                continue
            if role == "PORT_ERROR":
                self.log("串口异常 " + value); self.info.clear()
                self.transport_broken = True; self.stop()
                continue
            if role == "SENDER_DONE":
                run_id, complete, reason = value
                if self.active and run_id == self.run.run:
                    self.run.sender_complete = complete
                    self.finish_reason = reason
                    if complete and self.loop_enabled and not self.stop_event.is_set() and not self.transport_broken:
                        self.detach_round(reason)
                        continue
                    self.finish_at = time.monotonic() + 3
                    self.status.set(reason + "，等待剩余接收记录")
                continue
            try:
                self.handle(role, value)
            except Exception as exc:
                self.serial_errors += 1
                self.log("记录处理失败：" + str(exc))
        now = time.monotonic()
        for run_id, context in list(self.draining.items()):
            if now >= context['deadline'] and self.events.empty():
                self.close_draining(run_id)
        for future, session in list(self.saves):
            if future.done():
                self.saves.remove((future, session))
                try:
                    future.result()
                    self.log("结果已保存 " + str(session))
                except Exception as exc:
                    self.log("后台保存失败：" + str(exc))
                    self.stop()
        if self.ports and now - self.last_rssi_render >= 0.1:
            self.rssi_text.set(self.live_rssi.label(now, self.active or self.loop_enabled or bool(self.draining)))
            self.last_rssi_render = now
        if self.dirty and now - self.last_render >= 0.1:
            self.render_received()
        if self.run:
            result = self.final_summary if self.final_summary is not None else self.snapshot()
            ber = "—" if result["conditional_ber"] is None else f"{100*result['conditional_ber']:.5f}%"
            per = result.get("packet_error_rate")
            per_text = "—" if per is None else f"{100*per:.1f}%"
            fcs_rate = result.get("local_fcs_fail_rate")
            fcs_text = "—" if fcs_rate is None else f"{100*fcs_rate:.2f}%"
            head_rate = result.get("header_corrupt_rate")
            head_text = "—" if head_rate is None else f"{100*head_rate:.2f}%"
            rate_hist = result.get("phy_rate_mbps_histogram") or {}
            rate_text = ",".join(f"{k}Mbps×{v}" for k, v in sorted(rate_hist.items())) or "—"
            self.progress["value"] = 100 * result["received_bytes"] / result["total_bytes"]
            self.metrics.set(
                f"已提交 {result['api_accepted']}/{result['total_blocks']}块    收到 {result['unique_received']}块    接收覆盖 {100*result['received_bytes']/result['total_bytes']:.1f}%\n"
                f"空口丢包率 {per_text}    本端FCS失败 {result.get('local_fcs_fail_frames', 0)}/{result.get('local_frames', 0)} ({fcs_text})    头损坏率 {head_text}    原始误码块 {result.get('raw_fcs_error_blocks', 0)}\n"
                f"条件载荷BER {ber}（仅已通过头校验的载荷）    串口错误 {result['serial_record_errors']}    Bob队列丢弃 {result['bob_queue_drops']}    电脑队列丢弃 {result['host_queue_drops']}\n"
                f"实测PHY速率 {rate_text}    过滤环境帧 {result.get('foreign_frames_filtered_lifetime', 0)}")
            if result.get("phy_metadata_samples", 0) == 0 and result['unique_received'] > 0:
                self.metrics.set(self.metrics.get() + "    注意：未收到PHY元数据，当前两端为旧固件，请重新烧录")
            if result.get("approx_payload_bps") is not None:
                self.metrics.set(self.metrics.get() + f"    提交耗时 {result['tx_ack_span_seconds']:.1f}s / 约 {result['approx_payload_bps']/1000:.1f} kbit/s")
        if self.active and self.finish_at and now >= self.finish_at and self.events.empty():
            self.finish(self.finish_reason)
        if now - self.last_flush > 1:
            for file in (self.connection_log, self.capture_log, *(c['log'] for c in self.draining.values())):
                if file:
                    file.flush()
            self.last_flush = now
        self.poll_after = self.after(5, self.poll)

    def open_folder(self, name):
        folder = ROOT / name; folder.mkdir(exist_ok=True)
        os.startfile(folder)

    def close(self):
        if self.building:
            messagebox.showinfo("正在编译", "请等待编译或烧录完成后关闭。")
            return
        self.disconnect()
        self.saver.shutdown(wait=True)
        self.after_cancel(self.poll_after)
        self.destroy()


if __name__ == "__main__":
    ImageApp().mainloop()
