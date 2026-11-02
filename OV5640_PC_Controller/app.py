"""PGL50H / OV5640 live demo controller. Requires Python 3.9+ and pyserial.

No additional FPGA firmware is required if the actual programmed video_cmd_ctrl.v
matches the version inspected on 2026-10-27 (types 10 and 12, commands 01..08).
"""
from __future__ import annotations

from datetime import datetime
import tkinter as tk
from tkinter import messagebox, ttk

from protocol import (BAUD, Config, build_coeff_packet, build_config_packet,
                      format_coeff, hex_bytes, parse_int32)

BG = "#0B1220"
CARD = "#162338"
CARD2 = "#1B2B45"
FG = "#EFF6FF"
MUTED = "#A5B8CE"
ACCENT = "#38C8FA"
GOOD = "#38D8B1"
WARN = "#F5B85F"
BORDER = "#31445D"


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PGL50H  |  FPGA 实时图像处理控制台")
        self.geometry("1200x860")
        self.minsize(1060, 750)
        self.configure(bg=BG)
        self.port = None
        self.port_map = {}
        self.serial_obj = None
        self.angle = tk.StringVar(value="0")
        self.tx = tk.StringVar(value="0")
        self.ty = tk.StringVar(value="0")
        self.thumbnail = tk.BooleanVar(value=False)
        self.correction = tk.BooleanVar(value=True)
        self.gamma = tk.BooleanVar(value=True)
        self.com_selected = tk.StringVar()
        self.connection_label = tk.StringVar(value="●  未连接")
        self.preview = tk.StringVar(value="")
        self.preview_detail = tk.StringVar(value="12 B  •  type 0x10")
        # 初始默认值改为 655, 16, 8, 8
        self.coeffs = [tk.StringVar(value=v) for v in ("655", "16", "8", "8")]
        self.coeff_preview = tk.StringVar()
        self.coeff_detail = tk.StringVar(value="24 B  •  type 0x12")
        self._build_styles()
        self._build_layout()
        for v in (self.angle, self.tx, self.ty, self.thumbnail, self.correction, self.gamma):
            v.trace_add('write', lambda *_: self.update_preview())
        for v in self.coeffs:
            v.trace_add('write', lambda *_: self.update_coeff_preview())
        self.update_preview()
        self.update_coeff_preview()
        self.after(120, self.refresh_ports)
        self.protocol("WM_DELETE_WINDOW", self.close_app)
        self.log("软件启动成功。选择 COM 端口并点击“打开串口”。")
        self.log("协议：115200 8N1，配置包 12 字节，四系数包 24 字节；FPGA 无回读确认。")

    def _build_styles(self):
        self.option_add('*Font', ('Microsoft YaHei UI', 10))
        style = ttk.Style(self)
        style.theme_use('clam')
        style.configure('Dark.TFrame', background=BG)
        style.configure('Card.TFrame', background=CARD)
        style.configure('Card2.TFrame', background=CARD2)
        style.configure('TLabel', background=CARD, foreground=FG, font=('Microsoft YaHei UI', 10))
        style.configure('Heading.TLabel', background=CARD, foreground=FG, font=('Microsoft YaHei UI', 13, 'bold'))
        style.configure('Muted.TLabel', background=CARD, foreground=MUTED, font=('Microsoft YaHei UI', 9))
        style.configure('Dark.TLabel', background=BG, foreground=FG)
        style.configure('Hero.TLabel', background=BG, foreground=FG, font=('Microsoft YaHei UI', 21, 'bold'))
        style.configure('Sub.TLabel', background=BG, foreground=MUTED, font=('Microsoft YaHei UI', 10))
        style.configure('TEntry', fieldbackground='#0E1A2E', foreground=FG,
                        bordercolor=BORDER, insertcolor=FG, padding=6)
        style.configure('TCombobox', fieldbackground='#0E1A2E', foreground=FG,
                        selectbackground=ACCENT, selectforeground=BG, padding=6)
        style.configure('TSpinbox', fieldbackground='#0E1A2E', foreground=FG,
                        bordercolor=BORDER, padding=5, arrowsize=12)
        style.configure('TCheckbutton', background=CARD, foreground=FG,
                        font=('Microsoft YaHei UI', 10), padding=(2, 4))
        style.map('TCheckbutton', background=[('active', CARD)], foreground=[('disabled', MUTED)])
        style.configure('Primary.TButton', background=ACCENT, foreground=BG,
                        borderwidth=0, padding=(17, 11), font=('Microsoft YaHei UI', 11, 'bold'))
        style.map('Primary.TButton', background=[('active', '#7BDFFF'), ('disabled', BORDER)],
                  foreground=[('disabled', MUTED)])
        style.configure('Secondary.TButton', background=CARD2, foreground=FG,
                        borderwidth=0, padding=(12, 9), font=('Microsoft YaHei UI', 10, 'bold'))
        style.map('Secondary.TButton', background=[('active', '#284565')])
        style.configure('Mini.TButton', background=CARD2, foreground=FG,
                        borderwidth=0, padding=(9, 5), font=('Microsoft YaHei UI', 9))
        style.map('Mini.TButton', background=[('active', '#284565')])
        style.configure('Port.TCombobox', fieldbackground='#0E1A2E', foreground=FG,
                        selectbackground=ACCENT, selectforeground=BG, padding=(8, 7))

    def _section(self, parent, title, subtitle=None):
        box = ttk.Frame(parent, style='Card.TFrame', padding=16)
        box.columnconfigure(0, weight=1)
        ttk.Label(box, text=title, style='Heading.TLabel').grid(row=0, column=0, sticky='w')
        if subtitle:
            ttk.Label(box, text=subtitle, style='Muted.TLabel', wraplength=520,
                      justify='left').grid(row=1, column=0, sticky='w', pady=(3, 10))
        else:
            ttk.Frame(box, style='Card.TFrame', height=8).grid(row=1, column=0)
        return box

    def _build_layout(self):
        outer = ttk.Frame(self, style='Dark.TFrame', padding=(18, 15, 18, 13))
        outer.pack(fill='both', expand=True)
        top = ttk.Frame(outer, style='Dark.TFrame')
        top.pack(fill='x', pady=(0, 14))
        ttk.Label(top, text='FPGA 视觉控制台', style='Hero.TLabel').pack(anchor='w')
        ttk.Label(top, text='PGL50H  /  OV5640  /  1080P  ·  UART 115200 8N1  ·  帧级更新',
                  style='Sub.TLabel').pack(anchor='w', pady=(4, 0))

        connection = self._section(outer, '01  串口连接', '自动扫描电脑 COM 端口；关闭串口时不会发送任何控制命令。')
        connection.pack(fill='x', pady=(0, 12))
        port_row = ttk.Frame(connection, style='Card.TFrame')
        port_row.grid(row=2, column=0, sticky='ew')
        port_row.columnconfigure(0, weight=1)
        self.combo = ttk.Combobox(port_row, state='readonly', textvariable=self.com_selected,
                                  style='Port.TCombobox', width=50)
        self.combo.grid(row=0, column=0, sticky='ew', padx=(0, 10))
        ttk.Button(port_row, text='刷新端口', style='Secondary.TButton',
                   command=self.refresh_ports).grid(row=0, column=1, padx=(0, 10))
        self.connect_btn = ttk.Button(port_row, text='打开串口', style='Primary.TButton',
                                      command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=(0, 14))
        self.status_lbl = tk.Label(port_row, textvariable=self.connection_label,
                                   bg=CARD, fg=WARN, font=('Microsoft YaHei UI', 10, 'bold'))
        self.status_lbl.grid(row=0, column=3)

        cols = ttk.Frame(outer, style='Dark.TFrame')
        cols.pack(fill='both', expand=True)
        cols.columnconfigure(0, weight=11)
        cols.columnconfigure(1, weight=10)
        cols.rowconfigure(0, weight=1)
        left = ttk.Frame(cols, style='Dark.TFrame')
        right = ttk.Frame(cols, style='Dark.TFrame')
        left.grid(row=0, column=0, sticky='nsew', padx=(0, 6))
        right.grid(row=0, column=1, sticky='nsew', padx=(6, 0))

        config_card = self._section(left, '02  动态配置 · 12 字节',
                                    'A5 5A 10 [flags] [角度LE16] [X LE16] [Y LE16] [XOR] 0D')
        config_card.pack(fill='x')
        form = ttk.Frame(config_card, style='Card.TFrame')
        form.grid(row=2, column=0, sticky='ew')
        form.columnconfigure(1, weight=1)
        self._spinrow(form, 0, '旋转角度 (°)', self.angle, 0, 359)
        self._spinrow(form, 1, '水平平移 X', self.tx, -960, 960)
        self._spinrow(form, 2, '垂直平移 Y', self.ty, -540, 540)
        switches = ttk.Frame(form, style='Card.TFrame')
        switches.grid(row=3, column=0, columnspan=3, sticky='ew', pady=(10, 4))
        for index, (label, var) in enumerate((('缩略图模式', self.thumbnail),
                                              ('畸变矫正', self.correction),
                                              ('Gamma增强', self.gamma))):
            ttk.Checkbutton(switches, text=label, variable=var).grid(row=0, column=index, padx=(0, 14), sticky='w')
        ttk.Label(config_card, text='注意：关闭缩略图时，硬件只接受角度=0且Y=0；X 平移仍可使用。',
                  style='Muted.TLabel').grid(row=3, column=0, sticky='w', pady=(4, 5))
        row = ttk.Frame(config_card, style='Card.TFrame')
        row.grid(row=4, column=0, sticky='ew', pady=(0, 8))
        ttk.Button(row, text='发送配置', style='Primary.TButton',
                   command=self.send_config).pack(side='left', padx=(0, 8))
        ttk.Button(row, text='90° 旋转预设', style='Secondary.TButton',
                   command=self.preset_rotate).pack(side='left', padx=(0, 8))
        ttk.Button(row, text='恢复零位', style='Secondary.TButton',
                   command=self.preset_zero).pack(side='left')
        self._packet_panel(config_card, 5, self.preview_detail, self.preview,
                           lambda: self.copy_text(self.preview.get()))

        quick = self._section(right, '03  快捷指令', '发送单字节 HEX；01、05、06、07、08 均为翻转/循环指令。')
        quick.pack(fill='x', pady=(0, 12))
        grid = ttk.Frame(quick, style='Card.TFrame')
        grid.grid(row=2, column=0, sticky='ew')
        for c in range(3):
            grid.columnconfigure(c, weight=1)
        shortcuts = [
            ('01  畸变切换', 0x01), ('04  恢复全屏', 0x04), ('05  Gamma切换', 0x05),
            ('06  参考线开关', 0x06), ('07  参考线模式', 0x07), ('08  OSD开关', 0x08),
        ]
        for i, (label, byte) in enumerate(shortcuts):
            ttk.Button(grid, text=label, style='Secondary.TButton',
                       command=lambda b=byte: self.send_shortcut(b)).grid(
                row=i//3, column=i%3, padx=3, pady=4, sticky='ew')
        ttk.Label(quick, text='无状态回读。快捷开关依赖板端当前状态；需确定状态时，请发送完整配置包。',
                  style='Muted.TLabel', wraplength=520).grid(row=3, column=0, sticky='w', pady=(9, 0))

        coeff = self._section(right, '04  畸变系数 · 24 字节',
                              'Q16.16 有符号32位整数。支持 655、-123、0x0000028F、0xFFFFFFFF。')
        coeff.pack(fill='x')
        cf = ttk.Frame(coeff, style='Card.TFrame')
        cf.grid(row=2, column=0, sticky='ew')
        cf.columnconfigure(1, weight=1)
        for row_idx, (name, var) in enumerate(zip(('K1', 'K2', 'P1', 'P2'), self.coeffs)):
            ttk.Label(cf, text=name).grid(row=row_idx, column=0, sticky='w', padx=(0, 10), pady=3)
            ttk.Entry(cf, textvariable=var, width=15).grid(row=row_idx, column=1, sticky='ew', padx=(0, 8))
            display = tk.Label(cf, bg=CARD, fg=MUTED, font=('Consolas', 9), anchor='w')
            display.grid(row=row_idx, column=2, sticky='w')
            var.trace_add('write', lambda *_, v=var, label=display: self.render_coeff(v, label))
            self.render_coeff(var, display)
        ttk.Label(coeff, text='范围 K1/K2: ±2621，P1/P2: ±262；同时检查硬件组合安全预算。',
                  style='Muted.TLabel').grid(row=3, column=0, sticky='w', pady=(8, 5))
        btn_row = ttk.Frame(coeff, style='Card.TFrame')
        btn_row.grid(row=4, column=0, sticky='ew', pady=(0, 7))
        ttk.Button(btn_row, text='加载全部系数', style='Primary.TButton',
                   command=self.send_coeff).pack(side='left', padx=(0, 8))
        # 修改点3：按钮文字改为“恢复演示参数输入”
        ttk.Button(btn_row, text='恢复演示参数输入', style='Secondary.TButton',
                   command=self.default_coeff).pack(side='left')
        self._packet_panel(coeff, 5, self.coeff_detail, self.coeff_preview,
                           lambda: self.copy_text(self.coeff_preview.get()))

        log_card = self._section(left, '05  发送日志', '所有报文以 HEX 记录；“发送成功”仅表示电脑串口写入，不代表 FPGA 已接收或执行。')
        log_card.pack(fill='both', expand=False, pady=(12, 0))
        log_row = ttk.Frame(log_card, style='Card.TFrame')
        log_row.grid(row=2, column=0, sticky='nsew')
        log_row.columnconfigure(0, weight=1)
        log_card.rowconfigure(2, weight=1)
        self.log_box = tk.Text(log_row, height=6, bg='#0B1728', fg='#C8D8E8',
                               insertbackground=FG, borderwidth=0, padx=10, pady=7,
                               font=('Consolas', 9), wrap='word', state='disabled')
        self.log_box.grid(row=0, column=0, sticky='nsew')
        scroll = ttk.Scrollbar(log_row, orient='vertical', command=self.log_box.yview)
        scroll.grid(row=0, column=1, sticky='ns')
        self.log_box.configure(yscrollcommand=scroll.set)
        ttk.Button(log_row, text='清空', style='Mini.TButton',
                   command=self.clear_log).grid(row=0, column=2, padx=(8, 0), sticky='n')

    def _spinrow(self, parent, row, name, variable, low, high):
        ttk.Label(parent, text=name).grid(row=row, column=0, sticky='w', pady=5, padx=(0, 16))
        ttk.Spinbox(parent, textvariable=variable, from_=low, to=high,
                    increment=1, width=13).grid(row=row, column=1, sticky='ew', pady=5)
        ttk.Label(parent, text=f'{low} ～ {high}', style='Muted.TLabel').grid(
            row=row, column=2, sticky='w', padx=(12, 0))

    def _packet_panel(self, parent, row, detail_var, packet_var, on_copy):
        container = ttk.Frame(parent, style='Card2.TFrame', padding=10)
        container.grid(row=row, column=0, sticky='ew')
        container.columnconfigure(0, weight=1)
        header = ttk.Frame(container, style='Card2.TFrame')
        header.grid(row=0, column=0, sticky='ew')
        header.columnconfigure(0, weight=1)
        tk.Label(header, textvariable=detail_var, bg=CARD2, fg=ACCENT,
                 font=('Consolas', 10, 'bold')).grid(row=0, column=0, sticky='w')
        ttk.Button(header, text='复制HEX', style='Mini.TButton', command=on_copy).grid(row=0, column=1)
        tk.Label(container, textvariable=packet_var, bg=CARD2, fg=FG,
                 font=('Consolas', 9), justify='left', anchor='w',
                 wraplength=500).grid(row=1, column=0, sticky='ew', pady=(5, 0))

    def make_config(self) -> Config:
        return Config(angle=int(self.angle.get().strip(), 10),
                      tx=int(self.tx.get().strip(), 10),
                      ty=int(self.ty.get().strip(), 10),
                      thumbnail=self.thumbnail.get(),
                      correction=self.correction.get(),
                      gamma=self.gamma.get())

    def update_preview(self):
        try:
            p = build_config_packet(self.make_config())
            self.preview.set(hex_bytes(p))
            self.preview_detail.set(f'{len(p)} B   ·   HEX   ·   XOR=0x{p[-2]:02X}')
        except (ValueError, TypeError) as err:
            self.preview.set(f'待修正配置：{err}')
            self.preview_detail.set('12 B   ·   配置无效')

    def make_coeff(self):
        return tuple(parse_int32(v.get()) for v in self.coeffs)

    def update_coeff_preview(self):
        try:
            p = build_coeff_packet(*self.make_coeff())
            self.coeff_preview.set(hex_bytes(p))
            self.coeff_detail.set(f'{len(p)} B   ·   HEX   ·   XOR=0x{p[-2]:02X}')
        except (ValueError, TypeError) as err:
            self.coeff_preview.set(f'待修正参数：{err}')
            self.coeff_detail.set('24 B   ·   参数无效')

    def render_coeff(self, var, label):
        try:
            value = parse_int32(var.get())
            label.configure(text=f'= {value/65536:+.6f}', fg=GOOD)
        except ValueError:
            label.configure(text='输入无效', fg=WARN)

    def preset_rotate(self):
        self.angle.set('90')
        self.tx.set('0')
        self.ty.set('0')
        self.thumbnail.set(True)

    def preset_zero(self):
        self.angle.set('0')
        self.tx.set('0')
        self.ty.set('0')
        self.thumbnail.set(False)

    def default_coeff(self):
        # 修改点2：点击“恢复演示参数输入”按钮时，恢复的四元组
        for v, default in zip(self.coeffs, ('655', '16', '8', '8')):
            v.set(default)

    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            entries = sorted(list_ports.comports(), key=lambda p: p.device.lower())
            self.port_map = {f'{p.device}   —   {p.description}': p.device for p in entries}
            selected_device = self.port_map.get(self.com_selected.get())
            displays = list(self.port_map.keys())
            self.combo['values'] = displays
            if selected_device:
                for item, device in self.port_map.items():
                    if device == selected_device:
                        self.com_selected.set(item)
                        break
            elif self.serial_obj and self.serial_obj.is_open:
                for item, device in self.port_map.items():
                    if device == self.port:
                        self.com_selected.set(item)
                        break
            elif displays and self.com_selected.get() not in self.port_map:
                self.com_selected.set(displays[0])
            elif not displays:
                self.com_selected.set('')
        except ImportError:
            self.com_selected.set('请先安装 pyserial：pip install pyserial')
            self.combo['values'] = ()
        except Exception as err:
            self.log(f'端口扫描异常：{err}')
        if not getattr(self, '_closing', False):
            if self._refresh_after_id:
                try: self.after_cancel(self._refresh_after_id)
                except tk.TclError: pass
            self._refresh_after_id = self.after(3500, self.refresh_ports)

    _refresh_after_id = None

    def toggle_connection(self):
        if self.serial_obj and self.serial_obj.is_open:
            self.disconnect()
            return
        target = self.port_map.get(self.com_selected.get())
        if not target:
            messagebox.showwarning('未选择串口', '请先连接 FPGA 串口并从下拉列表选择 COM 口。')
            return
        try:
            import serial
            self.serial_obj = serial.Serial(port=target, baudrate=BAUD,
                                            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                            stopbits=serial.STOPBITS_ONE, timeout=0,
                                            write_timeout=0.8)
            self.port = target
            self.connection_label.set(f'●  已连接 {target}')
            self.status_lbl.configure(fg=GOOD)
            self.connect_btn.configure(text='关闭串口')
            self.log(f'已打开 {target} · {BAUD} 8N1')
        except ImportError:
            messagebox.showerror('缺少 pyserial', '请先运行：python -m pip install pyserial')
        except Exception as err:
            self.serial_obj = None
            messagebox.showerror('连接失败', f'{target} 无法打开：\n{err}')
            self.log(f'打开串口失败：{err}')

    def disconnect(self):
        try:
            if self.serial_obj:
                self.serial_obj.close()
        except Exception:
            pass
        self.serial_obj = None
        self.port = None
        self.connection_label.set('●  未连接')
        self.status_lbl.configure(fg=WARN)
        self.connect_btn.configure(text='打开串口')
        self.log('串口已关闭。')

    def send(self, data: bytes, description: str) -> bool:
        if not self.serial_obj or not self.serial_obj.is_open:
            messagebox.showwarning('串口未连接', '请先打开串口。可先使用“复制HEX”查看将发送的报文。')
            return False
        try:
            written = self.serial_obj.write(data)
            self.serial_obj.flush()
            if written != len(data):
                raise OSError(f'写入不完整：{written}/{len(data)} 字节')
        except Exception as err:
            self.log(f'串口写入失败：{err}')
            messagebox.showerror('发送失败', f'串口写入失败：\n{err}')
            self.disconnect()
            return False
        self.log(f'TX {description} ({len(data)}B)  {hex_bytes(data)}')
        return True

    def send_config(self):
        try:
            c = self.make_config()
            packet = build_config_packet(c)
        except (ValueError, TypeError) as err:
            messagebox.showerror('配置无效', str(err))
            return
        self.send(packet, f'配置: angle={c.angle}, x={c.tx}, y={c.ty}, '
                          f'thumb={int(c.thumbnail)}, correction={int(c.correction)}, gamma={int(c.gamma)}')

    def send_coeff(self):
        try:
            values = self.make_coeff()
            packet = build_coeff_packet(*values)
        except (ValueError, TypeError) as err:
            messagebox.showerror('系数无效', str(err))
            return
        self.send(packet, '四参数加载: ' + ', '.join(f'{name}={value}' for name, value in
                                             zip(('K1','K2','P1','P2'), values)))

    def send_shortcut(self, code):
        names = {1:'畸变翻转',4:'恢复全屏',5:'Gamma翻转',6:'参考线翻转',7:'参考线模式循环',8:'OSD翻转'}
        if self.send(bytes([code]), f'快捷指令 {code:02X} ({names.get(code, "")})'):
            # UI values are desired/estimated, never an authenticated readback.
            if code == 1:
                self.correction.set(not self.correction.get())
            elif code == 5:
                self.gamma.set(not self.gamma.get())
            elif code == 4:
                self.preset_zero()

    def copy_text(self, text):
        if not text or text.startswith('待修正'):
            messagebox.showinfo('不可复制', '当前参数无效，请先修正。')
            return
        self.clipboard_clear()
        self.clipboard_append(text)
        self.log('已复制十六进制报文到剪贴板。')

    def log(self, line):
        if not hasattr(self, 'log_box'):
            return
        tm = datetime.now().strftime('%H:%M:%S')
        self.log_box.configure(state='normal')
        self.log_box.insert('end', f'[{tm}] {line}\n')
        self.log_box.see('end')
        self.log_box.configure(state='disabled')

    def clear_log(self):
        self.log_box.configure(state='normal')
        self.log_box.delete('1.0', 'end')
        self.log_box.configure(state='disabled')

    def close_app(self):
        self._closing = True
        if self._refresh_after_id:
            try: self.after_cancel(self._refresh_after_id)
            except tk.TclError: pass
        if self.serial_obj:
            try: self.serial_obj.close()
            except Exception: pass
        self.destroy()


def main():
    # Windows high-DPI hint; ignore if unavailable.
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (OSError, AttributeError):
        pass
    App().mainloop()


if __name__ == '__main__':
    main()