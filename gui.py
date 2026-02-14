import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
from dataclasses import dataclass
import math

SOF1 = 0xAA
SOF2 = 0x55

# Message types from Arduino
MSG_LIVE = 0x81
MSG_DTC  = 0x82
MSG_ACK  = 0x83
MSG_ERR  = 0x84

# Commands to Arduino (single byte)
CMD_GET_LIVE = 0x01
CMD_GET_DTC  = 0x02
CMD_RESET    = 0x03


@dataclass
class LiveData:
    rpm: int = 0
    vss: int = 0
    ect_c: float = 0.0
    iat_c: float = 0.0
    map_kpa: float = 0.0
    tps_pct: float = 0.0
    batt_v: float = 0.0
    o2_v: float = 0.0
    flags: int = 0


def u16(hi, lo) -> int:
    return (hi << 8) | lo

def s16(hi, lo) -> int:
    v = (hi << 8) | lo
    if v & 0x8000:
        v -= 0x10000
    return v

def calc_crc8_sum(header_bytes: bytes, payload: bytes) -> int:
    # Arduino code: crc = mkcrc(header,4) + mkcrc(payload,len) ; then crc & 0xFF
    # We don't know mkcrc internals, BUT your sketch is summing bytes via mkcrc.
    # For GUI robustness, we can validate using simple sum-of-bytes (common approach).
    # If mkcrc is different, set validate_crc=False in parser call.
    return (sum(header_bytes) + sum(payload)) & 0xFF


class FrameParser:
    """
    Parses frames:
      AA 55 type len payload... crc
    CRC validation is OFF by default.
    """
    def __init__(self, validate_crc: bool = False):
        self.buf = bytearray()
        self.validate_crc = validate_crc

    def feed(self, data: bytes):
        self.buf.extend(data)

    def _find_sof(self):
        for i in range(max(0, len(self.buf) - 1)):
            if self.buf[i] == SOF1 and self.buf[i+1] == SOF2:
                return i
        return -1

    def next_frame(self):
        while True:
            if len(self.buf) < 4:
                return None

            idx = self._find_sof()
            if idx < 0:
                if len(self.buf) > 1:
                    self.buf = self.buf[-1:]
                return None

            if idx > 0:
                del self.buf[:idx]

            if len(self.buf) < 4:
                return None

            sof1, sof2, mtype, length = self.buf[0], self.buf[1], self.buf[2], self.buf[3]
            if sof1 != SOF1 or sof2 != SOF2:
                del self.buf[0]
                continue

            needed = 4 + length + 1
            if len(self.buf) < needed:
                return None

            payload = bytes(self.buf[4:4+length])
            crc = self.buf[4+length]

            if self.validate_crc:
                header = bytes(self.buf[0:4])
                exp = calc_crc8_sum(header, payload)
                if (crc & 0xFF) != exp:
                    # bad frame; drop SOF and resync
                    del self.buf[0:2]
                    continue

            # consume frame
            del self.buf[:needed]
            return mtype, payload


def decode_live(payload: bytes) -> LiveData:
    if len(payload) != 16:
        raise ValueError(f"Expected 16 bytes live payload, got {len(payload)}")

    rpm = u16(payload[0], payload[1])
    vss = payload[2]

    ect = s16(payload[3], payload[4]) / 10.0
    iat = s16(payload[5], payload[6]) / 10.0
    mapv = s16(payload[7], payload[8]) / 10.0
    tps = s16(payload[9], payload[10]) / 10.0

    batt = u16(payload[11], payload[12]) / 100.0
    o2 = u16(payload[13], payload[14]) / 1000.0

    flags = payload[15]

    return LiveData(
        rpm=rpm, vss=vss,
        ect_c=ect, iat_c=iat,
        map_kpa=mapv, tps_pct=tps,
        batt_v=batt, o2_v=o2,
        flags=flags
    )


class Gauge(ttk.Frame):
    """
    Simple round gauge drawn on a Canvas.
    - start_angle_deg and end_angle_deg define the sweep (degrees, canvas coords where 0° is at 3 o'clock, positive CCW)
      We'll use a "car gauge" style: left-bottom to right-bottom sweep.
    """
    def __init__(self, parent, title: str, unit: str, vmin: float, vmax: float,
                 major_step: float, minor_step: float,
                 size: int = 220,
                 start_angle_deg: float = 225.0,
                 end_angle_deg: float = -45.0):
        super().__init__(parent)
        self.title = title
        self.unit = unit
        self.vmin = float(vmin)
        self.vmax = float(vmax)
        self.major_step = float(major_step)
        self.minor_step = float(minor_step)
        self.size = int(size)
        self.start_angle = float(start_angle_deg)
        self.end_angle = float(end_angle_deg)

        self.value = self.vmin

        self.canvas = tk.Canvas(self, width=self.size, height=self.size, highlightthickness=0)
        self.canvas.pack()

        self.center = (self.size // 2, self.size // 2)
        self.radius = int(self.size * 0.38)

        self._draw_static()
        self.needle = None
        self._draw_needle(self.vmin)

    def _deg_to_rad(self, deg: float) -> float:
        return deg * math.pi / 180.0

    def _lerp(self, a: float, b: float, t: float) -> float:
        return a + (b - a) * t

    def _value_to_angle(self, v: float) -> float:
        v = max(self.vmin, min(self.vmax, v))
        t = (v - self.vmin) / (self.vmax - self.vmin) if self.vmax != self.vmin else 0.0
        return self._lerp(self.start_angle, self.end_angle, t)

    def _polar_to_xy(self, r: float, ang_deg: float):
        ang = self._deg_to_rad(ang_deg)
        cx, cy = self.center
        x = cx + r * math.cos(ang)
        y = cy - r * math.sin(ang)  # canvas y is down, so subtract
        return x, y

    def _draw_static(self):
        cx, cy = self.center
        r = self.radius

        # Outer ring
        self.canvas.create_oval(cx - r - 12, cy - r - 12, cx + r + 12, cy + r + 12, width=2)

        # Title
        self.canvas.create_text(cx, cy - r - 24, text=self.title, font=("Segoe UI", 11, "bold"))

        # Unit text (small)
        self.canvas.create_text(cx, cy + r + 24, text=self.unit, font=("Segoe UI", 10))

        # Tick marks
        # Major ticks with labels
        v = self.vmin
        while v <= self.vmax + 1e-9:
            ang = self._value_to_angle(v)

            # major tick
            x1, y1 = self._polar_to_xy(r * 0.92, ang)
            x2, y2 = self._polar_to_xy(r * 1.02, ang)
            self.canvas.create_line(x1, y1, x2, y2, width=2)

            # label
            lx, ly = self._polar_to_xy(r * 0.72, ang)
            label = f"{int(v)}" if abs(v - round(v)) < 1e-6 else f"{v:g}"
            self.canvas.create_text(lx, ly, text=label, font=("Segoe UI", 9))

            v += self.major_step

        # Minor ticks
        if self.minor_step > 0:
            v = self.vmin
            while v <= self.vmax + 1e-9:
                if abs((v - self.vmin) % self.major_step) > 1e-6:
                    ang = self._value_to_angle(v)
                    x1, y1 = self._polar_to_xy(r * 0.95, ang)
                    x2, y2 = self._polar_to_xy(r * 1.02, ang)
                    self.canvas.create_line(x1, y1, x2, y2, width=1)
                v += self.minor_step

        # Digital readout
        self.readout = self.canvas.create_text(cx, cy + r * 0.55, text="—", font=("Consolas", 16, "bold"))

        # Center cap
        self.canvas.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill="black")

    def _draw_needle(self, v: float):
        cx, cy = self.center
        r = self.radius
        ang = self._value_to_angle(v)

        x, y = self._polar_to_xy(r * 0.9, ang)

        if self.needle is None:
            self.needle = self.canvas.create_line(cx, cy, x, y, width=3)
        else:
            self.canvas.coords(self.needle, cx, cy, x, y)

        # Update readout
        if self.unit.lower() in ("km/h", "kph"):
            self.canvas.itemconfig(self.readout, text=f"{int(round(v))}")
        else:
            # for rpm show integer
            self.canvas.itemconfig(self.readout, text=f"{int(round(v))}")

    def set_value(self, v: float):
        self.value = float(v)
        self._draw_needle(self.value)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Honda OBD UNI2 - Live + DTC")
        self.resizable(True, True)


        self.ser = None
        self.rx_thread = None
        self.running = False
        self.parser = FrameParser(validate_crc=False)

        self.polling = False
        self.poll_ms = tk.IntVar(value=200)

        self._build_ui()
        self._refresh_ports()
        self.after(250, self._ui_tick)

    def _build_ui(self):
        pad = {"padx": 8, "pady": 6}

        top = ttk.LabelFrame(self, text="Connection")
        top.pack(fill="x", **pad)

        self.port_var = tk.StringVar()
        self.baud_var = tk.IntVar(value=115200)

        ttk.Label(top, text="Port:").grid(row=0, column=0, sticky="w", **pad)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=16, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="w", **pad)

        ttk.Button(top, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, **pad)

        ttk.Label(top, text="Baud:").grid(row=0, column=3, sticky="e", **pad)
        ttk.Entry(top, textvariable=self.baud_var, width=8).grid(row=0, column=4, sticky="w", **pad)

        self.btn_connect = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.btn_connect.grid(row=0, column=5, **pad)

        ctrl = ttk.LabelFrame(self, text="Controls")
        ctrl.pack(fill="x", **pad)

        ttk.Label(ctrl, text="Live poll (ms):").grid(row=0, column=0, sticky="w", **pad)
        ttk.Entry(ctrl, textvariable=self.poll_ms, width=8).grid(row=0, column=1, sticky="w", **pad)

        self.btn_poll = ttk.Button(ctrl, text="Start Live", command=self._toggle_poll, state="disabled")
        self.btn_poll.grid(row=0, column=2, **pad)

        self.btn_dtc = ttk.Button(ctrl, text="Get DTC", command=self._get_dtc, state="disabled")
        self.btn_dtc.grid(row=0, column=3, **pad)

        self.btn_reset = ttk.Button(ctrl, text="RESET ECU", command=self._reset_ecu, state="disabled")
        self.btn_reset.grid(row=0, column=4, **pad)

        # Gauges row
        gauges = ttk.Frame(self)
        gauges.pack(fill="x", padx=8, pady=6)

        # RPM gauge: 0..9000 (change if you want), majors 1000, minors 250
        self.g_rpm = Gauge(gauges, title="RPM", unit="rpm", vmin=0, vmax=9000,
                           major_step=1000, minor_step=250, size=260)
        self.g_rpm.pack(side="left", padx=10, pady=10)

        # Speed gauge: 0..240 km/h, majors 20, minors 10
        self.g_vss = Gauge(gauges, title="SPEED", unit="km/h", vmin=0, vmax=240,
                           major_step=20, minor_step=10, size=260)
        self.g_vss.pack(side="left", padx=10)

        # Live values (other)
        live = ttk.LabelFrame(self, text="Live Data (Other)")
        live.pack(fill="x", **pad)

        self.vars = {
            "ect": tk.StringVar(value="—"),
            "iat": tk.StringVar(value="—"),
            "map": tk.StringVar(value="—"),
            "tps": tk.StringVar(value="—"),
            "batt": tk.StringVar(value="—"),
            "o2": tk.StringVar(value="—"),
            "status": tk.StringVar(value="Disconnected"),
        }

        rows = [
            ("ECT (°C)", "ect", "IAT (°C)", "iat"),
            ("MAP", "map", "TPS (%)", "tps"),
            ("Batt (V)", "batt", "O2 (V)", "o2"),
            ("Status", "status", "", ""),
        ]

        for r, (l1, k1, l2, k2) in enumerate(rows):
            ttk.Label(live, text=l1 + ":").grid(row=r, column=0, sticky="e", **pad)
            ttk.Label(live, textvariable=self.vars[k1], width=18).grid(row=r, column=1, sticky="w", **pad)
            if l2:
                ttk.Label(live, text=l2 + ":").grid(row=r, column=2, sticky="e", **pad)
                ttk.Label(live, textvariable=self.vars[k2], width=22).grid(row=r, column=3, sticky="w", **pad)

        # Indicators
        ind = ttk.LabelFrame(self, text="Indicators (flags byte)")
        ind.pack(fill="x", **pad)

        self.lamps = {}

        def make_lamp(parent, text):
            f = ttk.Frame(parent)
            c = tk.Canvas(f, width=18, height=18, highlightthickness=0)
            c.create_oval(2, 2, 16, 16, fill="gray", outline="black", tags=("lamp",))
            c.pack(side="left")
            ttk.Label(f, text=text, width=7).pack(side="left", padx=(6, 0))
            f.pack(side="left", padx=12, pady=4)
            return c

        self.lamps["ac"]    = make_lamp(ind, "A/C")
        self.lamps["brake"] = make_lamp(ind, "BRAKE")
        self.lamps["vtec"]  = make_lamp(ind, "VTEC")
        self.lamps["cel"]   = make_lamp(ind, "CEL")

        self.vtec_big = tk.StringVar(value="VTEC OFF")
        ttk.Label(ind, textvariable=self.vtec_big, font=("Segoe UI", 12, "bold")).pack(side="right", padx=12)

        # DTC box
        dtc = ttk.LabelFrame(self, text="DTC")
        dtc.pack(fill="both", expand=True, **pad)

        self.dtc_text = tk.Text(dtc, height=8)
        self.dtc_text.pack(fill="both", expand=True, padx=8, pady=6)
        self._set_dtc_text(0, [])
        self.dtc_text.config(state="disabled")

    def _set_lamp(self, canvas: tk.Canvas, on: bool):
        canvas.itemconfig("lamp", fill=("lime" if on else "gray"))

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _set_controls_enabled(self, enabled: bool):
        state = "normal" if enabled else "disabled"
        self.btn_poll.config(state=state)
        self.btn_dtc.config(state=state)
        self.btn_reset.config(state=state)

    def _toggle_connect(self):
        if self.ser:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Error", "Select a COM port first.")
            return

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=int(self.baud_var.get()),
                timeout=0.05
            )
        except Exception as e:
            messagebox.showerror("Connect failed", str(e))
            self.ser = None
            return

        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        self.btn_connect.config(text="Disconnect")
        self._set_controls_enabled(True)
        self.vars["status"].set(f"Connected to {port}")

    def _disconnect(self):
        self.polling = False
        self.running = False
        try:
            if self.ser:
                self.ser.close()
        except:
            pass
        self.ser = None
        self.btn_connect.config(text="Connect")
        self._set_controls_enabled(False)
        self.btn_poll.config(text="Start Live")
        self.vars["status"].set("Disconnected")

    def _write_cmd(self, cmd: int):
        if not self.ser:
            return
        try:
            self.ser.write(bytes([cmd]))
        except Exception as e:
            self.vars["status"].set(f"TX error: {e}")

    def _toggle_poll(self):
        if not self.ser:
            return
        self.polling = not self.polling
        self.btn_poll.config(text=("Stop Live" if self.polling else "Start Live"))
        if self.polling:
            self.vars["status"].set("Live polling…")

    def _get_dtc(self):
        self._write_cmd(CMD_GET_DTC)

    def _reset_ecu(self):
        if messagebox.askyesno("Confirm", "Send ECU RESET?"):
            self._write_cmd(CMD_RESET)

    def _rx_loop(self):
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    if data:
                        self.parser.feed(data)
                else:
                    time.sleep(0.01)

                while True:
                    fr = self.parser.next_frame()
                    if fr is None:
                        break
                    mtype, payload = fr
                    self.after(0, lambda mt=mtype, pl=payload: self._handle_frame_ui(mt, pl))

            except Exception as e:
                self.after(0, lambda: self.vars["status"].set(f"RX error: {e}"))
                time.sleep(0.2)

    def _handle_frame_ui(self, mtype: int, payload: bytes):
        if mtype == MSG_LIVE:
            try:
                live = decode_live(payload)
            except Exception as e:
                self.vars["status"].set(f"Decode live error: {e}")
                return

            # Gauges
            self.g_rpm.set_value(live.rpm)
            self.g_vss.set_value(live.vss)

            # Other text fields
            self.vars["ect"].set(f"{live.ect_c:.1f}")
            self.vars["iat"].set(f"{live.iat_c:.1f}")
            self.vars["map"].set(f"{live.map_kpa:.1f}")
            self.vars["tps"].set(f"{live.tps_pct:.1f}")
            self.vars["batt"].set(f"{live.batt_v:.2f}")
            self.vars["o2"].set(f"{live.o2_v:.3f}")

            flags = live.flags
            self._set_lamp(self.lamps["ac"],    bool(flags & (1 << 0)))
            self._set_lamp(self.lamps["brake"], bool(flags & (1 << 1)))
            self._set_lamp(self.lamps["vtec"],  bool(flags & (1 << 2)))
            self._set_lamp(self.lamps["cel"],   bool(flags & (1 << 3)))
            self.vtec_big.set("VTEC ON" if (flags & (1 << 2)) else "VTEC OFF")

            if self.polling:
                self.vars["status"].set("Live OK")

        elif mtype == MSG_DTC:
            if not payload:
                return
            count = payload[0]
            dtcs = list(payload[1:1+count])
            self._set_dtc_text(count, dtcs)
            self.vars["status"].set("DTC received")

        elif mtype == MSG_ACK:
            ok = payload[0] if payload else 0
            self.vars["status"].set("RESET OK" if ok else "RESET FAIL")

        elif mtype == MSG_ERR:
            code = payload[0] if payload else 0xFF
            self.vars["status"].set(f"ECU ERR: {code}")

        else:
            self.vars["status"].set(f"Unknown msg: 0x{mtype:02X}")

    def _set_dtc_text(self, count, dtcs):
        self.dtc_text.config(state="normal")
        self.dtc_text.delete("1.0", "end")
        self.dtc_text.insert("end", f"Count: {count}\n")
        if count == 0:
            self.dtc_text.insert("end", "No DTC stored.\n")
        else:
            self.dtc_text.insert("end", "Raw DTC bytes:\n")
            self.dtc_text.insert("end", " ".join([f"0x{x:02X}" for x in dtcs]) + "\n\n")
            self.dtc_text.insert("end", "If these are Honda blink codes, map later.\n")
        self.dtc_text.config(state="disabled")

    def _ui_tick(self):
        if self.ser and self.polling:
            self._write_cmd(CMD_GET_LIVE)
        ms = max(50, int(self.poll_ms.get() or 200))
        self.after(ms, self._ui_tick)


if __name__ == "__main__":
    app = App()
    app.mainloop()
