import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
from dataclasses import dataclass

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
    """
    def __init__(self, validate_crc=False):
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
                # keep last byte in case it's 0xAA
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


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Honda OBD UNI2 - Live + DTC")
        self.geometry("620x460")
        self.resizable(False, False)

        self.ser = None
        self.rx_thread = None
        self.running = False
        self.parser = FrameParser(validate_crc=False)  # set True if mkcrc == sum-of-bytes

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

        # Live values
        live = ttk.LabelFrame(self, text="Live Data")
        live.pack(fill="x", **pad)

        self.vars = {
            "rpm": tk.StringVar(value="—"),
            "vss": tk.StringVar(value="—"),
            "ect": tk.StringVar(value="—"),
            "iat": tk.StringVar(value="—"),
            "map": tk.StringVar(value="—"),
            "tps": tk.StringVar(value="—"),
            "batt": tk.StringVar(value="—"),
            "o2": tk.StringVar(value="—"),
            "status": tk.StringVar(value="Disconnected"),
        }

        rows = [
            ("RPM", "rpm", "VSS (km/h)", "vss"),
            ("ECT (°C)", "ect", "IAT (°C)", "iat"),
            ("MAP", "map", "TPS (%)", "tps"),
            ("Batt (V)", "batt", "O2 (V)", "o2"),
            ("Status", "status", "", ""),
        ]

        for r, (l1, k1, l2, k2) in enumerate(rows):
            ttk.Label(live, text=l1 + ":").grid(row=r, column=0, sticky="e", **pad)
            ttk.Label(live, textvariable=self.vars[k1], width=16).grid(row=r, column=1, sticky="w", **pad)
            if l2:
                ttk.Label(live, text=l2 + ":").grid(row=r, column=2, sticky="e", **pad)
                ttk.Label(live, textvariable=self.vars[k2], width=22).grid(row=r, column=3, sticky="w", **pad)

        # Indicators
        ind = ttk.LabelFrame(self, text="Indicators (from flags byte)")
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

    # ---------- Serial ----------
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

    # ---------- Buttons ----------
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

    # ---------- RX thread ----------
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

            self.vars["rpm"].set(str(live.rpm))
            self.vars["vss"].set(str(live.vss))
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

    # ---------- UI poll tick ----------
    def _ui_tick(self):
        if self.ser and self.polling:
            self._write_cmd(CMD_GET_LIVE)
        ms = max(50, int(self.poll_ms.get() or 200))
        self.after(ms, self._ui_tick)


if __name__ == "__main__":
    app = App()
    app.mainloop()
