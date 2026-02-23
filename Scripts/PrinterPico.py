import argparse
import asyncio
import csv
import fcntl
import io
import json
import os
import platform
import select
import subprocess
import threading
import time
from dataclasses import dataclass
from datetime import datetime
import termios

import numpy as np
import websockets


DEFAULT_MOONRAKER_WS = "ws://10.0.0.76/websocket"
RETURN_LIFT_MM = 40.0


class MoonrakerRPC:
    def __init__(self, ws):
        self._ws = ws
        self._next_id = 1

    async def call(self, method, params=None):
        call_id = self._next_id
        self._next_id += 1

        msg = {"jsonrpc": "2.0", "method": method, "id": call_id}
        if params is not None:
            msg["params"] = params
        await self._ws.send(json.dumps(msg))

        while True:
            resp = json.loads(await self._ws.recv())
            if resp.get("id") == call_id:
                return resp


def _extract_toolhead_position(query_resp):
    # Moonraker typically returns:
    # {"result":{"status":{"toolhead":{"position":[x,y,z,e],...}},...},...}
    try:
        status = query_resp["result"]["status"]
        toolhead = status["toolhead"]
        pos = toolhead["position"]
        return float(pos[0]), float(pos[1]), float(pos[2])
    except Exception:
        return None


async def wait_for_position(
    rpc,
    *,
    x=None,
    y=None,
    z=None,
    tol_mm=0.25,
    timeout_s=60.0,
    poll_s=0.1,
):
    deadline = time.monotonic() + float(timeout_s)
    targets = {"x": x, "y": y, "z": z}
    while True:
        resp = await rpc.call("printer.objects.query", {"objects": {"toolhead": ["position"]}})
        pos = _extract_toolhead_position(resp)
        if pos is not None:
            px, py, pz = pos
            ok = True
            if targets["x"] is not None:
                ok = ok and abs(px - float(targets["x"])) <= float(tol_mm)
            if targets["y"] is not None:
                ok = ok and abs(py - float(targets["y"])) <= float(tol_mm)
            if targets["z"] is not None:
                ok = ok and abs(pz - float(targets["z"])) <= float(tol_mm)
            if ok:
                return

        if time.monotonic() >= deadline:
            raise TimeoutError(f"Timed out waiting for position x={x} y={y} z={z}")
        await asyncio.sleep(float(poll_s))


@dataclass(frozen=True)
class SweepConfig:
    x_start: float = 0
    x_end: float = 350
    y: float = 150
    z: float = 40
    travel_feed_mm_min: float = 6000
    z_feed_mm_min: float = 1200
    passes: int = 1


class SerialLineLogger:
    def __init__(self, port, baud, log_path, write_header=True):
        self._port = port
        self._baud = int(baud)
        self._log_path = log_path
        self._write_header = write_header

        self._capture_event = threading.Event()
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._target_velocity_mm_s = None

        self._thread = None
        self._serial = None
        self._proc = None
        self._fd = None
        self._bio = None
        self._buf = bytearray()
        self._fh = None
        self._csv = None

    @staticmethod
    def _normalize_line(line):
        # Preserve content, but remove field padding like ",  52.40" -> ",52.40".
        if "," not in line:
            return line.strip()
        return ",".join(part.strip() for part in line.split(","))

    def start(self):
        os.makedirs(os.path.dirname(self._log_path) or ".", exist_ok=True)
        file_exists = os.path.exists(self._log_path) and os.path.getsize(self._log_path) > 0
        self._fh = open(self._log_path, "a", newline="")
        self._csv = csv.writer(self._fh)
        if self._write_header and not file_exists:
            self._csv.writerow(
                ["target_mm_s", "t_unix_s", "t_array_us", "vx_mm_s", "vy_mm_s"]
            )
            self._fh.flush()

        self._open_transport()
        self._thread = threading.Thread(target=self._run, name="serial-line-logger", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._capture_event.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

        if self._proc is not None:
            try:
                self._proc.terminate()
            except Exception:
                pass
            self._proc = None

        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

        if self._bio is not None:
            try:
                self._bio.close()
            except Exception:
                pass
            self._bio = None

        if self._fd is not None:
            try:
                os.close(self._fd)
            except Exception:
                pass
            self._fd = None

        if self._fh is not None:
            try:
                self._fh.flush()
            finally:
                self._fh.close()
            self._fh = None

    def begin_capture(self, target_velocity_mm_s):
        with self._lock:
            self._target_velocity_mm_s = float(target_velocity_mm_s)
        self._capture_event.set()

    def end_capture(self):
        self._capture_event.clear()
        with self._lock:
            self._target_velocity_mm_s = None

    def _resolve_port(self, port):
        if not port:
            raise ValueError("serial port is empty")
        if port.startswith("/dev/") and os.path.exists(port):
            return port

        if port.startswith("tty.") or port.startswith("cu."):
            candidate = f"/dev/{port}"
            if os.path.exists(candidate):
                return candidate

        candidates = [f"/dev/cu.{port}", f"/dev/tty.{port}", f"/dev/{port}"]
        for c in candidates:
            if os.path.exists(c):
                return c
        raise FileNotFoundError(f"Serial port not found: {port} (tried {candidates})")

    def _open_darwin_fd(self, port, baud):
        # macOS: termios does not expose arbitrary baud rates (no B250000).
        # Use IOSSIOSPEED ioctl to request a custom speed.
        # Value is stable on macOS (see sys/ttycom.h).
        IOSSIOSPEED = 0x80045402

        fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)

        # Raw mode
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
        iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
        oflag &= ~termios.OPOST
        lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
        cflag &= ~(termios.CSIZE | termios.PARENB)
        cflag |= termios.CS8
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 1  # 0.1s read timeout

        attrs = [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

        speed = int(baud)
        fcntl.ioctl(fd, IOSSIOSPEED, speed.to_bytes(8, byteorder="little", signed=True))
        return fd

    def _open_transport(self):
        resolved = self._resolve_port(self._port)
        self._port = resolved

        pyserial = None
        try:
            import serial as pyserial  # type: ignore
        except Exception:
            pyserial = None

        if pyserial is not None and hasattr(pyserial, "Serial"):
            try:
                self._serial = pyserial.Serial(self._port, baudrate=self._baud, timeout=0.2)
                return
            except Exception:
                self._serial = None

        # Fallback:
        # - macOS: open fd + configure raw + set speed via ioctl (supports 250000).
        # - others: configure port via stty, then stream via cat.
        if platform.system().lower() == "darwin":
            # Prefer /dev/cu.* on macOS to avoid tty open semantics.
            if self._port.startswith("/dev/tty."):
                cu_port = "/dev/cu." + self._port[len("/dev/tty.") :]
                if os.path.exists(cu_port):
                    self._port = cu_port

            try:
                self._fd = self._open_darwin_fd(self._port, self._baud)
            except PermissionError as e:
                raise PermissionError(f"Permission denied opening {self._port}: {e}") from e
            except Exception as e:
                raise RuntimeError(
                    f"Failed to open serial port {self._port} at {self._baud} baud on macOS: {e}. "
                    "Try a /dev/cu.* device (e.g. /dev/cu.usbmodemPICO1)."
                ) from e
            self._bio = io.FileIO(self._fd, mode="rb", closefd=False)
            return

        # Non-macOS fallback using stty/cat
        try:
            subprocess.run(["stty", "-F", self._port, str(self._baud), "raw", "-echo"], check=True)
        except subprocess.CalledProcessError:
            subprocess.run(["stty", "-f", self._port, str(self._baud), "raw", "-echo"], check=True)
        self._proc = subprocess.Popen(["cat", self._port], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)

    def _readline(self):
        if self._bio is not None and self._fd is not None:
            r, _, _ = select.select([self._fd], [], [], 0.2)
            if not r:
                return b""
            try:
                chunk = os.read(self._fd, 4096)
            except BlockingIOError:
                return b""
            except Exception:
                return None
            if not chunk:
                return None
            self._buf.extend(chunk)
            nl = self._buf.find(b"\n")
            if nl == -1:
                return b""
            line = bytes(self._buf[: nl + 1])
            del self._buf[: nl + 1]
            return line

        if self._serial is not None:
            try:
                raw = self._serial.readline()
            except Exception:
                return None
            if not raw:
                return b""
            return raw

        if self._proc is None or self._proc.stdout is None:
            return None
        try:
            return self._proc.stdout.readline()
        except Exception:
            return None

    def _run(self):
        while not self._stop_event.is_set():
            raw = self._readline()
            if raw is None:
                # Keep logger alive across transient serial read failures.
                time.sleep(0.05)
                continue
            if raw == b"":
                continue

            try:
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            except Exception:
                continue
            line = self._normalize_line(line)
            parts = [p.strip() for p in line.split(",")] if "," in line else [line.strip()]
            # Drop truncated/garbled lines that can appear around capture boundaries.
            if len(parts) < 3:
                continue
            try:
                int(parts[0])  # pico timestamp column
                float(parts[1])
            except Exception:
                continue

            if not self._capture_event.is_set():
                continue

            with self._lock:
                v = self._target_velocity_mm_s
            if v is None:
                continue

            self._csv.writerow([v, time.time(), *parts])
            self._fh.flush()


async def x_sweep(rpc, serial_logger, sweep: SweepConfig, v_mm_s):
    feed_sweep_mm_min = float(v_mm_s) * 60.0  # mm/s -> mm/min

    await rpc.call("printer.gcode.script", {"script": f"G1 X{sweep.x_start} F{sweep.travel_feed_mm_min}"})
    await rpc.call("printer.gcode.script", {"script": f"G1 Y{sweep.y} F{sweep.travel_feed_mm_min}"})
    await rpc.call("printer.gcode.script", {"script": f"G1 Z{sweep.z} F{sweep.z_feed_mm_min}"})
    await rpc.call("printer.gcode.script", {"script": "M400"})
    await wait_for_position(rpc, x=sweep.x_start, y=sweep.y, z=sweep.z, timeout_s=120.0)

    for p in range(int(sweep.passes)):
        await asyncio.sleep(1.0)
        print(f"\n===BEGIN SWEEP v={v_mm_s:.3f} mm/s pass={p + 1}/{sweep.passes}===")
        if serial_logger is not None:
            serial_logger.begin_capture(v_mm_s)
        await rpc.call("printer.gcode.script", {"script": f"G1 X{sweep.x_end} F{feed_sweep_mm_min}"})
        await rpc.call("printer.gcode.script", {"script": "M400"})
        if serial_logger is not None:
            serial_logger.end_capture()
        print("===SWEEP END===")

        lifted_z = sweep.z + RETURN_LIFT_MM
        await rpc.call("printer.gcode.script", {"script": f"G1 Z{lifted_z} F{sweep.z_feed_mm_min}"})
        await rpc.call("printer.gcode.script", {"script": f"G1 X{sweep.x_start} F{sweep.travel_feed_mm_min}"})
        await rpc.call("printer.gcode.script", {"script": f"G1 Z{sweep.z} F{sweep.z_feed_mm_min}"})
        await rpc.call("printer.gcode.script", {"script": "M400"})
        await wait_for_position(rpc, x=sweep.x_start, y=sweep.y, z=sweep.z, timeout_s=120.0)


def _parse_velocities(args):
    if args.velocities:
        return [float(v.strip()) for v in args.velocities.split(",") if v.strip()]
    return [float(v) for v in np.linspace(args.v_min, args.v_max, args.v_count)]


def _default_log_path():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join("log", f"printer_sweep_serial_{ts}.csv")


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--moonraker-ws", default=DEFAULT_MOONRAKER_WS)
    ap.add_argument("--home", action="store_true", help="Home (G28) before sweeps")

    ap.add_argument("--serial-port", default=None, help="Serial port to read (e.g. /dev/tty.usbmodemXXXX)")
    ap.add_argument("--serial-baud", type=int, default=250000)
    ap.add_argument("--log-path", default=_default_log_path())
    ap.add_argument("--no-header", action="store_true")

    ap.add_argument("--v-min", type=float, default=50.0)
    ap.add_argument("--v-max", type=float, default=250.0)
    ap.add_argument("--v-count", type=int, default=5)
    ap.add_argument("--velocities", default=None, help="Comma-separated mm/s list (overrides --v-*)")

    ap.add_argument("--x-start", type=float, default=SweepConfig.x_start)
    ap.add_argument("--x-end", type=float, default=SweepConfig.x_end)
    ap.add_argument("--y", type=float, default=SweepConfig.y)
    ap.add_argument("--z", type=float, default=SweepConfig.z)
    ap.add_argument("--passes", type=int, default=SweepConfig.passes)
    ap.add_argument("--travel-feed", type=float, default=SweepConfig.travel_feed_mm_min)
    ap.add_argument("--z-feed", type=float, default=SweepConfig.z_feed_mm_min)
    args = ap.parse_args()

    sweep = SweepConfig(
        x_start=args.x_start,
        x_end=args.x_end,
        y=args.y,
        z=args.z,
        travel_feed_mm_min=args.travel_feed,
        z_feed_mm_min=args.z_feed,
        passes=args.passes,
    )
    velocities = _parse_velocities(args)

    serial_logger = None
    if args.serial_port:
        serial_logger = SerialLineLogger(
            args.serial_port,
            args.serial_baud,
            args.log_path,
            write_header=not args.no_header,
        )
        serial_logger.start()
        time.sleep(0.25)  # give the reader a moment to start

    try:
        async with websockets.connect(args.moonraker_ws, ping_interval=20) as ws:
            rpc = MoonrakerRPC(ws)
            sub = await rpc.call(
                "printer.objects.subscribe",
                {
                    "objects": {
                        "toolhead": ["position", "status"],
                        "print_stats": ["state", "filename"],
                        "idle_timeout": ["state"],
                        "gcode_move": ["gcode_position"],
                    }
                },
            )
            print("Subscribed:", sub)

            if args.home:
                await rpc.call("printer.gcode.script", {"script": "G28"})

            for v in velocities:
                await x_sweep(rpc, serial_logger, sweep, v_mm_s=v)

            await rpc.call("printer.objects.query", {"objects": {"toolhead": None, "print_stats": None}})
    finally:
        if serial_logger is not None:
            serial_logger.stop()


if __name__ == "__main__":
    asyncio.run(main())
