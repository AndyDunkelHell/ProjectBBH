"""
Arduino serial bridge for SpiRob, with proper mapping from tendon shortening [m]
to servo angles [deg].

Protocol:

- Handshake on start():
    repeatedly send    "!connect\r"
    expect any non-empty reply line from Arduino

- Command on send_ctrl(*dL):
    construct          "!UA a0 a1 a2 a3\r"
    where each a_i is an integer servo angle in [min_deg, max_deg]

Angle mapping:

Each tendon value passed from the GUI is a shortening ΔL_i in meters
(0.0 = no pull, >0 = pulled). We convert this to an angle based on a
cylindrical pulley:

    pulley circumference  C = π * d
    max cable travel      L_max = (max_deg - min_deg) / 360 * C

Then we clamp ΔL to [0, L_max] and map linearly:

    frac   = ΔL / L_max
    angle  = min_deg + frac * (max_deg - min_deg)

Tweak `min_deg`, `max_deg`, and `pulley_diameter_mm` in the constructor
to match test bench hardware.
"""

from __future__ import annotations
import threading
import queue
import time
import math
from typing import Optional, Sequence

import serial  # pip install pyserial


class ArduinoSerialBridge:
    def __init__(
        self,
        port: str,
        baud: int = 250000,
        *,
        min_deg: float = 0.0,
        max_deg: float = 190.0,
        pulley_diameter_mm: float = 15.0,
        connect_retries: int = 150,
        connect_timeout_s: float = 0.05,
    ):

        self.port = port
        self.baud = baud
        self.min_deg = float(min_deg)
        self.max_deg = float(max_deg)
        self.pulley_diameter_mm = float(pulley_diameter_mm)

        # Derived geometry
        self._deg_span = max(self.max_deg - self.min_deg, 1e-6)
        self._pulley_circ_m = math.pi * (self.pulley_diameter_mm / 1000.0)
        # Maximum cable travel for the configured angle span
        self._max_cable_m = self._pulley_circ_m * (self._deg_span / 360.0)
        if self._max_cable_m <= 0:
            # Fall back to something nonzero to avoid division by zero
            self._max_cable_m = 1e-3

        # Handshake settings
        self._connect_retries = connect_retries
        self._connect_timeout_s = connect_timeout_s

        # Runtime 
        self.q: "queue.Queue[str]" = queue.Queue()
        self._stop = threading.Event()
        self._thr: Optional[threading.Thread] = None
        self._ser: Optional[serial.Serial] = None

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    def start(self):
        """
        Open the serial port, perform a '!connect\\r' handshake, and then start the background writer
        thread.

        Raises RuntimeError if the port cannot be opened or the handshake fails.
        """
        if self._thr is not None:
            # already started
            return

        # Open serial
        try:
            self._ser = serial.Serial(
                self.port,
                baudrate=self.baud,
                timeout=self._connect_timeout_s,
            )
        except Exception as e:
            raise RuntimeError(f"Failed to open serial port {self.port}: {e}") from e

        # Required delay after opening port
        time.sleep(2.0)
        try:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except Exception:
            pass

        # Handshake: send '!connect\\r' until we see any reply
        ok = False
        for i in range(self._connect_retries):
            try:
                self._ser.write("!connect\r".encode())
                line = self._ser.readline().decode(errors="ignore").strip()
                # print(f"[handshake {i}] reply: {repr(line)}")  # debug
            except Exception:
                line = ""

            if line != "":
                ok = True
                break

        if not ok:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
            raise RuntimeError(
                f"No reply to '!connect' from Arduino on {self.port} "
                f"after {self._connect_retries} attempts"
            )

        # Start writer thread
        self._stop.clear()
        self._thr = threading.Thread(target=self._run, daemon=True)
        self._thr.start()

    def stop(self):
        """Stop writer thread and close the serial port."""
        self._stop.set()
        if self._thr is not None:
            self._thr.join(timeout=1.0)
        self._thr = None

        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def send_ctrl(self, *dLs: float):
        """
        Send tendon shortenings to the Arduino as a '!UA' command.

        Each argument in `dLs` is interpreted as a shortening ΔL [m] for one
        cable (as produced by `spirob_gui.py`).

        Map each ΔL to an angle in [min_deg, max_deg] based on the pulley
        geometry and then send:

            !UA a0 a1 a2 a3\\r

        The number of angles in the message matches the number of provided dLs.
        """
        if not dLs:
            return

        angles: list[int] = []
        for dL in dLs:
            dL_m = max(0.0, float(dL))        

            # Clamp to physical max travel of the cable
            dL_clamped = min(dL_m, self._max_cable_m)

            # Fraction of the usable travel
            frac = dL_clamped / self._max_cable_m

            # Linear mapping to angle range
            angle = self.min_deg + frac * self._deg_span

            angles.append(int(round(angle)))

        cmd = "!UA " + " ".join(str(a) for a in angles) + "\r"
        try:
            self.q.put_nowait(cmd)
        except queue.Full:
            # Drop oldest if queue somehow fills up
            try:
                _ = self.q.get_nowait()
                self.q.put_nowait(cmd)
            except queue.Empty:
                pass

    # ------------------------------------------------------------------ #
    # Internal worker
    # ------------------------------------------------------------------ #

    def _run(self):
        """Background loop to transmit queued messages to the Arduino."""
        assert self._ser is not None, "Serial port must be open before _run"
        ser = self._ser

        while not self._stop.is_set():
            try:
                msg = self.q.get(timeout=0.05)
            except queue.Empty:
                continue

            try:
                ser.write(msg.encode("ascii"))
                ser.flush()
            except Exception:
                # If something goes wrong, stop the thread; GUI can recreate bridge
                break
