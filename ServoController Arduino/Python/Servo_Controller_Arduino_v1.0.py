import sys
import time
import queue
import serial.tools.list_ports
import csv
import threading


from PySide6 import QtCore, QtWidgets, QtGui

# IMPORTANT: use your provided bridge
from arduino_serial import ArduinoSerialBridge  # :contentReference[oaicite:5]{index=5}


class PlaybackDialog(QtWidgets.QDialog):
    stop_requested = QtCore.Signal()
    disconnect_requested = QtCore.Signal()

    def __init__(self, total_steps: int, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Playback running")
        self.setWindowModality(QtCore.Qt.ApplicationModal)
        self.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint, True)

        layout = QtWidgets.QVBoxLayout(self)

        self.lb_info = QtWidgets.QLabel("Playing…")
        self.lb_info.setAlignment(QtCore.Qt.AlignLeft)
        layout.addWidget(self.lb_info)

        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, max(1, total_steps))
        self.progress.setValue(0)
        layout.addWidget(self.progress)

        btn_row = QtWidgets.QHBoxLayout()
        self.bt_stop = QtWidgets.QPushButton("Stop playback")
        self.bt_disconnect = QtWidgets.QPushButton("Disconnect (STOP)")
        self.bt_disconnect.setStyleSheet("font-weight: 600;")

        btn_row.addWidget(self.bt_stop)
        btn_row.addWidget(self.bt_disconnect)
        layout.addLayout(btn_row)

        self.bt_stop.clicked.connect(self.stop_requested.emit)
        self.bt_disconnect.clicked.connect(self.disconnect_requested.emit)

    def update_progress(self, step: int, total: int, rep: int, repeats: int):
        self.progress.setRange(0, max(1, total))
        self.progress.setValue(step)
        self.lb_info.setText(
            f"Repeat {rep}/{repeats} — Step {step}/{total}"
        )


class PlaybackWorker(QtCore.QObject):
    progress = QtCore.Signal(int, int, int, int)  # step, total, rep, repeats
    finished = QtCore.Signal()
    error = QtCore.Signal(str)

    def __init__(self, samples, repeats, send_fn, set_combo_fn, stop_flag, sleep_s=0.016):
        super().__init__()
        self.samples = samples            # list[(k,a0,a1,a2,a3)]
        self.repeats = repeats
        self.send_fn = send_fn            # callable(str cmd)
        self.set_combo_fn = set_combo_fn  # callable(int k)->bool
        self.stop_flag = stop_flag        # threading.Event
        self.sleep_s = sleep_s

    @QtCore.Slot()
    def run(self):
        try:
            total = len(self.samples) * self.repeats
            step = 0

            for rep in range(1, self.repeats + 1):
                last_k = None

                for (k, a0, a1, a2, a3) in self.samples:
                    if self.stop_flag.is_set():
                        self.finished.emit()
                        return

                    if last_k != k:
                        _ = self.set_combo_fn(k)
                        last_k = k

                    self.send_fn(f"!UA {a0} {a1} {a2} {a3}\r")

                    step += 1
                    self.progress.emit(step, total, rep, self.repeats)

                    time.sleep(self.sleep_s)

            self.finished.emit()
        except Exception as e:
            self.error.emit(str(e))


class SetupDialog(QtWidgets.QDialog):
    """Onboarding dialog: COM port + baud + min/max angle (global)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Servo Controller Setup")

        layout = QtWidgets.QFormLayout(self)

        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setEditable(True)
        ports = list(serial.tools.list_ports.comports())
        if ports:
            for p in ports:
                self.port_combo.addItem(p.device)
        else:
            default_port = "COM11" if sys.platform.startswith("win") else "/dev/ttyACM0"
            self.port_combo.addItem(default_port)

        self.baud_spin = QtWidgets.QSpinBox()
        self.baud_spin.setRange(1200, 10_000_000)
        self.baud_spin.setValue(250000)

        self.min_spin = QtWidgets.QSpinBox()
        self.min_spin.setRange(0, 180)
        self.min_spin.setValue(0)

        self.max_spin = QtWidgets.QSpinBox()
        self.max_spin.setRange(0, 180)
        self.max_spin.setValue(180)

        layout.addRow("COM port:", self.port_combo)
        layout.addRow("Baud rate:", self.baud_spin)
        layout.addRow("Min angle [deg]:", self.min_spin)
        layout.addRow("Max angle [deg]:", self.max_spin)

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def get_config(self) -> dict:
        return {
            "port": self.port_combo.currentText().strip(),
            "baud": int(self.baud_spin.value()),
            "min_deg": int(self.min_spin.value()),
            "max_deg": int(self.max_spin.value()),
        }


class ServoControlWindow(QtWidgets.QWidget):
    """
    16 servos total, controlled in 4 groups of 4:
      combo 0 -> servos 1..4   (k=0)
      combo 1 -> servos 5..8   (k=4)
      combo 2 -> servos 9..12  (k=8)
      combo 3 -> servos 13..16 (k=12)

    Commands match your Tk script:
      - '!UK k\\r' to change combo, wait for Arduino to reply 'k'
      - '!UA a b c d\\r' to send angles
    """

    def __init__(self, cfg: dict, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Servo Controller v2.0")

        self.cfg = cfg
        self.min_deg = cfg["min_deg"]
        self.max_deg = cfg["max_deg"]

        self.num_servos = 16
        self.servos_per_combo = 4
        self.num_combos = self.num_servos // self.servos_per_combo

        self.servo_vals = [float(self.min_deg)] * self.num_servos
        self.combo_index = 0

        self.bridge: ArduinoSerialBridge | None = None

        # --- Recording / playback state (like spirob_gui.py) ---
        self.recording: bool = False
        self.record_rows: list[list[float]] = []
        self.record_filepath: str | None = None
        self.record_frame_idx: int = 0
        self._playback_thread: threading.Thread | None = None

        # 60 Hz timer: used for recording samples each refresh
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.on_tick)
        self.timer.start(16)  # ~60 fps

        self._playback_stop = threading.Event()
        self._playback_thread = None
        self._playback_worker = None
        self._playback_dialog: PlaybackDialog | None = None
        self._playback_qthread: QtCore.QThread | None = None

        self._playback_active = False


        self._build_ui()

    # ---------------- UI ----------------

    def _build_ui(self):
        root = QtWidgets.QVBoxLayout(self)

        # Menu bar (like spirob_gui.py)
        self.menu_bar = QtWidgets.QMenuBar()
        file_menu = self.menu_bar.addMenu("File")
        act_play = file_menu.addAction("Play recording...")
        act_play.triggered.connect(self.on_play_recording)

        root.setMenuBar(self.menu_bar)


        # Connection row
        row = QtWidgets.QHBoxLayout()
        self.bt_connect = QtWidgets.QPushButton("Connect")
        self.bt_disconnect = QtWidgets.QPushButton("Disconnect")
        self.bt_disconnect.setEnabled(False)

        self.lb_status = QtWidgets.QLabel("Disconnected")
        self.lb_status.setStyleSheet("color: red;")
        row.addWidget(self.bt_connect)
        row.addWidget(self.bt_disconnect)
        row.addSpacing(20)
        row.addWidget(self.lb_status, 1)
        root.addLayout(row)

        # Combo controls
        combo_row = QtWidgets.QHBoxLayout()
        self.bt_prev_combo = QtWidgets.QPushButton("Prev combo")
        self.bt_next_combo = QtWidgets.QPushButton("Next combo")
        self.lb_combo = QtWidgets.QLabel("")
        self.lb_combo.setAlignment(QtCore.Qt.AlignCenter)

        combo_row.addWidget(self.bt_prev_combo)
        combo_row.addWidget(self.lb_combo, 1)
        combo_row.addWidget(self.bt_next_combo)
        root.addLayout(combo_row)

        # Sliders group
        grid = QtWidgets.QGridLayout()
        self.sliders: list[QtWidgets.QSlider] = []
        self.lb_servo_names: list[QtWidgets.QLabel] = []
        self.lb_servo_vals: list[QtWidgets.QLabel] = []

        for i in range(4):
            name = QtWidgets.QLabel("")
            name.setAlignment(QtCore.Qt.AlignCenter)
            self.lb_servo_names.append(name)
            grid.addWidget(name, 0, i)

            sld = QtWidgets.QSlider(QtCore.Qt.Vertical)
            sld.setRange(self.min_deg, self.max_deg)
            sld.setValue(self.min_deg)
            sld.setTickPosition(QtWidgets.QSlider.TicksLeft)
            sld.setTickInterval(max(1, (self.max_deg - self.min_deg) // 10))
            sld.valueChanged.connect(lambda val, idx=i: self.on_slider_changed(idx, val))
            self.sliders.append(sld)
            grid.addWidget(sld, 1, i)

            val_lab = QtWidgets.QLabel(str(self.min_deg))
            val_lab.setAlignment(QtCore.Qt.AlignCenter)
            self.lb_servo_vals.append(val_lab)
            grid.addWidget(val_lab, 2, i)

        root.addLayout(grid)
        root.addStretch(1)

        # Wire signals
        self.bt_connect.clicked.connect(self.on_connect)
        self.bt_disconnect.clicked.connect(self.on_disconnect)
        self.bt_next_combo.clicked.connect(self.on_next_combo)
        self.bt_prev_combo.clicked.connect(self.on_prev_combo)

        self._update_combo_labels()
        self._sync_sliders_from_state()

        rec_row = QtWidgets.QHBoxLayout()
        self.bt_record = QtWidgets.QPushButton("Record")
        self.bt_stop_record = QtWidgets.QPushButton("Stop")
        self.bt_stop_record.setEnabled(False)

        self.bt_record.clicked.connect(self.start_recording)
        self.bt_stop_record.clicked.connect(self.stop_recording)

        rec_row.addWidget(self.bt_record)
        rec_row.addWidget(self.bt_stop_record)
        root.addLayout(rec_row)


        # Make the window "hug" its contents vertically
        self.adjustSize()
        self.setMaximumHeight(self.sizeHint().height())

        # ---------- Recording (angles in degrees) ----------

    def _set_manual_controls_enabled(self, enabled: bool):
        self._set_sliders_enabled(enabled)
        self.bt_prev_combo.setEnabled(enabled)
        self.bt_next_combo.setEnabled(enabled)
        self.bt_record.setEnabled(enabled and not self.recording)
        self.bt_stop_record.setEnabled(enabled and self.recording)


    def start_recording(self):
        if self.recording:
            return

        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Choose CSV file for recording",
            filter="CSV files (*.csv);;All files (*.*)",
        )
        if not path:
            return

        self.record_filepath = path
        self.record_rows = []
        self.record_frame_idx = 0
        self.recording = True

        self.bt_record.setEnabled(False)
        self.bt_stop_record.setEnabled(True)

    def stop_recording(self):
        if not self.recording:
            return

        self.recording = False
        self.bt_record.setEnabled(True)
        self.bt_stop_record.setEnabled(False)

        if not self.record_filepath:
            self.record_rows = []
            return

        try:
            with open(self.record_filepath, "w", newline="") as f:
                w = csv.writer(f)
                # Save: time + combo_k + 4 angles (deg)
                w.writerow(["t", "k", "a0", "a1", "a2", "a3"])
                for row in self.record_rows:
                    w.writerow(row)

            QtWidgets.QMessageBox.information(
                self,
                "Recording",
                f"Saved {len(self.record_rows)} samples to\n{self.record_filepath}",
            )
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Recording error", str(e))
        finally:
            self.record_rows = []
            self.record_filepath = None

    def on_tick(self):
        """Called at ~60 Hz; record current state if enabled."""
        if not self.recording:
            return

        dt = self.timer.interval() / 1000.0
        t = self.record_frame_idx * dt
        k = self._combo_k()

        idxs = self._combo_servo_indices(self.combo_index)
        angles = [float(self.servo_vals[i]) for i in idxs]  # degrees

        self.record_rows.append([t, float(k), *angles])
        self.record_frame_idx += 1

    # ---------- Playback ----------

    def _request_stop_playback(self):
        # stops sending further commands
        self._playback_stop.set()

    def _request_disconnect_stop(self):
        self._playback_stop.set()
        self._playback_active = False

        # close dialog immediately
        if self._playback_dialog:
            self._playback_dialog.close()
            self._playback_dialog = None

        self.on_disconnect()


    def _playback_finished(self):
        # Re-enable manual controls (only if still connected)
        self._set_manual_controls_enabled(True)
        self._playback_active = False

        if self._playback_dialog:
            self._playback_dialog.close()
            self._playback_dialog = None

    def _playback_error(self, msg: str):
        self._set_manual_controls_enabled(True)
        self._playback_active = False


        if self._playback_dialog:
            self._playback_dialog.close()
            self._playback_dialog = None

        QtWidgets.QMessageBox.critical(self, "Playback error", msg)


    def on_play_recording(self):
        if not self.bridge:
            QtWidgets.QMessageBox.warning(self, "Play recording", "Connect to Arduino first.")
            return

        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Open recording CSV",
            filter="CSV files (*.csv);;All files (*.*)",
        )
        if not path:
            return

        repeats, ok = QtWidgets.QInputDialog.getInt(
            self, "Repeat playback", "How many times to repeat this recording?",
            1, 1, 1000
        )
        if not ok:
            return

        # Don’t start if already running
        if self._playback_active:
            QtWidgets.QMessageBox.warning(self, "Play recording", "Playback already running.")
            return


        # Load CSV into memory (k,a0,a1,a2,a3) as ints
        samples = []
        try:
            with open(path, newline="") as f:
                r = csv.reader(f)
                for row in r:
                    if not row:
                        continue
                    try:
                        vals = [float(x) for x in row]
                    except ValueError:
                        continue  # header
                    if len(vals) < 6:
                        continue
                    _, k, a0, a1, a2, a3 = vals[:6]
                    samples.append((
                        int(round(k)),
                        int(round(a0)),
                        int(round(a1)),
                        int(round(a2)),
                        int(round(a3)),
                    ))
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Play recording", f"Failed to read CSV:\n{e}")
            return

        if not samples:
            QtWidgets.QMessageBox.warning(self, "Play recording", "CSV contains no samples.")
            return

        # Stop flag + UI lock
        self._playback_stop.clear()
        self._set_manual_controls_enabled(False)

        total_steps = len(samples) * repeats
        dlg = PlaybackDialog(total_steps, parent=self)
        self._playback_dialog = dlg

        # Wire dialog buttons
        dlg.stop_requested.connect(self._request_stop_playback)
        dlg.disconnect_requested.connect(self._request_disconnect_stop)

        # Build worker in QThread (Qt-safe progress updates)
        qthread = QtCore.QThread(self)
        self._playback_qthread = qthread

        worker = PlaybackWorker(
            samples=samples,
            repeats=repeats,
            send_fn=self._send_raw,                 # uses your bridge queue
            set_combo_fn=self._set_combo_with_ack,  # UK + ack
            stop_flag=self._playback_stop,
            sleep_s=0.016,
        )
        self._playback_worker = worker
        worker.moveToThread(qthread)

        # Connect signals
        qthread.started.connect(worker.run)
        worker.progress.connect(dlg.update_progress)
        worker.finished.connect(self._playback_finished)
        worker.error.connect(self._playback_error)

        # Cleanup thread when done
        worker.finished.connect(qthread.quit)
        worker.error.connect(qthread.quit)
        qthread.finished.connect(worker.deleteLater)
        qthread.finished.connect(qthread.deleteLater)

        qthread.finished.connect(self._on_playback_thread_finished)


        # Start + show modal window
        self._playback_active = True
        qthread.start()
        dlg.show()


    def _on_playback_thread_finished(self):
        # Thread is gone (or about to be). Clear references so we don't call into deleted objects.
        self._playback_qthread = None
        self._playback_worker = None
        self._playback_active = False




    def _playback_worker(self, path: str, repeats: int):
        try:
            for _rep in range(repeats):
                last_k = None

                with open(path, newline="") as f:
                    r = csv.reader(f)
                    for row in r:
                        if not row:
                            continue
                        try:
                            vals = [float(x) for x in row]
                        except ValueError:
                            # header line
                            continue

                        if len(vals) < 6:
                            continue

                        if not self.bridge:
                            return

                        t, k, a0, a1, a2, a3 = vals[:6]
                        k = int(round(k))

                        # If combo changed, send UK and wait for ack
                        if last_k != k:
                            ok = self._set_combo_with_ack(k)
                            if not ok:
                                print(f"[Playback] Warning: combo change to k={k} timed out", flush=True)
                            last_k = k

                        # Send angles
                        self._send_raw(
                            f"!UA {int(round(a0))} {int(round(a1))} {int(round(a2))} {int(round(a3))}\r"
                        )

                        time.sleep(0.016)  # 60 Hz
        except Exception as e:
            print(f"[Playback error] {e}", flush=True)

    # ---------------- Helpers ----------------

    def _set_status(self, text: str, color: str):
        self.lb_status.setText(text)
        self.lb_status.setStyleSheet(f"color: {color};")

    def _set_sliders_enabled(self, enabled: bool):
        for s in self.sliders:
            s.setEnabled(enabled)

    def _combo_servo_indices(self, combo_index: int) -> list[int]:
        base = combo_index * self.servos_per_combo
        return [base + i for i in range(self.servos_per_combo)]

    def _combo_k(self) -> int:
        return self.combo_index * 4  # 0,4,8,12 

    def _update_combo_labels(self):
        idxs = self._combo_servo_indices(self.combo_index)
        servo_numbers = [i + 1 for i in idxs]
        self.lb_combo.setText(f"Combo {self.combo_index + 1} (Servos {servo_numbers[0]}–{servo_numbers[-1]})")
        for i, num in enumerate(servo_numbers):
            self.lb_servo_names[i].setText(f"Servo {num}")

    def _sync_sliders_from_state(self):
        idxs = self._combo_servo_indices(self.combo_index)
        for local_i, global_i in enumerate(idxs):
            v = int(round(self.servo_vals[global_i]))
            self.sliders[local_i].blockSignals(True)
            self.sliders[local_i].setValue(v)
            self.sliders[local_i].blockSignals(False)
            self.lb_servo_vals[local_i].setText(str(v))

    def _drain_bridge_queue(self):
        """Drop any pending queued writes to avoid mixing UK with UA."""
        if not self.bridge:
            return
        try:
            while True:
                _ = self.bridge.q.get_nowait()
        except queue.Empty:
            pass

    def _send_raw(self, cmd: str):
        """
        Send a raw command through the SAME writer queue used by ArduinoSerialBridge.
        This keeps one single serial connection/thread.
        """
        if not self.bridge:
            return
        if not cmd.endswith("\r"):
            cmd += "\r"
        try:
            self.bridge.q.put_nowait(cmd)
        except queue.Full:
            # best-effort: drop one and retry
            try:
                _ = self.bridge.q.get_nowait()
                self.bridge.q.put_nowait(cmd)
            except queue.Empty:
                pass

    # ---------------- Serial protocol ----------------

    def _set_combo_with_ack(self, k: int, tries: int = 150) -> bool:
        """
        Replicates your Tk behavior for changing combos:
          read a line; if not int -> send '!UK k\\r'
          if int and equals k -> success
        """
        if not self.bridge or self.bridge._ser is None:
            return False

        ser = self.bridge._ser
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        # stop other traffic while switching
        self._drain_bridge_queue()

        for _ in range(tries):
            try:
                line = ser.readline().decode(errors="ignore").strip()
            except Exception:
                line = ""

            try:
                dif = int(line)
            except Exception:
                # not an int yet -> request combo
                try:
                    ser.write(f"!UK {k}\r".encode("ascii"))
                    ser.flush()
                except Exception:
                    return False
            else:
                if dif == k:
                    return True

        return False

    def _send_current_combo_angles(self):
        """Send '!UA a b c d\\r' using same format as Tk script."""
        if not self.bridge:
            return

        idxs = self._combo_servo_indices(self.combo_index)
        angles = [int(round(self.servo_vals[i])) for i in idxs]

        cmd = "!UA " + " ".join(str(a) for a in angles) + "\r"
        self._send_raw(cmd)

    # ---------------- Slots ----------------

    def on_connect(self):
        if self.bridge is not None:
            self._set_status("Already connected", "orange")
            return

        port = self.cfg["port"]
        baud = self.cfg["baud"]

        try:
            # Use your bridge: it does '!connect' handshake inside start()
            self.bridge = ArduinoSerialBridge(
                port=port,
                baud=baud,
                min_deg=float(self.min_deg),
                max_deg=float(self.max_deg),
                pulley_diameter_mm=15.0,  # not used if we send raw UA, but fine
            )
            self.bridge.start()
        except Exception as e:
            self.bridge = None
            self._set_status(f"Connect failed: {e}", "red")
            return

        self._set_status(f"Connected to {port} @ {baud}", "green")
        self.bt_connect.setEnabled(False)
        self.bt_disconnect.setEnabled(True)

        # Apply current combo (UK) and then send angles once
        self._apply_combo_to_board()
        self._send_current_combo_angles()

    def on_disconnect(self):
        self._playback_stop.set()
        self._playback_active = False

        if self.bridge:
            try:
                self.bridge.stop()
            except Exception:
                pass
        self.bridge = None
        self.bt_connect.setEnabled(True)
        self.bt_disconnect.setEnabled(False)
        self._set_status("Disconnected", "red")

    def on_next_combo(self):
        if self.combo_index >= self.num_combos - 1:
            self._set_status("Already at last combo", "orange")
            return
        self.combo_index += 1
        self._update_combo_labels()
        self._sync_sliders_from_state()
        self._apply_combo_to_board()
        self._send_current_combo_angles()

    def on_prev_combo(self):
        if self.combo_index <= 0:
            self._set_status("Already at first combo", "orange")
            return
        self.combo_index -= 1
        self._update_combo_labels()
        self._sync_sliders_from_state()
        self._apply_combo_to_board()
        self._send_current_combo_angles()

    def _apply_combo_to_board(self):
        """Switch group via '!UK k\\r' and wait for Arduino to reply k. :contentReference[oaicite:10]{index=10}"""
        if not self.bridge:
            return

        k = self._combo_k()
        self._set_status("Changing motor combination...", "orange")
        self._set_sliders_enabled(False)

        ok = self._set_combo_with_ack(k)

        if ok:
            self._set_status(f"Combination set (k={k})", "green")
        else:
            # Still usable; you can keep sending UA, but it's likely on wrong group.
            self._set_status(f"Combo change timed out (k={k})", "red")

        self._set_sliders_enabled(True)

    def on_slider_changed(self, local_idx: int, value: int):
        idxs = self._combo_servo_indices(self.combo_index)
        global_idx = idxs[local_idx]

        self.servo_vals[global_idx] = float(value)
        self.lb_servo_vals[local_idx].setText(str(value))

        # Send immediately
        self._send_current_combo_angles()


def main():
    app = QtWidgets.QApplication(sys.argv)

    icon_path = "Python/StepperController32.ico"

    app.setWindowIcon(QtGui.QIcon(icon_path))


    setup = SetupDialog()
    if setup.exec() != QtWidgets.QDialog.Accepted:
        return

    cfg = setup.get_config()
    w = ServoControlWindow(cfg)
    w.resize(720, 480)
    w.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
