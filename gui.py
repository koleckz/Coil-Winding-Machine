#!/usr/bin/env python3
"""Desktop control panel for the two-motor winding machine."""

from __future__ import annotations

import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

sys.path.insert(0, str(Path(__file__).resolve().parent))

from machine import (Event, Machine, Status, DIR_RIGHT, DIR_LEFT,
                     MIN_SPEED, MAX_SPEED, MANUAL_SPEED_0, MANUAL_SPEED_1)
from relay import Arduino

# states in which the operating mode must not be switched
BUSY = (Status.SEMI_MANUAL, Status.MANUAL, Status.RUNNING, Status.STOPPING)


class SimulatedArduino:
    """Small hardware substitute used to exercise the GUI without an Arduino."""

    def __init__(self) -> None:
        self.commands: list[str] = []
        self._running = [False, False]
        self._speed = [0.0, 0.0]
        self._dir = [1, 1]
        self._steps = [0.0, 0.0]
        self._t = [time.monotonic()] * 2
        self._remaining: list[float | None] = [None, None]

    def _tick(self, n: int) -> None:
        now = time.monotonic()
        if self._running[n]:
            dist = self._speed[n] * (now - self._t[n])
            if self._remaining[n] is not None:
                dist = min(dist, self._remaining[n])
                self._remaining[n] -= dist
                if self._remaining[n] <= 0:
                    self._remaining[n] = None
                    self._running[n] = False
            self._steps[n] += (1 if self._dir[n] else -1) * dist
        self._t[n] = now

    def _send(self, command: str, extra_lines: int = 0) -> list[str]:
        self.commands.append(command)
        return ["0"] * extra_lines

    def run(self, n: int, on: bool) -> None:
        self._tick(n)
        self._running[n] = bool(on)
        self._remaining[n] = None
        self._send(f"RUN {n},{1 if on else 0}")

    def direction(self, n: int, direction: int) -> None:
        self._tick(n)
        self._dir[n] = direction
        self._send(f"DIR {n},{direction}")

    def speed(self, n: int, speed: int) -> None:
        self._tick(n)
        self._speed[n] = float(speed)
        self._send(f"SPD {n},{int(speed)}")

    def accel(self, n: int, acceleration: int) -> None:
        self._send(f"ACC {n},{int(acceleration)}")

    def steps(self, n: int, steps: int) -> None:
        self._tick(n)
        if steps > 0:
            self._remaining[n] = float(steps)
            self._running[n] = True
        else:
            self._remaining[n] = None
        self._send(f"STP {n},{int(steps)}")

    def stop_all(self, motors=(0, 1), strict: bool = False) -> None:
        for motor in motors:
            self.run(motor, False)

    def halt(self) -> None:
        self.stop_all()

    def total_steps(self, n: int) -> int:
        self._tick(n)
        return int(self._steps[n])

    def ping(self) -> None:
        pass

    def end_switch(self) -> tuple[bool, bool, int]:
        return False, False, 0

    def close(self) -> None:
        pass


class WinderGUI(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Winding Machine Control")
        self.configure(bg="#121212")

        # Fullscreen setup
        self.is_fullscreen = True
        self.attributes("-fullscreen", self.is_fullscreen)
        self.bind("<Escape>", self.toggle_fullscreen)

        self.machine: Machine | None = None
        self.backend = None
        
        # Debounce variables
        self.moving_button: str | None = None
        self._release_timer: str | None = None
        self._shift_last = 0.0
        
        # UI Variables
        self.port = tk.StringVar(value="/dev/ttyUSB0")
        self.simulation = tk.BooleanVar(value=True)
        self.status_text = tk.StringVar(value="DISCONNECTED")
        self.connection_text = tk.StringVar(value="No controller connected")
        
        self.coil_diameter = tk.StringVar(value="20.0")
        self.coil_length = tk.StringVar(value="50.0")
        self.wire_diameter = tk.StringVar(value="0.5")
        self.wire_length = tk.StringVar(value="1000.0")
        self.jog_size = tk.StringVar(value="2.0")
        
        self.turns_text = tk.StringVar(value="— / —")
        self.remaining_text = tk.StringVar(value="—")
        self.total_time_text = tk.StringVar(value="—")
        self.progress_value = tk.DoubleVar(value=0.0)

        self.manual_mode = tk.BooleanVar(value=False)
        self.mode_text = tk.StringVar()
        self.manual_speed_0 = tk.StringVar(value=str(MANUAL_SPEED_0))
        self.manual_speed_1 = tk.StringVar(value=str(MANUAL_SPEED_1))
        self.speed_hint = tk.StringVar(value=f"Allowed {MIN_SPEED}–{MAX_SPEED} steps/s")

        self.wind_dir_text = tk.StringVar(value="RIGHT")
        self.traverse_dir_text = tk.StringVar(value="—")
        self.end_switch_text = tk.StringVar(value="—")
        self._last_end_hits: int | None = None
        self.auto_text = tk.StringVar(value="▶ START WINDING  (Shift)")
        self.log_text: tk.Text | None = None

        self._configure_style()
        self._build_layout()
        self._update_mode_widgets()
        self.manual_speed_0.trace_add("write", self._push_manual_speeds)
        self.manual_speed_1.trace_add("write", self._push_manual_speeds)
        
        # Keyboard bindings (ignored while typing in an entry field, except the emergency stop)
        # Left/Right: wind/unwind (semi-manual) or spindle only (manual)
        self.bind("<Left>", lambda e: self._key_hold("KEY_LEFT", self._wind_event(False)))
        self.bind("<KeyRelease-Left>", lambda e: self._end_hold("KEY_LEFT"))
        self.bind("<Right>", lambda e: self._key_hold("KEY_RIGHT", self._wind_event(True)))
        self.bind("<KeyRelease-Right>", lambda e: self._end_hold("KEY_RIGHT"))

        # A/D: traverse only, manual mode
        for key, name, event in (("a", "KEY_A", Event.TRAVERSE_LEFT), ("A", "KEY_A", Event.TRAVERSE_LEFT),
                                 ("d", "KEY_D", Event.TRAVERSE_RIGHT), ("D", "KEY_D", Event.TRAVERSE_RIGHT)):
            self.bind(f"<KeyPress-{key}>", lambda e, n=name, ev=event: self._key_hold(n, self._traverse_event(ev)))
            self.bind(f"<KeyRelease-{key}>", lambda e, n=name: self._end_hold(n))

        # Up/Down: spindle direction (motor 0 only)
        self.bind("<Up>", lambda e: None if self._typing() else self._set_wind_dir(DIR_RIGHT))
        self.bind("<Down>", lambda e: None if self._typing() else self._set_wind_dir(DIR_LEFT))

        # Shift: start automatic winding / stop it when running
        self.bind("<KeyPress-Shift_L>", self._handle_shift)
        self.bind("<KeyPress-Shift_R>", self._handle_shift)
        
        # Spacebar for Emergency Stop
        self.bind("<space>", self._handle_space)

        # clicking outside an entry field (or Enter) gives the keyboard back to the machine
        self.bind_all("<Button-1>", self._click_focus, add="+")
        self.bind_all("<Return>", lambda e: self.focus_set(), add="+")

        self.protocol("WM_DELETE_WINDOW", self._close)
        self.after(250, self._poll)

    def _handle_space(self, event):
        self._emergency_stop()
        return "break"

    def _handle_shift(self, event):
        if self._typing():
            return
        now = time.monotonic()
        repeated = now - self._shift_last < 0.4      # key auto-repeat or an accidental double tap
        self._shift_last = now
        if not repeated:
            self._event(Event.TOGGLE_AUTO)

    def _typing(self) -> bool:
        try:
            return isinstance(self.focus_get(), (tk.Entry, ttk.Entry))
        except (KeyError, tk.TclError):
            return False

    def _click_focus(self, event) -> None:
        if not isinstance(event.widget, (tk.Entry, ttk.Entry)):
            self.focus_set()

    def toggle_fullscreen(self, event=None):
        self.is_fullscreen = not self.is_fullscreen
        self.attributes("-fullscreen", self.is_fullscreen)
        return "break"

    def _poll(self) -> None:
        self._refresh_status()
        self.after(250, self._poll)

    def _configure_style(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        
        bg_main = "#121212"
        bg_panel = "#1e1e1e"
        fg_text = "#ffffff"
        fg_muted = "#a0a0a0"
        
        style.configure("TFrame", background=bg_main)
        style.configure("Panel.TFrame", background=bg_panel)
        style.configure("TLabel", background=bg_main, foreground=fg_text, font=("Helvetica", 14))
        style.configure("Panel.TLabel", background=bg_panel, foreground=fg_text, font=("Helvetica", 14))
        style.configure("Hint.TLabel", background=bg_panel, foreground=fg_muted, font=("Helvetica", 11))
        style.configure("Title.TLabel", background=bg_main, foreground=fg_text, font=("Helvetica", 28, "bold"))
        style.configure("Status.TLabel", background=bg_main, foreground="#4caf50", font=("Helvetica", 24, "bold"))
        style.configure("Header.TLabel", background=bg_panel, foreground=fg_muted, font=("Helvetica", 12, "bold"))
        
        style.configure("TButton", padding=(15, 10), font=("Helvetica", 14, "bold"))
        style.configure("Accent.TButton", background="#ff9800", foreground="#000000")
        style.map("Accent.TButton", background=[("active", "#ffb74d")])
        style.configure("Danger.TButton", background="#f44336", foreground="#ffffff")
        style.map("Danger.TButton", background=[("active", "#e57373")])
        
        style.configure("TEntry", fieldbackground="#2c2c2c", foreground="#ffffff", font=("Helvetica", 14), padding=5)
        style.configure("Value.TLabel", background=bg_panel, foreground=fg_text, font=("Helvetica", 26, "bold"))
        style.configure("Horizontal.TProgressbar", troughcolor="#2c2c2c", background="#ff9800", bordercolor=bg_panel, lightcolor="#ff9800", darkcolor="#ff9800")
        style.configure("TCheckbutton", background=bg_panel, foreground=fg_text, font=("Helvetica", 14, "bold"))

    def _build_layout(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=0)
        self.rowconfigure(1, weight=1)

        # 1. HEADER
        header = ttk.Frame(self)
        header.grid(row=0, column=0, sticky="ew", padx=30, pady=20)
        header.columnconfigure(1, weight=1)
        
        ttk.Label(header, text="WINDING MACHINE", style="Title.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(header, textvariable=self.status_text, style="Status.TLabel").grid(row=0, column=1, sticky="e", padx=20)
        
        ttk.Button(header, text="Exit Fullscreen", command=self.toggle_fullscreen).grid(row=0, column=2, sticky="e")

        # 2. MAIN CONTENT AREA
        main = ttk.Frame(self)
        main.grid(row=1, column=0, sticky="nsew", padx=30, pady=(0, 30))
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=0)
        main.rowconfigure(1, weight=0)
        main.rowconfigure(2, weight=1)

        # -- Connection Bar --
        conn_frame = ttk.Frame(main, style="Panel.TFrame", padding=15)
        conn_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 20))
        for i in range(5): conn_frame.columnconfigure(i, weight=1)
        
        ttk.Label(conn_frame, text="Serial Port:", style="Panel.TLabel").grid(row=0, column=0, sticky="e", padx=10)
        ttk.Entry(conn_frame, textvariable=self.port, width=15).grid(row=0, column=1, sticky="w", padx=10)
        ttk.Checkbutton(conn_frame, text="Simulation Mode", variable=self.simulation).grid(row=0, column=2, padx=10)
        ttk.Button(conn_frame, text="CONNECT", style="Accent.TButton", command=self._connect).grid(row=0, column=3, padx=10)
        ttk.Label(conn_frame, textvariable=self.connection_text, style="Panel.TLabel").grid(row=0, column=4, sticky="w", padx=10)

        # -- Progress Bar --
        prog_frame = ttk.Frame(main, style="Panel.TFrame", padding=15)
        prog_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 20))
        for i in range(3): prog_frame.columnconfigure(i, weight=1)

        for col, (title, var) in enumerate([("TURNS DONE / NEEDED", self.turns_text),
                                            ("TIME REMAINING", self.remaining_text),
                                            ("TOTAL WINDING TIME", self.total_time_text)]):
            ttk.Label(prog_frame, text=title, style="Header.TLabel").grid(row=0, column=col, sticky="w", padx=10)
            ttk.Label(prog_frame, textvariable=var, style="Value.TLabel").grid(row=1, column=col, sticky="w", padx=10)
        ttk.Progressbar(prog_frame, variable=self.progress_value, maximum=100).grid(row=2, column=0, columnspan=3, sticky="ew", padx=10, pady=(10, 0))

        # -- Left Panel (Settings & hand control) --
        left_panel = ttk.Frame(main, style="Panel.TFrame", padding=25)
        left_panel.grid(row=2, column=0, sticky="nsew", padx=(0, 10))
        left_panel.columnconfigure(1, weight=1)

        ttk.Label(left_panel, text="WINDING PARAMETERS", style="Header.TLabel").grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 10))
        
        fields = [
            ("Coil Diameter (mm):", self.coil_diameter),
            ("Coil Length (mm):", self.coil_length),
            ("Wire Diameter (mm):", self.wire_diameter),
            ("Wire Length (mm):", self.wire_length),
            ("Jog Distance (mm along coil):", self.jog_size)
        ]
        
        for row, (label, variable) in enumerate(fields, start=1):
            ttk.Label(left_panel, text=label, style="Panel.TLabel").grid(row=row, column=0, sticky="w", pady=5)
            ttk.Entry(left_panel, textvariable=variable, width=12).grid(row=row, column=1, sticky="e", pady=5)
            
        ttk.Button(left_panel, text="APPLY PARAMETERS", command=self._apply_parameters).grid(row=6, column=0, columnspan=2, sticky="ew", pady=(15, 15))

        # Hand control (semi-manual / manual)
        ctrl = ttk.Frame(left_panel, style="Panel.TFrame")
        ctrl.grid(row=7, column=0, columnspan=2, sticky="ew")
        ctrl.columnconfigure(0, weight=1)
        ctrl.columnconfigure(1, weight=1)

        ttk.Label(ctrl, text="HAND CONTROL  (hold arrows ← / →)", style="Header.TLabel").grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))
        ttk.Checkbutton(ctrl, text="Manual mode", variable=self.manual_mode,
                        command=self._on_mode_change).grid(row=1, column=0, sticky="w")
        ttk.Label(ctrl, textvariable=self.mode_text, style="Panel.TLabel").grid(row=1, column=1, sticky="w")

        self.wind_back_btn = self._hold_button(ctrl, "", lambda: self._wind_event(False), 2, 0)
        self.wind_fwd_btn = self._hold_button(ctrl, "", lambda: self._wind_event(True), 2, 1)
        self.jog_back_btn = ttk.Button(ctrl, command=lambda: self._event(self._jog_event(False)))
        self.jog_back_btn.grid(row=3, column=0, sticky="ew", padx=(0, 5), pady=(8, 0))
        self.jog_fwd_btn = ttk.Button(ctrl, command=lambda: self._event(self._jog_event(True)))
        self.jog_fwd_btn.grid(row=3, column=1, sticky="ew", padx=(5, 0), pady=(8, 0))

        ttk.Label(ctrl, text="MANUAL MODE  (traverse: hold A / D)", style="Header.TLabel").grid(row=4, column=0, columnspan=2, sticky="w", pady=(18, 8))
        speeds = ttk.Frame(ctrl, style="Panel.TFrame")
        speeds.grid(row=5, column=0, columnspan=2, sticky="ew")
        speeds.columnconfigure(1, weight=1)
        speeds.columnconfigure(3, weight=1)
        ttk.Label(speeds, text="Spindle M0 (steps/s):", style="Panel.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Entry(speeds, textvariable=self.manual_speed_0, width=6).grid(row=0, column=1, sticky="w", padx=(8, 15))
        ttk.Label(speeds, text="Traverse M1 (steps/s):", style="Panel.TLabel").grid(row=0, column=2, sticky="w")
        ttk.Entry(speeds, textvariable=self.manual_speed_1, width=6).grid(row=0, column=3, sticky="w", padx=(8, 0))
        ttk.Label(ctrl, textvariable=self.speed_hint, style="Hint.TLabel").grid(row=6, column=0, columnspan=2, sticky="w", pady=(4, 0))

        self.trav_left_btn = self._hold_button(ctrl, "<< Traverse (A)", lambda: self._traverse_event(Event.TRAVERSE_LEFT), 7, 0)
        self.trav_right_btn = self._hold_button(ctrl, "Traverse (D) >>", lambda: self._traverse_event(Event.TRAVERSE_RIGHT), 7, 1)

        # -- Right Panel (Machine & Log) --
        right_panel = ttk.Frame(main, style="Panel.TFrame", padding=25)
        right_panel.grid(row=2, column=1, sticky="nsew", padx=(10, 0))
        right_panel.columnconfigure(0, weight=1)
        right_panel.rowconfigure(9, weight=1)

        ttk.Label(right_panel, text="MACHINE OPERATION", style="Header.TLabel").grid(row=0, column=0, sticky="w", pady=(0, 5))
        
        ttk.Label(right_panel, text="Spindle direction M0 (Up / Down arrows):", style="Panel.TLabel").grid(row=1, column=0, sticky="w", pady=(10, 0))
        ttk.Label(right_panel, textvariable=self.wind_dir_text, style="Status.TLabel", font=("Helvetica", 16, "bold")).grid(row=2, column=0, sticky="w", pady=(0, 8))
        # traverse direction and end switch side by side (rows 3 + 4)
        info = ttk.Frame(right_panel, style="Panel.TFrame")
        info.grid(row=3, column=0, rowspan=2, sticky="ew", pady=(0, 15))
        info.columnconfigure(0, weight=1)
        info.columnconfigure(1, weight=1)
        ttk.Label(info, text="Traverse direction M1:", style="Panel.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(info, textvariable=self.traverse_dir_text, style="Status.TLabel", font=("Helvetica", 16, "bold")).grid(row=1, column=0, sticky="w")
        ttk.Label(info, text="End switches (pin 9 / pin 10):", style="Panel.TLabel").grid(row=0, column=1, sticky="w")
        ttk.Label(info, textvariable=self.end_switch_text, style="Status.TLabel", font=("Helvetica", 16, "bold")).grid(row=1, column=1, sticky="w")
        
        ttk.Button(right_panel, text="INITIALIZE / RECOVER", command=self._initialize).grid(row=5, column=0, sticky="ew", pady=10)
        ttk.Button(right_panel, textvariable=self.auto_text, style="Accent.TButton",
                   command=lambda: self._event(Event.TOGGLE_AUTO)).grid(row=6, column=0, sticky="ew", pady=10)
        ttk.Button(right_panel, text="EMERGENCY STOP (Spacebar)", style="Danger.TButton", command=self._emergency_stop).grid(row=7, column=0, sticky="ew", pady=(25, 15))
        
        ttk.Label(right_panel, text="EVENT LOG", style="Header.TLabel").grid(row=8, column=0, sticky="sw", pady=(10, 5))
        
        self.log_text = tk.Text(right_panel, height=6, bg="#121212", fg="#808080", insertbackground="white", 
                                relief="flat", state="disabled", font=("Courier", 12))
        self.log_text.grid(row=9, column=0, sticky="nsew")

    # --- modes ---
    def _wind_event(self, forward: bool) -> Event:
        if self.manual_mode.get():
            return Event.MANUAL_WIND_FWD if forward else Event.MANUAL_WIND_BACK
        return Event.SEMI_WIND_FWD if forward else Event.SEMI_WIND_BACK

    def _jog_event(self, forward: bool) -> Event:
        if self.manual_mode.get():
            return Event.MANUAL_JOG_FWD if forward else Event.MANUAL_JOG_BACK
        return Event.SEMI_JOG_FWD if forward else Event.SEMI_JOG_BACK

    def _traverse_event(self, event: Event) -> Event | None:
        return event if self.manual_mode.get() else None    # traverse alone only in manual mode

    def _on_mode_change(self) -> None:
        if (self.machine and self.machine.status in BUSY) or self.moving_button is not None:
            self.manual_mode.set(not self.manual_mode.get())     # undo the click
            self._log("Stop the machine before switching mode.")
            return
        self._update_mode_widgets()
        self._log(f"Mode: {self.mode_text.get()}")

    def _update_mode_widgets(self) -> None:
        manual = self.manual_mode.get()
        self.mode_text.set("MANUAL – spindle only" if manual else "SEMI-MANUAL – spindle + traverse")
        self.wind_back_btn.configure(text="<< Spindle Back (Hold)" if manual else "<< Unwind (Hold)")
        self.wind_fwd_btn.configure(text="Spindle Fwd (Hold) >>" if manual else "Wind (Hold) >>")
        self.jog_back_btn.configure(text="|< Spindle Jog Back" if manual else "|< Unwind Jog")
        self.jog_fwd_btn.configure(text="Spindle Jog Fwd >|" if manual else "Wind Jog >|")
        for button in (self.trav_left_btn, self.trav_right_btn):
            button.state(["!disabled"] if manual else ["disabled"])

    def _push_manual_speeds(self, *_args) -> None:
        try:
            speeds = [int(v.get()) for v in (self.manual_speed_0, self.manual_speed_1)]
            if not all(MIN_SPEED <= s <= MAX_SPEED for s in speeds):
                raise ValueError
        except ValueError:
            self.speed_hint.set(f"Invalid speed – allowed {MIN_SPEED}–{MAX_SPEED}, last valid value is used")
            return
        self.speed_hint.set(f"Allowed {MIN_SPEED}–{MAX_SPEED} steps/s")
        if self.machine:
            self.machine.set_manual_speeds(*speeds)

    def _set_wind_dir(self, direction: int) -> None:
        direction_str = "RIGHT" if direction == DIR_RIGHT else "LEFT"
        self.wind_dir_text.set(direction_str)
        if self.machine:
            self.machine.wind_dir = direction
            self._log(f"Spindle direction changed: {direction_str}")

    # --- hold buttons / keys ---
    def _hold_button(self, parent, label: str, make_event, row: int, column: int) -> ttk.Button:
        button = ttk.Button(parent, text=label)
        button.grid(row=row, column=column, sticky="ew", padx=(0, 5) if column == 0 else (5, 0), pady=(8, 0))
        name = str(button)
        button.bind("<ButtonPress-1>", lambda _e: self._begin_hold(name, make_event()))
        button.bind("<ButtonRelease-1>", lambda _e: self._end_hold(name))
        return button

    def _key_hold(self, button: str, event: Event | None) -> None:
        if not self._typing():
            self._begin_hold(button, event)

    def _begin_hold(self, button: str, event: Event | None) -> None:
        if event is None:
            return
        if self.moving_button == button:
            # key auto-repeat sends release+press pairs: keep holding
            if self._release_timer is not None:
                self.after_cancel(self._release_timer)
                self._release_timer = None
            return
        if self.moving_button is not None:
            return                       # another key/button is already held
        self.moving_button = button
        self._event(event)

    def _end_hold(self, button: str) -> None:
        if self.moving_button == button:
            if self._release_timer is not None:
                self.after_cancel(self._release_timer)
            self._release_timer = self.after(50, lambda: self._commit_release(button))

    def _commit_release(self, button: str) -> None:
        self._release_timer = None
        if self.moving_button == button:
            self.moving_button = None
            self._event(Event.HOLD_RELEASED)

    # --- connection ---
    def _connect(self) -> None:
        if self.backend is not None and hasattr(self.backend, "close"):
            try:
                self.backend.close()
            except Exception:
                pass
            self.backend = None
            self.machine = None
            
        try:
            self.backend = SimulatedArduino() if self.simulation.get() else Arduino(self.port.get().strip())
            self.machine = Machine(self.backend)
            self.machine.wind_dir = DIR_RIGHT if self.wind_dir_text.get() == "RIGHT" else DIR_LEFT
            self._push_manual_speeds()
            self._last_end_hits = None
            self.backend.ping()          # fails right here if the Arduino does not answer
            
        except Exception as error:
            if self.backend is not None:
                try:
                    self.backend.close()
                except Exception:
                    pass
            self.backend = None
            self.machine = None
            self.connection_text.set("Connection failed")
            self._log(f"Connection error: {error}")
            messagebox.showerror("Connection Error", str(error))
            return
            
        mode = "Simulation" if self.simulation.get() else self.port.get().strip()
        self.connection_text.set(f"Connected: {mode}")
        self._log(f"Connected to {mode}")
        self._refresh_status()

    def _initialize(self) -> None:
        if not self.machine:
            self._log("Connect a controller first.")
            return
            
        # ÚPLNÁ OBNOVA ZO STAVU KILL
        if self.machine.status == Status.KILL:
            self._log("Recovering from EMERGENCY STOP...")
            old = self.machine
            self.machine = Machine(self.backend) # Vytvorí čistý stavový automat
            
            # Synchronizácia UI premenných späť do stroja
            self.machine.wind_dir = DIR_RIGHT if self.wind_dir_text.get() == "RIGHT" else DIR_LEFT
            self.machine.traverse_dir = old.traverse_dir   # traverz pokračuje rovnakým smerom
            self._push_manual_speeds()
            
            try:
                self._event(Event.RESET_DONE)
                self._event(Event.INIT_DONE)
                values = [float(v.get()) for v in (self.coil_diameter, self.coil_length, self.wire_length, self.wire_diameter, self.jog_size)]
                self.machine.set_parameters(*values)
                self._log("Parameters auto-restored.")
            except Exception:
                pass
            return

        self._event(Event.RESET_DONE)
        self._event(Event.INIT_DONE)

    def _apply_parameters(self) -> None:
        if not self.machine:
            self._log("Connect a controller first.")
            return
        try:
            values = [float(value.get()) for value in (self.coil_diameter, self.coil_length, self.wire_length, self.wire_diameter, self.jog_size)]
            self.machine.set_parameters(*values)
            self._log("Parameters applied successfully.")
        except (ValueError, RuntimeError) as error:
            messagebox.showwarning("Invalid Parameters", str(error))
        self._refresh_status()

    def _event(self, event: Event) -> None:
        if not self.machine:
            self._log("Connect a controller first.")
            return
        previous = self.machine.status
        try:
            self.machine.handle_event(event)
        except ValueError as error:
            self._log(f"[{event.name}] {error}")
        
        if self.machine.status != previous or event == Event.EMERGENCY_STOP:
            self._log(f"[{event.name}] -> {self.machine.status.name}")
        self._refresh_status()

    def _emergency_stop(self) -> None:
        if self.machine:
            self._event(Event.EMERGENCY_STOP)

    @staticmethod
    def _fmt_time(seconds: float) -> str:
        seconds = int(round(seconds))
        return f"{seconds // 3600}:{seconds % 3600 // 60:02d}:{seconds % 60:02d}"

    def _refresh_progress(self) -> None:
        m = self.machine
        if not m or m.turns_needed <= 0:
            self.turns_text.set("— / —")
            self.remaining_text.set("—")
            self.total_time_text.set("—")
            self.progress_value.set(0.0)
            return
        self.turns_text.set(f"{m.turns_done:.1f} / {m.turns_needed:.1f}")
        self.remaining_text.set(self._fmt_time(m.time_remaining))
        self.total_time_text.set(self._fmt_time(m.time_total))
        self.progress_value.set(min(100.0, 100.0 * m.turns_done / m.turns_needed))

    def _refresh_status(self) -> None:
        m = self.machine
        if not m:
            return
        m.poll()
        self.status_text.set(m.status.name)

        if m.status == Status.RUNNING:
            self.auto_text.set("⏹ STOP WINDING  (Shift)")
        elif m.status == Status.STOPPING:
            self.auto_text.set("… STOPPING")
        else:
            self.auto_text.set("▶ START WINDING  (Shift)")

        self.traverse_dir_text.set("RIGHT" if m.traverse_dir > 0 else "LEFT")
        self._refresh_end_switch(m)

        self._refresh_progress()

    def _refresh_end_switch(self, m: Machine) -> None:
        state = m.read_end_switch()
        if state is None:
            self.end_switch_text.set("?")
            return
        closed_9, closed_10, hits = state
        label = lambda closed: "PRESSED" if closed else "open"
        self.end_switch_text.set(f"{label(closed_9)} / {label(closed_10)}  ·  hits {hits}")
        if self._last_end_hits is not None and hits > self._last_end_hits:
            self._log("End switch: spindle stopped, traverse backed off and stopped.")
        self._last_end_hits = hits

    def _log(self, message: str) -> None:
        if self.log_text is None:
            return
        self.log_text.configure(state="normal")
        self.log_text.insert("end", message + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _close(self) -> None:
        if self.machine:
            self.machine.kill()
        if self.backend and hasattr(self.backend, "close"):
            self.backend.close()
        self.destroy()


if __name__ == "__main__":
    app = WinderGUI()
    app.mainloop()
