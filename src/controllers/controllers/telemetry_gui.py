"""
Live Graphical Telemetry Dashboard Window for F1Tenth Autonomous Racing.

Launches a dedicated dark-mode Tkinter GUI window in a background daemon thread
alongside the ROS 2 terminal and RViz2, displaying:
  - Live Digital Speedometer (m/s & km/h) + Animated Speed Bar
  - Live Lap Counter (Lap X / 10) & Current Lap Stopwatch (s)
  - Best Lap Time, Cumulative Time Saved vs. Lap 1, Active Pace Trim (T0..T9)
  - Current Lap Distance (m) & Minimum LiDAR Wall Clearance (m)
  - Full Lap-by-Lap Telemetry Table (Lap #, Trim, Time, Gain vs Lap 1, Delta, Distance, Peak Speed, Min Clearance)
"""

import os
import threading


class RaceTelemetryGUI:
    def __init__(self, title="F1Tenth Sakhir GP — Live Telemetry & Lap Timer Window", target_laps=10):
        self.title = title
        self.target_laps = target_laps
        self._lock = threading.Lock()
        self._state = {
            "status": "RACING",
            "trim": "T0",
            "current_lap": 0,
            "target_laps": target_laps,
            "current_lap_time": 0.0,
            "best_lap_time": None,
            "cum_saved": 0.0,
            "speed_mps": 0.0,
            "peak_speed_mps": 0.0,
            "steer_deg": 0.0,
            "lap_dist_m": 0.0,
            "total_dist_m": 0.0,
            "min_clearance_m": None,
            "collisions": 0,
            "lap_rows": [],
            "laps_version": 0,
        }
        self._shown_laps_version = -1
        self._enabled = bool(os.environ.get("DISPLAY") or os.name == "nt")
        self._thread = None
        if self._enabled:
            self._thread = threading.Thread(target=self._run_gui, daemon=True)
            self._thread.start()

    def update_live(
        self,
        *,
        status="RACING",
        trim="T0",
        current_lap=0,
        current_lap_time=0.0,
        best_lap_time=None,
        cum_saved=0.0,
        speed_mps=0.0,
        peak_speed_mps=0.0,
        steer_rad=0.0,
        lap_dist_m=0.0,
        total_dist_m=0.0,
        min_clearance_m=None,
        collisions=0,
    ):
        if not self._enabled:
            return
        with self._lock:
            self._state["status"] = status
            self._state["trim"] = trim
            self._state["current_lap"] = current_lap
            self._state["current_lap_time"] = float(current_lap_time)
            self._state["best_lap_time"] = (
                float(best_lap_time) if best_lap_time is not None and best_lap_time < 1e6 else None
            )
            self._state["cum_saved"] = float(cum_saved)
            self._state["speed_mps"] = float(speed_mps)
            self._state["peak_speed_mps"] = max(float(peak_speed_mps), self._state["peak_speed_mps"])
            self._state["steer_deg"] = float(steer_rad) * 57.2957795
            self._state["lap_dist_m"] = float(lap_dist_m)
            self._state["total_dist_m"] = float(total_dist_m)
            if min_clearance_m is not None and min_clearance_m < 100.0:
                self._state["min_clearance_m"] = float(min_clearance_m)
            self._state["collisions"] = int(collisions)

    def add_completed_lap(
        self,
        *,
        lap_num,
        trim,
        lap_time,
        gain_vs_lap1,
        gain_vs_prev,
        lap_dist,
        peak_speed,
        min_clear,
    ):
        if not self._enabled:
            return
        with self._lock:
            self._state["lap_rows"].append(
                {
                    "lap_num": int(lap_num),
                    "trim": str(trim),
                    "lap_time": float(lap_time),
                    "gain_vs_lap1": float(gain_vs_lap1),
                    "gain_vs_prev": float(gain_vs_prev),
                    "lap_dist": float(lap_dist),
                    "peak_speed": float(peak_speed),
                    "min_clear": float(min_clear),
                }
            )
            self._state["laps_version"] += 1

    def _run_gui(self):
        try:
            import tkinter as tk
            from tkinter import ttk
        except Exception:
            self._enabled = False
            return

        try:
            root = tk.Tk()
            root.title(self.title)
            root.geometry("1020x620")
            root.minsize(960, 560)
            root.configure(bg="#0f172a")

            # Header frame
            header = tk.Frame(root, bg="#1e293b", padx=16, pady=10)
            header.pack(fill="x", padx=10, pady=(10, 6))

            title_lbl = tk.Label(
                header,
                text="🏎️  F1TENTH SAKHIR GP — LIVE TELEMETRY & LAP TIMER",
                font=("DejaVu Sans", 13, "bold"),
                fg="#38bdf8",
                bg="#1e293b",
            )
            title_lbl.pack(side="left")

            status_lbl = tk.Label(
                header,
                text="STATUS: RACING | TRIM: T0",
                font=("DejaVu Sans Mono", 11, "bold"),
                fg="#4ade80",
                bg="#1e293b",
            )
            status_lbl.pack(side="right")

            # Speedometer + Metrics cards
            top_cards = tk.Frame(root, bg="#0f172a")
            top_cards.pack(fill="x", padx=10, pady=4)

            # Left: Digital Speedometer
            speed_frame = tk.Frame(top_cards, bg="#1e293b", padx=16, pady=10)
            speed_frame.pack(side="left", fill="both", expand=True, padx=(0, 6))

            tk.Label(
                speed_frame,
                text="LIVE VEHICLE SPEED",
                font=("DejaVu Sans", 9, "bold"),
                fg="#94a3b8",
                bg="#1e293b",
            ).pack(anchor="w")

            speed_val_lbl = tk.Label(
                speed_frame,
                text="0.00 m/s  (0.0 km/h)",
                font=("DejaVu Sans Mono", 20, "bold"),
                fg="#f8fafc",
                bg="#1e293b",
            )
            speed_val_lbl.pack(anchor="w", pady=(2, 6))

            speed_canvas = tk.Canvas(
                speed_frame, height=18, bg="#0f172a", highlightthickness=1, highlightbackground="#334155"
            )
            speed_canvas.pack(fill="x", pady=(0, 6))
            speed_bar = speed_canvas.create_rectangle(0, 0, 0, 18, fill="#22d3ee", width=0)

            sub_speed_lbl = tk.Label(
                speed_frame,
                text="Peak: 0.00 m/s  |  Steer: +0.0°  |  Min Clear: --.--- m  |  Collisions: 0",
                font=("DejaVu Sans Mono", 9),
                fg="#cbd5e1",
                bg="#1e293b",
            )
            sub_speed_lbl.pack(anchor="w")

            # Right: Live Lap Counter & Stopwatch
            timer_frame = tk.Frame(top_cards, bg="#1e293b", padx=16, pady=10)
            timer_frame.pack(side="right", fill="both", expand=True, padx=(6, 0))

            lap_counter_lbl = tk.Label(
                timer_frame,
                text=f"LAP  0 / {self.target_laps}",
                font=("DejaVu Sans Mono", 16, "bold"),
                fg="#facc15",
                bg="#1e293b",
            )
            lap_counter_lbl.pack(anchor="w")

            cur_timer_lbl = tk.Label(
                timer_frame,
                text="Current Lap :  00.000 s",
                font=("DejaVu Sans Mono", 14, "bold"),
                fg="#f8fafc",
                bg="#1e293b",
            )
            cur_timer_lbl.pack(anchor="w", pady=(2, 2))

            best_timer_lbl = tk.Label(
                timer_frame,
                text="Best Lap    :  --.--- s   |   Time Saved: +0.000 s",
                font=("DejaVu Sans Mono", 10, "bold"),
                fg="#4ade80",
                bg="#1e293b",
            )
            best_timer_lbl.pack(anchor="w", pady=(2, 2))

            dist_lbl = tk.Label(
                timer_frame,
                text="Lap Dist    :   0.0 m     |   Total Dist:    0.0 m",
                font=("DejaVu Sans Mono", 9),
                fg="#cbd5e1",
                bg="#1e293b",
            )
            dist_lbl.pack(anchor="w")

            # Lap-by-Lap Table Frame
            table_frame = tk.Frame(root, bg="#1e293b", padx=10, pady=10)
            table_frame.pack(fill="both", expand=True, padx=10, pady=(6, 10))

            tk.Label(
                table_frame,
                text="📊 COMPLETED LAP-BY-LAP TELEMETRY & TIME-GAIN RECORDS",
                font=("DejaVu Sans", 10, "bold"),
                fg="#38bdf8",
                bg="#1e293b",
            ).pack(anchor="w", pady=(0, 6))

            style = ttk.Style()
            style.theme_use("clam")
            style.configure(
                "Treeview",
                background="#0f172a",
                foreground="#f8fafc",
                fieldbackground="#0f172a",
                rowheight=26,
                font=("DejaVu Sans Mono", 10),
            )
            style.configure(
                "Treeview.Heading",
                background="#334155",
                foreground="#38bdf8",
                font=("DejaVu Sans", 9, "bold"),
            )

            columns = ("lap", "trim", "time", "gain1", "delta", "dist", "peak", "clear", "tag")
            tree = ttk.Treeview(table_frame, columns=columns, show="headings", height=11)
            headings = {
                "lap": ("Lap #", 65),
                "trim": ("Trim", 60),
                "time": ("Lap Time (s)", 105),
                "gain1": ("Gain vs Lap 1", 110),
                "delta": ("Delta Prev", 95),
                "dist": ("Distance (m)", 100),
                "peak": ("Peak (m/s)", 95),
                "clear": ("Min Clear (m)", 105),
                "tag": ("Note", 85),
            }
            for col, (text, width) in headings.items():
                tree.heading(col, text=text)
                tree.column(col, width=width, anchor="center")

            tree.tag_configure("best", background="#14532d", foreground="#4ade80")
            tree.tag_configure("normal", background="#0f172a", foreground="#e2e8f0")
            tree.pack(fill="both", expand=True)

            def _refresh():
                with self._lock:
                    st = dict(self._state)
                    rows = list(self._state["lap_rows"])

                spd = st["speed_mps"]
                kmh = spd * 3.6
                speed_val_lbl.config(text=f"{spd:5.2f} m/s  ({kmh:4.1f} km/h)")

                c_width = max(speed_canvas.winfo_width(), 200)
                fill_w = int(min(1.0, max(0.0, spd / 18.5)) * c_width)
                bar_color = "#22d3ee" if spd < 14.5 else ("#4ade80" if spd < 16.0 else "#f59e0b")
                speed_canvas.coords(speed_bar, 0, 0, fill_w, 18)
                speed_canvas.itemconfig(speed_bar, fill=bar_color)

                mc_str = f"{st['min_clearance_m']:.3f} m" if st["min_clearance_m"] is not None else "--.--- m"
                sub_speed_lbl.config(
                    text=(
                        f"Peak: {st['peak_speed_mps']:5.2f} m/s  |  "
                        f"Steer: {st['steer_deg']:+5.1f}°  |  "
                        f"Min Clear: {mc_str}  |  Collisions: {st['collisions']}"
                    )
                )

                status_color = "#4ade80" if "RACING" in st["status"] else "#facc15"
                status_lbl.config(text=f"STATUS: {st['status']}  |  TRIM: {st['trim']}", fg=status_color)

                lap_disp = min(st["current_lap"] + 1, st["target_laps"]) if st["status"] == "RACING" else st["current_lap"]
                lap_counter_lbl.config(text=f"LAP  {lap_disp:2d} / {st['target_laps']:2d}   (Completed: {st['current_lap']})")
                cur_timer_lbl.config(text=f"Current Lap :  {st['current_lap_time']:6.3f} s")

                best_str = f"{st['best_lap_time']:6.3f} s" if st["best_lap_time"] is not None else "--.--- s"
                best_timer_lbl.config(
                    text=f"Best Lap    :  {best_str}   |   Time Saved: {st['cum_saved']:+6.3f} s"
                )
                dist_lbl.config(
                    text=f"Lap Dist    : {st['lap_dist_m']:6.1f} m     |   Total Dist: {st['total_dist_m']:7.1f} m"
                )

                if st["laps_version"] != self._shown_laps_version:
                    self._shown_laps_version = st["laps_version"]
                    for item in tree.get_children():
                        tree.delete(item)
                    best_t = min((r["lap_time"] for r in rows), default=None)
                    for r in rows:
                        is_best = best_t is not None and abs(r["lap_time"] - best_t) < 1e-6
                        note = "★ FASTEST" if is_best else "✔ CLEAN"
                        tag = "best" if is_best else "normal"
                        tree.insert(
                            "",
                            "end",
                            values=(
                                f"Lap {r['lap_num']:2d}",
                                r["trim"],
                                f"{r['lap_time']:.3f} s",
                                f"{r['gain_vs_lap1']:+.3f} s",
                                f"{r['gain_vs_prev']:+.3f} s",
                                f"{r['lap_dist']:.2f} m",
                                f"{r['peak_speed']:.2f} m/s",
                                f"{r['min_clear']:.3f} m",
                                note,
                            ),
                            tags=(tag,),
                        )

                root.after(100, _refresh)

            root.after(100, _refresh)
            root.mainloop()
        except Exception:
            self._enabled = False
