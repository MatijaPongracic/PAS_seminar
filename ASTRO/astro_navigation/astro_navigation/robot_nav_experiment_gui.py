#!/usr/bin/env python3
import os
import re
import signal
import subprocess
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def get_metrics_dir(metrics_subdir: str) -> Path:
    env_root = os.environ.get("WORKSPACE_ROOT")
    if env_root:
        return Path(env_root).expanduser().resolve() / "metrics" / metrics_subdir

    candidates = []

    for var in ("COLCON_PREFIX_PATH", "AMENT_PREFIX_PATH"):
        for entry in os.environ.get(var, "").split(":"):
            if not entry:
                continue
            p = Path(entry).expanduser().resolve()
            candidates.append(p.parent if p.name == "install" else p)

    cwd = Path.cwd().resolve()
    here = Path(__file__).resolve()

    candidates.extend([cwd, *cwd.parents, here.parent, *here.parents])

    seen = set()
    for candidate in candidates:
        candidate = candidate.resolve()
        if candidate in seen:
            continue
        seen.add(candidate)

        metrics_root = candidate / "metrics"
        if metrics_root.exists():
            return metrics_root / metrics_subdir

    raise RuntimeError(
        f"Ne mogu pronaći workspace root za metrics/{metrics_subdir}. "
        "Postavi WORKSPACE_ROOT, npr. "
        "'export WORKSPACE_ROOT=/home/$USER/<ime_workspacea>'"
    )


class NavExperimentGui(Node):
    def __init__(self):
        super().__init__('nav_experiment_gui')

        self.namespace = ''
        self.initialpose_topic = '/initialpose'
        self.reset_positions_service = None

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            10
        )

        self.reset_positions_client = None

        self.metrics_dir = get_metrics_dir("metrics_robot")
        self.scenario = 'crta_mapa'
        self.goal_x = 7.84887
        self.goal_y = 38.8461
        self.goal_yaw = 2.39103

        self.launch_proc = None
        self.benchmark_proc = None
        self.active_planner = None
        self.active_controller = None
        self.reset_done = False

        self.root = tk.Tk()
        self.root.title('Nav Experiment GUI')
        self.root.geometry('760x620')
        self.root.minsize(680, 540)
        self.root.resizable(True, True)
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        self.planner_var = tk.StringVar(value='dijkstra')
        self.controller_var = tk.StringVar(value='dwb')
        self.status_var = tk.StringVar(value='Ready.')
        self.active_var = tk.StringVar(value='Aktivno: ništa nije pokrenuto')
        self.selection_var = tk.StringVar(
            value='Odabrano: planner=dijkstra, controller=dwb'
        )

        self._build_gui()
        self._update_selection_label()
        self._refresh_start_button()
        self.root.after(500, self._poll_processes)

    def _build_gui(self):
        style = ttk.Style()
        try:
            style.theme_use('clam')
        except Exception:
            pass

        style.configure('Title.TLabel', font=('TkDefaultFont', 16, 'bold'))
        style.configure('Section.TLabelframe.Label', font=('TkDefaultFont', 12, 'bold'))
        style.configure('Big.TRadiobutton', font=('TkDefaultFont', 12))
        style.configure('Big.TButton', font=('TkDefaultFont', 11, 'bold'))
        style.configure('Info.TLabel', font=('TkDefaultFont', 11))

        main = ttk.Frame(self.root, padding=14)
        main.pack(fill=tk.BOTH, expand=True)
        main.columnconfigure(0, weight=1)
        main.rowconfigure(1, weight=1)

        ttk.Label(
            main,
            text='Navigation Experiment Control',
            style='Title.TLabel'
        ).grid(row=0, column=0, sticky='w', pady=(0, 12))

        content = ttk.Frame(main)
        content.grid(row=1, column=0, sticky='nsew')
        content.columnconfigure(0, weight=1)
        content.columnconfigure(1, weight=1)
        content.rowconfigure(0, weight=1)

        planner_frame = ttk.LabelFrame(
            content,
            text='Planner',
            style='Section.TLabelframe',
            padding=14
        )
        planner_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 8), pady=(0, 10))
        planner_frame.columnconfigure(0, weight=1)

        for i, (text, value) in enumerate([
            ('Dijkstra', 'dijkstra'),
            ('A*', 'astar'),
            ('Theta*', 'thetastar')
        ]):
            ttk.Radiobutton(
                planner_frame,
                text=text,
                value=value,
                variable=self.planner_var,
                command=self._update_selection_label,
                style='Big.TRadiobutton'
            ).grid(row=i, column=0, sticky='w', pady=8)

        controller_frame = ttk.LabelFrame(
            content,
            text='Controller',
            style='Section.TLabelframe',
            padding=14
        )
        controller_frame.grid(row=0, column=1, sticky='nsew', padx=(8, 0), pady=(0, 10))
        controller_frame.columnconfigure(0, weight=1)

        for i, (text, value) in enumerate([
            ('DWB', 'dwb'),
            ('RPP', 'rpp'),
            ('MPPI', 'mppi')
        ]):
            ttk.Radiobutton(
                controller_frame,
                text=text,
                value=value,
                variable=self.controller_var,
                command=self._update_selection_label,
                style='Big.TRadiobutton'
            ).grid(row=i, column=0, sticky='w', pady=8)

        button_frame = ttk.LabelFrame(
            main,
            text='Akcije',
            style='Section.TLabelframe',
            padding=12
        )
        button_frame.grid(row=2, column=0, sticky='ew', pady=(0, 10))
        button_frame.columnconfigure((0, 1, 2), weight=1)

        self.enter_button = ttk.Button(
            button_frame,
            text='ENTER',
            command=self.on_enter,
            style='Big.TButton'
        )
        self.enter_button.grid(row=0, column=0, sticky='ew', padx=6, pady=4, ipady=8)

        self.reset_button = ttk.Button(
            button_frame,
            text='RESET',
            command=self.on_reset,
            style='Big.TButton'
        )
        self.reset_button.grid(row=0, column=1, sticky='ew', padx=6, pady=4, ipady=8)

        self.start_button = ttk.Button(
            button_frame,
            text='START',
            command=self.on_start,
            style='Big.TButton'
        )
        self.start_button.grid(row=0, column=2, sticky='ew', padx=6, pady=4, ipady=8)

        info_frame = ttk.LabelFrame(
            main,
            text='Status',
            style='Section.TLabelframe',
            padding=12
        )
        info_frame.grid(row=3, column=0, sticky='nsew')
        info_frame.columnconfigure(0, weight=1)

        ttk.Label(
            info_frame,
            textvariable=self.selection_var,
            style='Info.TLabel',
            justify='left',
            wraplength=700
        ).grid(row=0, column=0, sticky='w', pady=(0, 8))

        ttk.Label(
            info_frame,
            textvariable=self.active_var,
            style='Info.TLabel',
            justify='left',
            wraplength=700
        ).grid(row=1, column=0, sticky='w', pady=(0, 8))

        ttk.Label(
            info_frame,
            textvariable=self.status_var,
            style='Info.TLabel',
            justify='left',
            wraplength=700,
            foreground='blue'
        ).grid(row=2, column=0, sticky='w')

    def _update_selection_label(self):
        self.selection_var.set(
            f'Odabrano: planner={self.planner_var.get()}, controller={self.controller_var.get()}'
        )
        self._refresh_start_button()

    def set_status(self, text):
        self.status_var.set(text)
        self.get_logger().info(text)

    def _is_running(self, proc):
        return proc is not None and proc.poll() is None

    def _selection_is_entered(self):
        return (
            self._is_running(self.launch_proc) and
            self.active_planner == self.planner_var.get() and
            self.active_controller == self.controller_var.get()
        )

    def _refresh_start_button(self):
        can_start = (
            self._selection_is_entered() and
            self.reset_done and
            not self._is_running(self.benchmark_proc)
        )

        if can_start:
            self.start_button.state(['!disabled'])
        else:
            self.start_button.state(['disabled'])

    def _terminate_process(self, proc, name):
        if not self._is_running(proc):
            return

        self.set_status(f'Gasim {name}...')

        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=8.0)
            return
        except Exception:
            pass

        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5.0)
            return
        except Exception:
            pass

        try:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=2.0)
        except Exception:
            pass

    def _next_run_name(self, planner, controller):
        self.metrics_dir.mkdir(parents=True, exist_ok=True)
        prefix = f'{planner}_{controller}_run'
        pattern = re.compile(rf'^{re.escape(prefix)}(\d+)$')
        max_num = 0

        for entry in self.metrics_dir.iterdir():
            candidates = {entry.name, entry.stem}
            for candidate in candidates:
                match = pattern.match(candidate)
                if match:
                    max_num = max(max_num, int(match.group(1)))

        return f'{prefix}{max_num + 1:02d}'

    def _launch_command(self, planner, controller):
        return [
            'ros2', 'launch', 'astro_navigation', 'navigation.launch.py',
            f'planner:={planner}',
            f'controller:={controller}',
        ]

    def _benchmark_command(self, run_name, planner, controller):
        return [
            'ros2', 'run', 'astro_navigation', 'robot_nav_benchmark_runner.py', '--ros-args',
            '-p', f'output_dir:={self.metrics_dir}',
            '-p', f'run_name:={run_name}',
            '-p', f'planner_label:={planner}',
            '-p', f'controller_label:={controller}',
            '-p', f'scenario:={self.scenario}',
            '-p', f'goal_x:={self.goal_x}',
            '-p', f'goal_y:={self.goal_y}',
            '-p', f'goal_yaw:={self.goal_yaw}',
        ]

    def _call_reset_positions_service(self):
        return

    def on_enter(self):
        planner = self.planner_var.get()
        controller = self.controller_var.get()

        if self._is_running(self.benchmark_proc):
            messagebox.showwarning(
                'Benchmark u tijeku',
                'Pričekaj da završi benchmark prije promjene planner/controller kombinacije.'
            )
            return

        if self._is_running(self.launch_proc):
            if planner == self.active_planner and controller == self.active_controller:
                self.set_status(
                    f'Navigation launch već radi za {planner} + {controller}.'
                )
                self._refresh_start_button()
                return

            self._terminate_process(self.launch_proc, 'navigation launch')
            self.launch_proc = None
            self.active_planner = None
            self.active_controller = None
            self.reset_done = False

        cmd = self._launch_command(planner, controller)

        try:
            self.launch_proc = subprocess.Popen(cmd, start_new_session=True)
            self.active_planner = planner
            self.active_controller = controller
            self.reset_done = False
            self.active_var.set(
                f'Aktivno: planner={planner}, controller={controller}'
            )
            self.set_status(
                f'Pokrenut navigation launch za {planner} + {controller}. '
                f'Prije START pritisni RESET.'
            )
        except Exception as exc:
            self.launch_proc = None
            self.active_planner = None
            self.active_controller = None
            self.reset_done = False
            self.active_var.set('Aktivno: ništa nije pokrenuto')
            messagebox.showerror('Greška pri pokretanju', str(exc))
            self.set_status(f'Greška pri pokretanju launch-a: {exc}')

        self._refresh_start_button()

    def on_reset(self):

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = 8.441
        msg.pose.pose.position.y = 28.987
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.378338
        msg.pose.pose.orientation.w = 0.925668

        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685,
        ]

        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initialpose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        self.reset_done = True
        self.set_status(
            f'Objavljen initial pose na {self.initialpose_topic}.'
        )
        self._refresh_start_button()

    def on_start(self):
        planner = self.planner_var.get()
        controller = self.controller_var.get()

        if not self._is_running(self.launch_proc):
            messagebox.showwarning(
                'Nema aktivnog launch-a',
                'Najprije pritisni ENTER da pokreneš navigation launch.'
            )
            self._refresh_start_button()
            return

        if planner != self.active_planner or controller != self.active_controller:
            messagebox.showwarning(
                'Odabir nije aktiviran',
                'Trenutni radio button odabir nije aktivan launch. Pritisni ENTER za odabranu kombinaciju pa tek onda START.'
            )
            self._refresh_start_button()
            return

        if not self.reset_done:
            messagebox.showwarning(
                'Potreban RESET',
                'Prije START moraš pritisnuti RESET.'
            )
            self._refresh_start_button()
            return

        if self._is_running(self.benchmark_proc):
            messagebox.showwarning(
                'Benchmark u tijeku',
                'Benchmark je već pokrenut. Pričekaj da završi.'
            )
            self._refresh_start_button()
            return

        run_name = self._next_run_name(planner, controller)
        cmd = self._benchmark_command(run_name, planner, controller)

        try:
            self.benchmark_proc = subprocess.Popen(cmd, start_new_session=True)
            self.set_status(
                f'Pokrenut benchmark: {run_name}'
            )
        except Exception as exc:
            self.benchmark_proc = None
            messagebox.showerror('Greška pri benchmarku', str(exc))
            self.set_status(f'Greška pri pokretanju benchmarka: {exc}')

        self._refresh_start_button()

    def _poll_processes(self):
        if self.launch_proc is not None and self.launch_proc.poll() is not None:
            code = self.launch_proc.returncode
            self.launch_proc = None
            self.active_planner = None
            self.active_controller = None
            self.reset_done = False
            self.active_var.set('Aktivno: ništa nije pokrenuto')
            self.set_status(f'Navigation launch je završio s kodom {code}.')

        if self.benchmark_proc is not None and self.benchmark_proc.poll() is not None:
            code = self.benchmark_proc.returncode
            self.benchmark_proc = None
            self.reset_done = False
            self.set_status(
                f'Benchmark je završio s kodom {code}. '
                f'Za novi START prvo pritisni RESET.'
            )

        self._refresh_start_button()
        self.root.after(500, self._poll_processes)

    def on_close(self):
        try:
            if self._is_running(self.benchmark_proc):
                self._terminate_process(self.benchmark_proc, 'benchmark')
            if self._is_running(self.launch_proc):
                self._terminate_process(self.launch_proc, 'navigation launch')
        finally:
            self.destroy_node()
            rclpy.shutdown()
            self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    gui = NavExperimentGui()
    gui.run()


if __name__ == '__main__':
    main()
