"""
GUI for Fuselage Volume Optimization - REAL-TIME LOG (SUBPROCESS VERSION)
==========================================================================
With real-time validation, optimized colors, dynamic status, and compact layout.
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import os
import json
import csv
import sys
import subprocess
import threading
import queue
from typing import Optional, Dict, Any, List

# Import your existing optimization module
try:
    from Aero_analysis import OptConfig
except ImportError as e:
    print(f"Warning: Could not import optimization module: {e}")
    class OptConfig:
        OUTPUT_BASE = "./optimization_results"


class OptimizationProcess:
    """Handles the optimization process using a subprocess to capture real-time output."""
    def __init__(self):
        self.process = None
        self.is_running = False
        self.output_queue = queue.Queue()
        self.reader_thread = None

    def start(self, config_dict):
        """
        Starts the optimization via subprocess.
        Args:
            config_dict: Dictionary containing the configuration parameters.
        Returns:
            True if the process started successfully, False otherwise.
        """
        if self.is_running:
            return False

        self.is_running = True
        script = self._create_runner_script(config_dict)

        try:
            self.process = subprocess.Popen(
                [sys.executable, '-c', script],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding='utf-8',  # Encoding correction
                errors='replace', # Encoding correction
                bufsize=1,
                universal_newlines=True
            )

            self.reader_thread = threading.Thread(target=self._read_output, daemon=True)
            self.reader_thread.start()

            return True
        except Exception as e:
            print(f"Error starting process: {e}")
            self.is_running = False
            return False

    def stop(self):
        """Stops the running optimization process."""
        if self.process:
            try:
                self.process.terminate()
                self.process.wait(timeout=5)
            except:
                self.process.kill()
            self.is_running = False
            return True
        return False

    def is_alive(self):
        """
        Checks if the optimization process is still running.
        Returns:
            True if the process is running, False otherwise.
        """
        if self.process:
            return self.process.poll() is None
        return False

    def get_output(self):
        """
        Reads messages from the output queue in a non-blocking way.
        Returns:
            A list of output messages from the queue.
        """
        messages = []
        try:
            while True:
                msg = self.output_queue.get_nowait()
                messages.append(msg)
        except queue.Empty:
            pass
        return messages

    def _read_output(self):
        """Internal method to read output from the process in real-time."""
        try:
            if self.process and self.process.stdout:
                for line in iter(self.process.stdout.readline, ''):
                    if line:
                        self.output_queue.put(line.rstrip('\n'))
        except Exception as e:
            self.output_queue.put(f"ERROR reading output: {e}")

    def _create_runner_script(self, config_dict):
        """
        Creates the Python script string to be executed by the subprocess.
        Args:
            config_dict: The configuration dictionary to pass to the script.
        Returns:
            A string containing the Python script.
        """
        script = f"""
import sys
import os # Added for file check
from Aero_analysis import run_optimization, OptConfig, load_volumes_from_file

config_dict = {config_dict}
OptConfig.update_from_gui(config_dict)

try:
    run_optimization()
except Exception as e:
    print(f"ERROR: {{e}}")
    import traceback
    traceback.print_exc()
"""
        return script


class FuselageOptimizationGUI:
    # Custom colors
    COLORS = {
        'bg_console': '#1e1e1e',
        'fg_console': '#e8e8e8',
        'accent_console': '#4fc3f7',
    }

    STATUS_COLORS = {
        'ready': '#4caf50',
        'running': '#ff9800',
        'stopped': '#f44336',
        'completed': '#4caf50',
        'error': '#f44336'
    }

    def __init__(self, root):
        self.root = root
        self.root.title("Fuselage Optimization - MDO Software")
        self.root.geometry("1400x800")
        self.root.resizable(True, True)

        self.current_config = {}
        self.optimization_process = OptimizationProcess()

        self.create_gui()
        self.load_default_config()
        self.update_gui_from_config() # Ensure defaults are displayed
        self.monitor_process()

    def create_gui(self):
        """Create the main GUI layout with two columns"""
        main_container = ttk.Frame(self.root)
        main_container.pack(fill='both', expand=True, padx=10, pady=10)

        # LEFT COLUMN - Parameters
        left_frame = ttk.Frame(main_container)
        left_frame.pack(side='left', fill='both', expand=True, padx=(0, 5))

        # RIGHT COLUMN - Results & Status
        right_frame = ttk.Frame(main_container)
        right_frame.pack(side='right', fill='both', expand=True, padx=(5, 0))

        self.create_parameters_section(left_frame)
        self.create_results_section(right_frame)

    def create_parameters_section(self, parent):
        """Create parameters input section"""
        params_notebook = ttk.Notebook(parent)
        params_notebook.pack(fill='both', expand=True)

        self.create_main_params_tab(params_notebook)
        self.create_advanced_params_tab(params_notebook)

    def create_main_params_tab(self, notebook):
        """Create the main parameters tab (compact, no scrollbar)."""
        main_frame = ttk.Frame(notebook)
        notebook.add(main_frame, text="Main Parameters")

        title_label = ttk.Label(main_frame, text="Fuselage Parameters", font=('Arial', 12, 'bold'))
        # Use anchor='n' (north) to center it at the top
        title_label.pack(pady=(5, 10), padx=5, anchor='n')

        # --- Container for parameter frames ---
        # Use a normal frame, not a scrollable one
        # Center it using anchor='n'
        params_container = ttk.Frame(main_frame)
        params_container.pack(fill='x', expand=False, padx=10, pady=5, anchor='n')

        # LENGTH PARAMETERS - Compact
        length_frame = ttk.LabelFrame(params_container, text="Length (L)", padding=5)
        length_frame.pack(fill='x', padx=5, pady=3)

        ttk.Label(length_frame, text="Min:").grid(row=0, column=0, sticky='w', padx=2, pady=2)
        self.l_min_var = tk.DoubleVar(value=18.0)
        self._create_validated_spinbox(length_frame, from_=0, textvariable=self.l_min_var, width=10,
                                       increment=0.1).grid(row=0, column=1, padx=2, pady=2)
        ttk.Label(length_frame, text="m").grid(row=0, column=2, sticky='w', padx=1)

        ttk.Label(length_frame, text="Max:").grid(row=1, column=0, sticky='w', padx=2, pady=2)
        self.l_max_var = tk.DoubleVar(value=22.0)
        self._create_validated_spinbox(length_frame, from_=0, textvariable=self.l_max_var, width=10,
                                       increment=0.1).grid(row=1, column=1, padx=2, pady=2)
        ttk.Label(length_frame, text="m").grid(row=1, column=2, sticky='w', padx=1)

        ttk.Label(length_frame, text="Step:").grid(row=2, column=0, sticky='w', padx=2, pady=2)
        self.l_step_var = tk.DoubleVar(value=1.0)
        # Minimum step correction
        self._create_validated_spinbox(length_frame, from_=0.01, textvariable=self.l_step_var, width=10,
                                       increment=0.01).grid(row=2, column=1, padx=2, pady=2)
        ttk.Label(length_frame, text="m").grid(row=2, column=2, sticky='w', padx=1)

        # RADIUS PARAMETERS - Compact
        radius_frame = ttk.LabelFrame(params_container, text="Radius (R)", padding=5)
        radius_frame.pack(fill='x', padx=5, pady=3)

        ttk.Label(radius_frame, text="Min:").grid(row=0, column=0, sticky='w', padx=2, pady=2)
        self.r_min_var = tk.DoubleVar(value=1.35)
        self._create_validated_spinbox(radius_frame, from_=0, textvariable=self.r_min_var, width=10,
                                       increment=0.1).grid(row=0, column=1, padx=2, pady=2)
        ttk.Label(radius_frame, text="m").grid(row=0, column=2, sticky='w', padx=1)

        ttk.Label(radius_frame, text="Max:").grid(row=1, column=0, sticky='w', padx=2, pady=2)
        self.r_max_var = tk.DoubleVar(value=1.65)
        self._create_validated_spinbox(radius_frame, from_=0, textvariable=self.r_max_var, width=10,
                                       increment=0.1).grid(row=1, column=1, padx=2, pady=2)
        ttk.Label(radius_frame, text="m").grid(row=1, column=2, sticky='w', padx=1)

        ttk.Label(radius_frame, text="Step:").grid(row=2, column=0, sticky='w', padx=2, pady=2)
        self.r_step_var = tk.DoubleVar(value=0.15)
        # Minimum step correction
        self._create_validated_spinbox(radius_frame, from_=0.01, textvariable=self.r_step_var, width=10,
                                       increment=0.01).grid(row=2, column=1, padx=2, pady=2)
        ttk.Label(radius_frame, text="m").grid(row=2, column=2, sticky='w', padx=1)

        # CONFIGURATION SUMMARY - Compact
        summary_frame = ttk.LabelFrame(params_container, text="Configuration Summary", padding=5)
        summary_frame.pack(fill='x', padx=5, pady=3)

        self.summary_text = tk.StringVar(value="Click Calculate to see configurations")

        # 1. Remove fixed wraplength and save the widget
        self.summary_label = ttk.Label(summary_frame, textvariable=self.summary_text, font=('Arial', 9),
                                       foreground='darkblue', justify='left')
        self.summary_label.pack(anchor='w', padx=2, pady=2)

        ttk.Button(summary_frame, text="Calculate", command=self.calculate_summary, width=15).pack(pady=3)

        # 2. Define the callback function
        def configure_summary_wrap(event):
            # Calculate new width. Subtract 20px for padding.
            new_width = event.width - 20
            # Apply the new wraplength only if width is positive.
            if new_width > 0:
                self.summary_label.config(wraplength=new_width)

        # 3. Bind the <Configure> event to the parent frame
        summary_frame.bind("<Configure>", configure_summary_wrap)

        # VOLUME FILE INPUT
        volume_frame = ttk.LabelFrame(params_container, text="Input Data", padding=5)
        volume_frame.pack(fill='x', padx=5, pady=3)

        self.volume_file_var = tk.StringVar(value="")

        ttk.Label(volume_frame, text="Volume File:").grid(row=0, column=0, sticky='w', padx=2, pady=5)

        self.volume_file_label = ttk.Entry(volume_frame, textvariable=self.volume_file_var, width=50, state='readonly')
        self.volume_file_label.grid(row=0, column=1, sticky='we', padx=2, pady=5)

        ttk.Button(volume_frame, text="Browse...", command=self.load_volume_file, width=12).grid(row=0, column=2,
                                                                                                 sticky='e', padx=5,
                                                                                                 pady=5)
        # This line is correct and applies to 'volume_frame'
        volume_frame.grid_columnconfigure(1, weight=1)

    def create_advanced_params_tab(self, notebook):
        """Create the advanced parameters tab (compact, no scrollbar)."""
        advanced_frame = ttk.Frame(notebook)
        notebook.add(advanced_frame, text="Advanced")

        title_label = ttk.Label(advanced_frame, text="Advanced Settings", font=('Arial', 11, 'bold'))
        title_label.pack(pady=(5, 10), padx=5, anchor='n')

        # --- Container for parameter frames ---
        # Use a normal frame, not a scrollable one
        params_container = ttk.Frame(advanced_frame)
        params_container.pack(fill='x', expand=False, padx=10, pady=5, anchor='n')

        # OPTIMIZATION PARAMETERS
        opt_frame = ttk.LabelFrame(params_container, text="Optimization", padding=5)
        opt_frame.pack(fill='x', padx=5, pady=3)

        ttk.Label(opt_frame, text="Safety Margin:").grid(row=0, column=0, sticky='w', padx=2, pady=2)
        self.safety_margin_var = tk.DoubleVar(value=0.8)
        self._create_validated_spinbox(opt_frame, from_=0, to=1.0, textvariable=self.safety_margin_var, width=10,
                                       increment=0.05).grid(row=0, column=1, padx=2, pady=2)

        ttk.Label(opt_frame, text="CD Weight:").grid(row=1, column=0, sticky='w', padx=2, pady=2)
        self.cd_weight_var = tk.DoubleVar(value=0.7)
        self._create_validated_spinbox(opt_frame, from_=0, to=1.0, textvariable=self.cd_weight_var, width=10,
                                       increment=0.05).grid(row=1, column=1, padx=2, pady=2)

        ttk.Label(opt_frame, text="Volume Weight:").grid(row=2, column=0, sticky='w', padx=2, pady=2)
        self.volume_weight_var = tk.DoubleVar(value=0.3)
        self._create_validated_spinbox(opt_frame, from_=0, to=1.0, textvariable=self.volume_weight_var, width=10,
                                       increment=0.05).grid(row=2, column=1, padx=2, pady=2)

        # AERODYNAMIC ANALYSIS
        aero_frame = ttk.LabelFrame(params_container, text="Aerodynamic", padding=5)
        aero_frame.pack(fill='x', padx=5, pady=3)

        ttk.Label(aero_frame, text="Sref (m²):").grid(row=0, column=0, sticky='w', padx=2, pady=2)
        self.sref_var = tk.DoubleVar(value=100.0)
        self._create_validated_spinbox(aero_frame, from_=0, textvariable=self.sref_var, width=10, increment=10.0).grid(
            row=0, column=1, padx=2, pady=2)

        ttk.Label(aero_frame, text="Velocity (m/s):").grid(row=1, column=0, sticky='w', padx=2, pady=2)
        self.vinf_var = tk.DoubleVar(value=150.0)
        self._create_validated_spinbox(aero_frame, from_=0, textvariable=self.vinf_var, width=10, increment=10.0).grid(
            row=1, column=1, padx=2, pady=2)

        ttk.Label(aero_frame, text="Altitude (m):").grid(row=2, column=0, sticky='w', padx=2, pady=2)
        self.altitude_var = tk.DoubleVar(value=6096.0)
        self._create_validated_spinbox(aero_frame, from_=0, textvariable=self.altitude_var, width=10,
                                       increment=1000.0).grid(row=2, column=1, padx=2, pady=2)

        ttk.Label(aero_frame, text="ΔTemp (K):").grid(row=3, column=0, sticky='w', padx=2, pady=2)
        self.delta_temp_var = tk.DoubleVar(value=0.0)
        self._create_validated_spinbox(aero_frame, from_=-float('inf'), textvariable=self.delta_temp_var, width=10,
                                       increment=1.0).grid(row=3, column=1, padx=2, pady=2)

        button_frame = ttk.Frame(params_container)
        button_frame.pack(fill='x', padx=5, pady=10)
        ttk.Button(button_frame, text="Reset to Defaults", command=self.reset_to_defaults, width=20).pack()

    def create_results_section(self, parent):
        """Create results display section"""
        # STATUS FRAME
        status_frame = ttk.LabelFrame(parent, text="Status", padding=8)
        status_frame.pack(fill='x', pady=(0, 5))

        self.status_var = tk.StringVar(value="Ready")
        self.status_label = ttk.Label(status_frame, textvariable=self.status_var, font=('Arial', 11, 'bold'))
        self.status_label.pack(fill='x', pady=3)
        self.update_status('ready')

        # LOG FRAME
        log_frame = ttk.LabelFrame(parent, text="Real-Time Log", padding=8)
        log_frame.pack(fill='both', expand=True, pady=(0, 5))

        self.results_text = tk.Text(
            log_frame,
            height=30,
            wrap='word',
            state='disabled',
            bg=self.COLORS['bg_console'],
            fg=self.COLORS['fg_console'],
            font=('Courier New', 8),
            insertbackground=self.COLORS['accent_console']
        )
        scrollbar = ttk.Scrollbar(log_frame, orient='vertical', command=self.results_text.yview)
        self.results_text.configure(yscrollcommand=scrollbar.set)

        self.results_text.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')

        # Tags for coloring
        self.results_text.tag_configure('info', foreground=self.COLORS['accent_console'])
        self.results_text.tag_configure('success', foreground='#4caf50')
        self.results_text.tag_configure('error', foreground='#f44336')
        self.results_text.tag_configure('warning', foreground='#ff9800')

        # CONTROL BUTTONS
        button_frame = ttk.Frame(parent)
        button_frame.pack(fill='x', pady=(0, 5))

        ttk.Button(button_frame, text="Validate", command=self.validate_parameters, width=12).pack(side='left', padx=2)
        ttk.Button(button_frame, text="Save Config", command=self.save_configuration, width=12).pack(side='left', padx=2)
        ttk.Button(button_frame, text="Load Config", command=self.load_configuration, width=12).pack(side='left', padx=2)
        ttk.Button(button_frame, text="Save Log", command=self.save_results, width=12).pack(side='left', padx=2)
        ttk.Button(button_frame, text="Clear Log", command=self.clear_results, width=12).pack(side='left', padx=2)
        ttk.Button(button_frame, text="Open Folder", command=self.open_results_folder, width=12).pack(side='left', padx=2)

        self.stop_button = ttk.Button(button_frame, text="⛔ Stop", command=self.stop_optimization, state='disabled', width=12)
        self.stop_button.pack(side='right', padx=2)

        self.run_button = ttk.Button(button_frame, text="▶ Run", command=self.run_optimization, width=12)
        self.run_button.pack(side='right', padx=2)

    def _create_validated_spinbox(self, parent, from_, to=None, textvariable=None, width=12, increment=0.1, on_focus_out=None):
        """
        Creates a Spinbox with real-time validation.
        Args:
            parent: The parent widget.
            from_: The minimum value.
            to: The maximum value (optional).
            textvariable: The tk.DoubleVar to link.
            width: The widget width.
            increment: The step for spin buttons.
            on_focus_out: Optional callback for FocusOut event.
        Returns:
            A ttk.Spinbox widget.
        """
        spinbox = ttk.Spinbox(
            parent,
            from_=from_,
            to=to if to is not None else 1e9,
            textvariable=textvariable,
            width=width,
            increment=increment
        )

        def on_focus_out_event(event):
            try:
                val = textvariable.get()
                if val < from_:
                    textvariable.set(from_)
                    if on_focus_out:
                        on_focus_out()
                elif to is not None and val > to:
                    textvariable.set(to)
                    if on_focus_out:
                        on_focus_out()
            except (tk.TclError, ValueError):
                textvariable.set(from_)
                if on_focus_out:
                    on_focus_out()

        spinbox.bind('<FocusOut>', on_focus_out_event)
        return spinbox

    def update_status(self, status_type):
        """Updates the status label with the appropriate color."""
        color = self.STATUS_COLORS.get(status_type, '#999999')
        self.status_label.configure(foreground=color)

    def calculate_summary(self):
        """Calculate and display configuration count"""
        try:
            l_min = self.l_min_var.get()
            l_max = self.l_max_var.get()
            l_step = self.l_step_var.get()

            r_min = self.r_min_var.get()
            r_max = self.r_max_var.get()
            r_step = self.r_step_var.get()

            # Validate parameters before calculation
            errors = []

            if l_min < 0:
                errors.append("L_min < 0")
            if l_max < 0:
                errors.append("L_max < 0")
            if l_max <= l_min:
                errors.append("L_max ≤ L_min")
            if l_step <= 0:
                errors.append("L_step ≤ 0")

            if r_min < 0:
                errors.append("R_min < 0")
            if r_max < 0:
                errors.append("R_max < 0")
            if r_max <= r_min:
                errors.append("R_max ≤ R_min")
            if r_step <= 0:
                errors.append("R_step ≤ 0")

            if errors:
                summary = "❌ Invalid: " + ", ".join(errors)
                self.summary_text.set(summary)
                return

            l_count = int(round((l_max - l_min) / l_step)) + 1 if l_step > 0 else 1
            r_count = int(round((r_max - r_min) / r_step)) + 1 if r_step > 0 else 1


            total = l_count * r_count * r_count

            summary = f"""Configuration Summary:
            • Length values: {l_count} ({l_min}m to {l_max}m, step {l_step}m)
            • Radius values: {r_count} ({r_min}m to {r_max}m, step {r_step}m)
            • Width values: {r_count} (derived from radius)
            • Height values: {r_count} (derived from radius)

            TOTAL CONFIGURATIONS: {total}"""

            self.summary_text.set(summary)
            self.log_message(f"Configuration count calculated: {total} total configurations", 'info')

        except Exception as e:
            messagebox.showerror("Calculation Error", f"Error calculating summary: {str(e)}")

    def load_default_config(self):
        """Load default configuration values"""
        self.current_config = {
            'l_min': 18.0,
            'l_max': 22.0,
            'l_step': 1.0,
            'r_min': 1.35,
            'r_max': 1.65,
            'r_step': 0.15,
            'safety_margin': 0.8,
            'cd_weight': 0.7,
            'volume_weight': 0.3,
            'sref': 100.0,
            'vinf': 150.0,
            'altitude': 6096.0,
            'delta_temp': 0.0,
            'volume_file': '' # Added
        }

    def reset_to_defaults(self):
        """Reset all parameters to default values"""
        self.load_default_config()
        self.update_gui_from_config()
        self.log_message("Parameters reset to defaults", 'info')

    def update_gui_from_config(self):
        """Update GUI elements from current configuration"""
        config = self.current_config
        self.l_min_var.set(config.get('l_min', 18.0))
        self.l_max_var.set(config.get('l_max', 22.0))
        self.l_step_var.set(config.get('l_step', 1.0))
        self.r_min_var.set(config.get('r_min', 1.35))
        self.r_max_var.set(config.get('r_max', 1.65))
        self.r_step_var.set(config.get('r_step', 0.15))
        self.safety_margin_var.set(config.get('safety_margin', 0.8))
        self.cd_weight_var.set(config.get('cd_weight', 0.7))
        self.volume_weight_var.set(config.get('volume_weight', 0.3))
        self.sref_var.set(config.get('sref', 100.0))
        self.vinf_var.set(config.get('vinf', 150.0))
        self.altitude_var.set(config.get('altitude', 6096.0))
        self.delta_temp_var.set(config.get('delta_temp', 0.0))
        self.volume_file_var.set(config.get('volume_file', '')) # Added

    def get_current_config(self) -> Dict[str, Any]:
        """Get current configuration from GUI"""
        return {
            'l_min': self.l_min_var.get(),
            'l_max': self.l_max_var.get(),
            'l_step': self.l_step_var.get(),
            'r_min': self.r_min_var.get(),
            'r_max': self.r_max_var.get(),
            'r_step': self.r_step_var.get(),
            'safety_margin': self.safety_margin_var.get(),
            'cd_weight': self.cd_weight_var.get(),
            'volume_weight': self.volume_weight_var.get(),
            'sref': self.sref_var.get(),
            'vinf': self.vinf_var.get(),
            'altitude': self.altitude_var.get(),
            'delta_temp': self.delta_temp_var.get(),
            'volume_file': self.volume_file_var.get() # Added
        }

    def validate_parameters(self, show_success_message=True) -> bool:
        """
        Validate all input parameters.
        Args:
            show_success_message: If True, show a popup on successful validation.
        Returns:
            True if all parameters are valid, False otherwise.
        """
        try:
            config = self.get_current_config()
            errors = []

            if config['l_min'] < 0:
                errors.append("Minimum length cannot be negative")
            if config['l_max'] < 0:
                errors.append("Maximum length cannot be negative")
            if config['l_max'] <= config['l_min']:
                errors.append("Maximum length must be greater than minimum length")
            if config['l_step'] <= 0:
                errors.append("Length step must be positive")
            if (config['l_max'] - config['l_min']) > 0 and config['l_step'] > (config['l_max'] - config['l_min']):
                errors.append("Length step must be smaller than the range")

            if config['r_min'] < 0:
                errors.append("Minimum radius cannot be negative")
            if config['r_max'] < 0:
                errors.append("Maximum radius cannot be negative")
            if config['r_max'] <= config['r_min']:
                errors.append("Maximum radius must be greater than minimum radius")
            if config['r_step'] <= 0:
                errors.append("Radius step must be positive")
            if (config['r_max'] - config['r_min']) > 0 and config['r_step'] > (config['r_max'] - config['r_min']):
                errors.append("Radius step must be smaller than the range")

            if config['sref'] < 0:
                errors.append("Reference area cannot be negative")
            if config['vinf'] < 0:
                errors.append("Velocity cannot be negative")
            if config['altitude'] < 0:
                errors.append("Altitude cannot be negative")

            if not (0 <= config['safety_margin'] <= 1.0):
                errors.append("Safety margin must be between 0 and 1")
            if not (0 <= config['cd_weight'] <= 1.0):
                errors.append("CD weight must be between 0 and 1")
            if not (0 <= config['volume_weight'] <= 1.0):
                errors.append("Volume weight must be between 0 and 1")

            total_weight = config['cd_weight'] + config['volume_weight']
            if abs(total_weight - 1.0) > 0.001:
                errors.append(f"CD and Volume weights must sum to 1.0 (current: {total_weight:.3f})")

            # --- FILE VALIDATION ---
            if not config['volume_file']:
                errors.append("Volume definition file is not selected")
            elif not os.path.exists(config['volume_file']):
                errors.append(f"Volume file not found at: {config['volume_file']}")
            # --- END FILE VALIDATION ---

            if errors:
                error_msg = "Parameter validation failed:\n• " + "\n• ".join(errors)
                messagebox.showerror("Validation Error", error_msg)
                self.status_var.set("✗ Validation Failed")  # Status correction
                self.update_status('error')
                return False
            else:
                if show_success_message:
                    messagebox.showinfo("Validation Successful", "All parameters are valid!")
                    self.status_var.set("Ready")  # Status correction
                    self.update_status('ready')
                return True

        except Exception as e:
            messagebox.showerror("Validation Error", f"Error during validation: {str(e)}")
            self.status_var.set("✗ Validation Error")  # Status correction
            self.update_status('error')
            return False

    def load_volume_file(self):
        """Open file dialog to select volume definition file"""
        filename = filedialog.askopenfilename(
            title="Select Volume Definition File",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            self.volume_file_var.set(filename)
            self.log_message(f"Volume file selected: {filename}", 'info')

    def save_configuration(self):
        """Save current configuration to file"""
        filename = filedialog.asksaveasfilename(title="Save Configuration", defaultextension=".json", filetypes=[("JSON files", "*.json"), ("All files", "*.*")])

        if filename:
            try:
                config = self.get_current_config()
                with open(filename, 'w') as f:
                    json.dump(config, f, indent=2)
                self.log_message(f"Config saved: {filename}", 'success')
                messagebox.showinfo("Success", "Configuration saved!")
            except Exception as e:
                self.log_message(f"Save failed: {str(e)}", 'error')
                messagebox.showerror("Save Error", f"Failed: {str(e)}")

    def load_configuration(self):
        """Load configuration from file"""
        filename = filedialog.askopenfilename(title="Load Configuration", filetypes=[("JSON files", "*.json"), ("All files", "*.*")])

        if filename:
            try:
                with open(filename, 'r') as f:
                    config = json.load(f)
                self.current_config = config
                self.update_gui_from_config()
                self.log_message(f"Config loaded: {filename}", 'success')
                messagebox.showinfo("Success", "Configuration loaded!")
            except Exception as e:
                self.log_message(f"Load failed: {str(e)}", 'error')
                messagebox.showerror("Load Error", f"Failed: {str(e)}")

    def run_optimization(self):
        """Run the optimization process"""
        if not self.validate_parameters(show_success_message=False):
            # Status is already set by validate_parameters
            return

        if self.optimization_process.is_running:
            messagebox.showwarning("Warning", "Optimization already running!")
            return

        self.run_button.config(state='disabled')
        self.stop_button.config(state='normal')
        self.status_var.set("🔄 Running...")
        self.update_status('running')

        config = self.get_current_config()
        self.clear_results()

        self.log_message("=" * 80, 'info')
        self.log_message("FUSELAGE OPTIMIZATION STARTED", 'info')
        self.log_message("=" * 80, 'info')
        self.log_message(f"L: {config['l_min']}-{config['l_max']}m (Δ {config['l_step']}m)")
        self.log_message(f"R: {config['r_min']}-{config['r_max']}m (Δ {config['r_step']}m)")
        self.log_message(f"Weights - CD: {config['cd_weight']}, Vol: {config['volume_weight']}, SM: {config['safety_margin']}")
        self.log_message(f"Aero - Sref: {config['sref']}m², V: {config['vinf']}m/s, Alt: {config['altitude']}m")
        self.log_message(f"Volume File: {config['volume_file']}")
        self.log_message("")

        success = self.optimization_process.start(config)

        if success:
            self.monitor_process()
        else:
            messagebox.showerror("Error", "Failed to start optimization")
            self.run_button.config(state='normal')
            self.stop_button.config(state='disabled')
            self.status_var.set("✗ Error")
            self.update_status('error')

    def stop_optimization(self):
        """Stop the running optimization"""
        if self.optimization_process.is_alive():
            if self.optimization_process.stop():
                self.status_var.set("⛔ Stopped")
                self.update_status('stopped')
                self.log_message("⛔ Optimization stopped by user", 'warning')
                self.run_button.config(state='normal')
                self.stop_button.config(state='disabled')
            else:
                self.log_message("⚠ Failed to stop", 'warning')
        else:
            self.log_message("⚠ No optimization running", 'warning')

    def monitor_process(self):
        """Monitor the optimization process and read output in real-time"""
        messages = self.optimization_process.get_output()
        for msg in messages:
            self.log_message(msg)

        if self.optimization_process.is_running:
            if not self.optimization_process.is_alive():
                self.optimization_process.is_running = False

                final_messages = self.optimization_process.get_output()
                for msg in final_messages:
                    self.log_message(msg)

                self.run_button.config(state='normal')
                self.stop_button.config(state='disabled')

                # Check the output for errors
                log_content = self.results_text.get('1.0', 'end-1c')
                if "FATAL ERROR" in log_content or "ERROR:" in log_content:
                    self.status_var.set("✗ Failed")
                    self.update_status('error')
                    self.log_message("=" * 80, 'error')
                    self.log_message("✗ OPTIMIZATION FAILED", 'error')
                    self.log_message("=" * 80, 'error')
                else:
                    self.status_var.set("✓ Completed")
                    self.update_status('completed')
                    self.log_message("")
                    self.log_message("=" * 80, 'success')
                    self.log_message("✓ OPTIMIZATION COMPLETED", 'success')
                    self.log_message("=" * 80, 'success')
            else:
                self.root.after(100, self.monitor_process)

    def log_message(self, message: str, tag=''):
        """
        Add message to results log with optional tag for coloring.
        Args:
            message: The string message to add.
            tag: The style tag (e.g., 'info', 'error') to apply.
        """
        def _update_log():
            self.results_text.config(state='normal')
            if tag:
                self.results_text.insert('end', f"{message}\n", tag)
            else:
                self.results_text.insert('end', f"{message}\n")
            self.results_text.see('end')
            self.results_text.config(state='disabled')

        self.root.after(0, _update_log)

    def clear_results(self):
        """Clear the results log"""
        self.results_text.config(state='normal')
        self.results_text.delete('1.0', 'end')
        self.results_text.config(state='disabled')

    def save_results(self):
        """Save results log to file"""
        filename = filedialog.asksaveasfilename(title="Save Results Log", defaultextension=".txt", filetypes=[("Text files", "*.txt"), ("All files", "*.*")])

        if filename:
            try:
                content = self.results_text.get('1.0', 'end-1c')
                with open(filename, 'w', encoding='utf-8') as f: # Added encoding
                    f.write(content)
                self.log_message(f"Log saved: {filename}", 'success')
                messagebox.showinfo("Success", "Results log saved!")
            except Exception as e:
                self.log_message(f"Save failed: {str(e)}", 'error')
                messagebox.showerror("Save Error", f"Failed: {str(e)}")

    def open_results_folder(self):
        """Open the results folder in file explorer"""
        try:
            results_dir = OptConfig.OUTPUT_BASE
            if not os.path.exists(results_dir):
                os.makedirs(results_dir, exist_ok=True)

            if os.name == 'nt':  # Windows
                os.startfile(results_dir)
            elif os.name == 'posix':  # macOS, Linux
                import subprocess
                subprocess.run(['open', results_dir] if sys.platform == 'darwin' else ['xdg-open', results_dir])

            self.log_message(f"Opened: {results_dir}", 'info')

        except Exception as e:
            self.log_message(f"Error: {str(e)}", 'error')
            messagebox.showerror("Error", f"Could not open folder: {str(e)}")


def main():
    """Main function to run the GUI"""
    try:
        root = tk.Tk()

        try:
            from ttkbootstrap import Window
            root = Window(themename="darkly")
        except ImportError:
            # Fallback to standard ttk if ttkbootstrap is not installed
            style = ttk.Style()
            style.theme_use('clam') # A modern default theme
            pass

        app = FuselageOptimizationGUI(root)
        root.mainloop()

    except Exception as e:
        print(f"Failed to start GUI: {e}")
        input("Press Enter to exit...")


if __name__ == "__main__":
    main()