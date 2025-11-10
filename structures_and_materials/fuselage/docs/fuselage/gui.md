# Sub-Module: Graphical User Interface (GUI)

## Purpose

This module provides the graphical front-end for the fuselage optimization process, built using `tkinter`. Its primary functions are:

* To provide an intuitive interface for setting all `OptConfig` parameters.
* To perform robust, multi-level validation on all user inputs *before* launching the analysis.
* To launch the `Aero_analysis.py` script in a decoupled, non-blocking subprocess.
* To capture and display the real-time console output from the analysis.
* To provide controls for starting, stopping, and managing the optimization.

## Inputs

The GUI captures all user-provided parameters, which serve as the inputs for the `Aero_analysis.py` script:

* **Main Parameters:** Length (min, max, step), Radius (min, max, step).
* **Input Data:** The file path to the `volume_file.json`.
* **Advanced Parameters:**
    * Optimization: Safety Margin, CD Weight, Volume Weight.
    * Filtering: `TOP_N_CD` and `TOP_N_FINAL`.
    * Aerodynamic: Sref, Velocity, Altitude, and &Delta;Temp.

## Outputs

* **To the User:**
    * A real-time log of the optimization's console output, displayed in the GUI's text area.
    * Status messages (e.g., "Running," "Completed," "Error").
    * Pop-up dialogs for validation errors or success messages.
* **To the Analysis Script:**
    * A single Python dictionary (`config_dict`) containing all validated parameters. This is passed to the `OptConfig.update_from_gui()` method in the subprocess.

## Logic/Methodology

The GUI's operation is based on several key components:

1.  **Parameter Collection:** The interface is built with `tkinter` widgets, with user-set values (like `l_min`, `r_min`, ecc.) stored in `tk.DoubleVar` or `tk.StringVar` variables.

2.  **Pre-Run Validation (`validate_parameters`):** Before the run, this function performs a comprehensive check on all parameters.
    * It verifies logical ranges (e.g., `l_max >= l_min`).
    * It checks bounds (e.g., weights must sum to 1.0).
    * It validates parameter hierarchy (e.g., `TOP_N_FINAL` cannot be greater than `TOP_N_CD`).
    * It triggers the separate `validate_volume_file` check.

3.  **Volume File Validation (`validate_volume_file`):** This is a validation of the selected JSON file.
    * It parses the JSON to ensure it's not malformed.
    * It checks the schema: it must be a *list* of *dictionaries*.
    * It verifies that all required fields (`id`, `length`, `width`, `height`) are present and have positive, numeric values.
    * It checks for duplicate `id` values.
    * It iterates through all `distance_constraints` and confirms that every `target_id` refers to an `id` that actually exists in the file.

4.  **File Monitoring (`check_file_changes`):**
    * After a valid volume file is loaded, the GUI monitors it for external changes (by checking its modification time).
    * If the file is modified (e.g., the user edits it in a separate program), the GUI automatically re-runs the `validate_volume_file` function and warns the user if the file is now invalid.

5.  **Decoupled Subprocess (`OptimizationProcess`):**
    * To prevent the GUI from freezing, the `Aero_analysis.py` script is **not** run in the same process.
    * When "Run" is clicked, the GUI creates a dynamic script string (`_create_runner_script`).
    * This script string (which imports `Aero_analysis` and calls `run_optimization()`) is executed in a new, separate process using `subprocess.Popen`.

6.  **Real-Time Logging (`monitor_process`):**
    * A separate thread (`_read_output`) is launched to read the `stdout` of the subprocess in real-time.
    * It places each line of output into a `queue.Queue`.
    * The main `tkinter` GUI loop periodically checks this queue and displays any new messages in the log, ensuring a responsive, non-blocking interface.

## Pros and Cons

* **Pros:**
    * **Responsive:** The subprocess/threading/queue architecture ensures the GUI never freezes, even during a long optimization.
    * **Robust:** The multi-level validation (on both parameters and the JSON file schema) is extremely robust, preventing the analysis from starting with invalid data.
    * **Decoupled:** The GUI (`GUI.py`) and the core logic (`Aero_analysis.py`) are completely separate, which is excellent for maintainability.
    * **User-Friendly:** Features like real-time logging and active file monitoring provide excellent, immediate feedback to the user.
* **Cons:**
    * **None** : works