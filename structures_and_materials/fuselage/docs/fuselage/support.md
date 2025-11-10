# Sub-Module: Data Structures, Plots, and Utilities

## Purpose

This module documents the essential support components of the `Aero_analysis.py` script. These include:

1.  **Data Classes:** The `dataclass` structures used to organize and manage data cleanly and robustly during optimization.
2.  **Visualization:** The `generate_plots` function, which creates a visual summary of the results.
3.  **Utilities (I/O):** Helper functions for loading input files and saving output files.

---

## 1. Data Classes

The entire script relies on a series of Python `dataclasses` to ensure data is organized, consistent, and easy to pass between functions.

* `OptConfig`:
    **Purpose:** This is the global configuration class. It holds *all* input parameters (ranges, weights, file paths, aerodynamic parameters) as static variables. The `update_from_gui` function in this class allows the GUI to overwrite the default values.

* `BoxVolume`:
    **Purpose:** Represents a single internal volume (box). It is used in two ways:
    1.  As a **definition** (loaded from JSON) with `id`, `length`, `width`, `height`, and `distance_constraints`.
    2.  As a **placed object** after packing, which also includes its `x`, `y`, `z` coordinates.

* `CrossSection`:
    **Purpose:** A simple data structure that stores the properties of a single 2D "slice" of the fuselage (radius in Y, radius in Z, center, etc.) at a specific longitudinal `u` position.

* `FuselageBounds`:
    **Purpose:** This is the complete geometric "map" of a fuselage. It contains the total length, min/max coordinates, and, most importantly, a `List[CrossSection]`. This map is used by the `box_placement` module to quickly check if a point is inside the fuselage. 

* `FuselageConfiguration`:
    **Purpose:** This is the most important data object. It represents **a single candidate configuration** and tracks its entire lifecycle:
    * Its parameters (`config_id`, `length`, `width`, `height`).
    * Its results (`cd_value`, `fuselage_volume`, `combined_score`).
    * The placed volumes (`volumes_placed`) and skipped ones (`num_volumes_skipped`).
    * The paths to its output files (`vsp_file`).

---

## 2. Visualization (Plots)

* **Function:** `generate_plots()`
* **Purpose:** To create a single summary `.png` image that visualizes the results of the entire optimization run. It helps the user understand *why* certain configurations won.
* **Input:** The list of *all* analyzed configurations (for CD) and the list of the *top* final configurations.
* **Output:** A `step_based_optimization_analysis.png` file saved in the main results folder.
* **Logic (Plots Created):**
    1.  **CD Distribution:** A histogram showing how many designs had high or low CD.
    2.  **Top Configurations:** A bar chart ranking the final top configurations by their `combined_score`.
    3.  **CD vs. Volume:** A scatter plot showing the relationship between CD (primary metric) and volume (secondary metric).
    4.  **CD Selection Process:** A plot showing the CD of *all* configurations, highlighting the `TOP_N_CD` cutoff used for Stage 2.
    5.  **Score Breakdown:** A bar chart showing, for each top design, how much of the final score came from CD and how much from volume.
    6.  **Summary Text:** A text box with the main metrics (e.g., "Best CD," "Total Configurations," "Weights Used").

---

## 3. Utilities (Input/Output)

* `load_volumes_from_file(filepath: str)`:
    **Purpose:** To read the `volume_file.json` provided by the user.
    **Logic:** Opens the JSON file, parses it, and uses the data to create a `List[BoxVolume]`, turning the JSON dictionaries into the `BoxVolume` objects used by the program. It also performs validation (already covered in `gui.md`).

* `save_volumes_positions_csv(...)` and `save_constraints_csv(...)`:
    **Purpose:** To create the output `.csv` files for the final results.
    **Logic:** They iterate over the final list of `volumes_placed` and write their data (coordinates, dimensions, constraint verification) into an easy-to-read CSV tabular format for the user.