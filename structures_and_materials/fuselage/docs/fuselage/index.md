# Fuselage Optimization Module

## Purpose

The fuselage optimization module is a core component of the MDO software designed to find the optimal fuselage shape that minimizes aerodynamic drag while successfully enclosing a set of required internal volumes (e.g., cargo, avionics, systems).

It operates on a **step-based, two-stage optimization process**. This approach is designed for computational efficiency:
1.  **Aerodynamic-First Filtering:** It first generates *all* possible fuselage geometries based on the input ranges and evaluates their parasite drag (Cd).
2.  **Volume-Based Selection:** It then takes only the **Top N** best-performing configurations (lowest Cd) and subjects them to the computationally expensive volume placement analysis.

This ensures that the complex and time-consuming box-placement algorithm is only run on configurations that are already aerodynamically promising.

## Important
There is an important step to run the code: 
1. The only way to make boxes/custom geoms work is to manually copy the `OpenVSP-3.45.4-win64\CustomScripts` into `C:\Users\username`, this seems hardcoded by the api for some reason. 
2. The file to run is `GUI.py` in `structures_and_materials\fuselage\src\aerodynamic_analysis`

Note:  
- The three `Unnamed.csv/txt` are OpenVsp dump files, they need to be ignored
- Use meters (the script automatically converts in feet when needed)

**Important clarification on Radius/Width/Height**
- The frontend (GUI) requires a Radius and a Length (a fusolage can be seen as a cylinder for the most part)
- The backend (script) converts radius into width and height (which are forced to be equal = radius) because openvsp needs these to modify cross sections (`Ellipse_Width `,` Ellipse_Height`)


## Workflow

The high-level logic, as executed by the `run_optimization()` function, is as follows:

1.  **Generate Configurations:** Create a list of all possible fuselage geometries based on the user-defined step parameters.
    * *See: `iteration_logic.md`*
2.  **Run CD Analysis (Stage 1):** Iterate through *every* configuration, create it in OpenVSP (in feet), and run the `ParasiteDrag` analysis to get its Cd value.
    * *See: `cd_calculation.md`*
3.  **Filter by CD:** Sort all configurations by their Cd value (lowest is best) and select the `TOP_N_CD` configurations.
4.  **Run Volume Analysis (Stage 2):** For *only* this reduced set of top configurations:
    a.  Compute the total internal volume of the fuselage.
    * *See: `volume_estimation.md`*
    b.  Attempt to place all required internal boxes, respecting collision, safety, and distance constraints.
    * *See: `box_placement.md`*
5.  **Final Scoring:** From the configurations that successfully placed *all* volumes, calculate a final `combined_score` based on weighted CD and fuselage volume.
6.  **Select Best:** Sort by the final score and select the `TOP_N_FINAL` best configurations.
    * *See: `best_configuration.md`*

## Inputs

Primary inputs are provided via the `OptConfig` class, which is populated by the GUI:

* **Geometric Ranges:**
    * `l_min`, `l_max`, `l_step` (Length in meters)
    * `r_min`, `r_max`, `r_step` (Radius in meters; used for Width and Height)
* **Volume Definition File:** A JSON file path (`volume_file`, we decided to use json because it's easier to use than other type of files like csv) specifying all internal boxes, their dimensions, and their distance constraints.
* **Filtering Parameters:**
    * `TOP_N_CD`: How many configurations to select after the CD analysis (e.g., 10).
    * `TOP_N_FINAL`: How many final configurations to save (e.g., 5).
* **Scoring Weights:**
    * `CD_WEIGHT`: The importance of the drag coefficient (e.g., 0.7).
    * `VOLUME_WEIGHT`: The importance of the fuselage volume (e.g., 0.3).
* **Aerodynamic Parameters:**
    * `ANALYSIS_SREF`: Reference area (m²).
    * `ANALYSIS_VINF`: Freestream velocity (m/s).
    * `ANALYSIS_ALTITUDE`: Analysis altitude (m).
* **Placement Parameters:**
    * `SAFETY_MARGIN`: How close to the fuselage skin boxes can be placed (e.g., 0.8 = 80% of radius).
    * `COLLISION_MARGIN`: Safety factor for collision between boxes (e.g., 1.1 = 10% margin).

## Outputs

The module generates a time-stamped results folder (`optimization_results/run_.../`) containing:

* **`top_configurations/`:** A sub-folder for each of the `TOP_N_FINAL` configurations, which contains:
    * `..._with_volumes.vsp3`: OpenVSP file (in meters) with the fuselage and all placed boxes.
    * `..._empty.vsp3`: OpenVSP file (in feet) used for the Cd analysis.
    * `config_data.json`: A JSON file with all metrics for this specific configuration.
    * `volumes_positions.csv`: A CSV table of the X,Y,Z coordinates for all placed boxes.
    * `distance_constraints.csv`: A CSV report verifying all distance constraints.
* **`optimization_summary.csv`:** A top-level summary of the `TOP_N_FINAL` configurations and their scores.
* **`cd_analysis_all_configs.csv`:** A full report of all configurations and their initial Cd values.
* **`..._analysis.png`:** A plot visualizing the optimization results.