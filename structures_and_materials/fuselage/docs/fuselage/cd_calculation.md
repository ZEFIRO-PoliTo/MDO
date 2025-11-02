# Sub-Module: CD Calculation

## Purpose

This module performs the aerodynamic analysis for a single, empty fuselage geometry to determine its parasite drag coefficient (Cd). This value is the primary metric used for the Stage 1 filtering. (We need to use an empty fuselage otherwise openvsp crashes)

## Inputs

* A `FuselageConfiguration` object (containing `length`, `width`, `height` in meters).
* OpenVSP API.
* Global aerodynamic parameters from `OptConfig`:
    * `ANALYSIS_SREF` (Reference Area, m²)
    * `ANALYSIS_VINF` (Velocity, m/s)
    * `ANALYSIS_ALTITUDE` (Altitude, m)
    * `ANALYSIS_DELTA_TEMP` (Temperature offset, K)

## Outputs

* A single float value for the total parasite drag coefficient (`cd_value`).

## Logic/Methodology

The process is defined in the `run_parasite_drag_analysis_feet()` (line 370) function:

1.  **Unit Conversion (Critical):** The OpenVSP API functions for drag analysis operate in **feet**. The input `length`, `width`, and `height` (which are in meters) are first converted to feet using the `METERS_TO_FEET` constant (3.28084).
2.  **Model Creation:** A new, empty fuselage is created in OpenVSP using the dimensions in *feet* (`create_fuselage_in_feet`). The fuselage is defined with a `XS_POINT` at the nose and tail and `XS_ELLIPSE` for the main body ( i.e. the cylinder + "cones" shape).
3.  **Parameter Conversion:** The aerodynamic parameters (`Sref`, `Vinf`, `Altitude`) are also converted from metric to imperial units (e.g., m² to ft², m/s to ft/s) before being passed to OpenVSP.
4.  **Run Analysis:**
    * The `ParasiteDrag` analysis is selected.
    * Inputs like `Sref`, `Vinf`, and `Altitude` are set.
    * The analysis is executed using `vsp.ExecAnalysis()`.
5.  **Extract Result:** The `Total_CD_Total` value is read from the analysis results. This single, dimensionless float is returned.
6.  **Cleanup:** The VSP model is cleared to prepare for the next iteration.

## Pros and Cons

* **Pros:**
    * **Fast:** OpenVSP's `ParasiteDrag` analysis is very fast, making it ideal for iterating over thousands of configurations.
    * **Standardized:** Provides a consistent, automated method for evaluating aerodynamic performance.
* **Cons:**
    * **Simplified:** This is a *parasite drag* calculation on an *isolated fuselage*. It does not account for induced drag, wave drag, or interference drag from other components (like wings or engines).
    * **Unit Sensitivity:** The user needs to remember which units are accepted