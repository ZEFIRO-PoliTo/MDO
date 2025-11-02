# Sub-Module: Iteration Logic

## Purpose

This module is responsible for generating the initial, complete list of all possible fuselage geometries to be tested. It translates the user's step-based ranges into a discrete set of `FuselageConfiguration` objects.

## Inputs

* `OptConfig.LENGTH_MIN`, `OptConfig.LENGTH_MAX`, `OptConfig.LENGTH_STEP`
* `OptConfig.WIDTH_MIN`, `OptConfig.WIDTH_MAX`, `OptConfig.WIDTH_STEP`
* `OptConfig.HEIGHT_MIN`, `OptConfig.HEIGHT_MAX`, `OptConfig.HEIGHT_STEP`

**Note:** The script (`Aero_analysis.py`) and GUI (`GUI.py`) are configured to force `WIDTH = HEIGHT` by using a **Radius (R)** parameter. The `r_min`, `r_max`, and `r_step` from the GUI are used to set identical width and height parameters.

## Outputs

* A list of `FuselageConfiguration` data classes.
* Each class instance contains a unique `config_id`, `length`, `width`, and `height`.

## Logic/Methodology

The logic is implemented in the `generate_step_based_configurations()` (line 855 ) function:

1.  **Generate Lengths:** A list of all valid lengths is created by starting at `LENGTH_MIN` and incrementing by `LENGTH_STEP` until `LENGTH_MAX` is reached.
2.  **Generate Widths:** A list of all valid widths is created in the same way, using the `WIDTH` parameters.
3.  **Create Combinations:** The function performs a nested loop (Cartesian product) of all lengths and all widths.
4.  **Set Height:** For each combination, the `height` is set to be identical to the `width`.
5.  **Instantiate:** A `FuselageConfiguration` object is created for each combination and added to the master list.

> **Example:**
> * Lengths: [18.0, 19.0]
> * Widths: [3.0, 3.2]
> * Resulting Configurations:
>     1.  {L: 18.0, W: 3.0, H: 3.0}
>     2.  {L: 18.0, W: 3.2, H: 3.2}
>     3.  {L: 19.0, W: 3.0, H: 3.0}
>     4.  {L: 19.0, W: 3.2, H: 3.2}

## Pros and Cons

* **Pros:**
    - The logic is easy to understand and debug.
    - It guarantees that every single combination within the defined steps is evaluated, ensuring no potential optimum is missed (within the given discretization).
* **Cons:**
    - The total number of configurations (`L_count * W_count`) can become extremely large if the step sizes are small, leading to very long computation times for CD Analysis.