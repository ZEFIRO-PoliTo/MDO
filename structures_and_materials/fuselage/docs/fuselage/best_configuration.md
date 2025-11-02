# Sub-Module: Best Configuration Selection

## Purpose

This module takes all the configurations that have passed both the CD analysis and the volume placement, and calculates a final "Combined Score" to determine the overall best design.

## Inputs

* A list of `FuselageConfiguration` objects that have passed volume placement (i.e., `num_volumes_skipped == 0`).
* Each object has its `cd_value` and `fuselage_volume`.
* `OptConfig.CD_WEIGHT` (e.g., 0.7)
* `OptConfig.VOLUME_WEIGHT` (e.g., 0.3)
* `OptConfig.CD_THRESHOLD` (A baseline Cd value for normalization)
* `OptConfig.TOP_N_FINAL` (The final number of configurations to select)

## Outputs

* A final, sorted list of the `TOP_N_FINAL` best `FuselageConfiguration` objects.
* This list is used to generate the final summary files and VSP models.

## Logic/Methodology

The process is defined in the `calculate_combined_score()` [line 426] function, which is called for each valid configuration:

1.  **Filter:** The main loop in `run_optimization` first filters the list to include *only* configurations where `num_volumes_skipped == 0`.
2.  **Normalization:** The score is calculated using normalized values to combine the dissimilar metrics of Cd (dimensionless) and Volume (m³).
    a.  **CD Normalization:**
        `cd_normalized = cd_value / CD_THRESHOLD`
    b.  **Volume Normalization:** The *actual* fuselage volume is normalized against a *theoretical* bounding box volume.
        `theoretical_volume = length * width * height`
        `volume_normalized = fuselage_volume / theoretical_volume`
3.  **Weighted Score:** The final score is computed as a weighted sum. **A lower score is better.**
    `combined_score = (CD_WEIGHT * cd_normalized) + (VOLUME_WEIGHT * volume_normalized)`
4.  **Final Selection:**
    * All valid, scored configurations are sorted by `combined_score` (ascending).
    * The top `TOP_N_FINAL` configurations are selected from this sorted list.

## Pros and Cons

* **Pros:**
    * **Tunable:** The objective function is clear and easily tunable using the CD and Volume weights, allowing the user to prioritize drag reduction vs. size reduction.
    * **Combines Objectives:** Provides a logical way to blend the two competing objectives (low drag, low volume) into a single metric.
* **Cons:**
    * **Normalization Dependency:** The score is sensitive to the `CD_THRESHOLD` and the `theoretical_volume` calculation. An unusual `CD_THRESHOLD` could skew the results.
    * **Objective Function:** The score penalizes total fuselage volume, not *remaining* volume. This is an intentional design choice to favor smaller, sleeker fuselages, but it means a design with a large fuselage and a high `remaining_volume` would be penalized.