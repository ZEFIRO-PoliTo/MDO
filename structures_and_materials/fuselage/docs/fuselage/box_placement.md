# Sub-Module: Box Placement Logic

## Purpose

This is the core "packing" algorithm. Its purpose is to determine if a given set of required internal volumes (boxes) can successfully fit inside a specific fuselage geometry, while respecting all placement constraints.

This is the most computationally expensive part of the analysis, which is why it is only run on the `TOP_N_CD` configurations.

## Inputs

* `bounds`: A `FuselageBounds` object, which contains a detailed geometric map (cross-sections) of the fuselage.
* `base_volumes`: A list of `BoxVolume` objects loaded from the user's `volume_file.json`.
* `OptConfig.MAX_PLACEMENT_ATTEMPTS`: (e.g., 1000) How many random positions to try for *each box* before giving up.
* `OptConfig.SAFETY_MARGIN`: (e.g., 0.8) A factor applied to the fuselage radius to keep boxes away from the skin.
* `OptConfig.COLLISION_MARGIN`: (e.g., 1.1) A multiplier on box dimensions for collision checking.

## Outputs

* `placed`: A list of `BoxVolume` objects that have been successfully placed (including `x`, `y`, `z` coordinates).
* `skipped`: An integer count (`num_volumes_skipped`) of how many boxes *failed* to be placed. A configuration is only considered valid if `skipped == 0`.

## Logic/Methodology

The process is defined in the `place_volumes_in_fuselage()` [line 758] function:

1.  **Load Volumes:** The list of `BoxVolume` definitions is loaded from the JSON file.
2.  **Sort Volumes:** The boxes are sorted by volume, from **largest to smallest**. This is to make placement easier by placing the most difficult objects first.
3.  **Placement Loop:** The function iterates through each box to be placed:
    a.  **Attempt Loop:** It enters a `while` loop that runs up to `MAX_PLACEMENT_ATTEMPTS` times.
    b.  **Generate Random Position:**:
    - A random longitudinal position `u` (from 0 to 1) is chosen.
    - The local fuselage cross-section (radius, center) at that `u` is found.
    - A random `y` and `z` coordinate is chosen within the "safe" area of that cross-section (i.e., inside the `SAFETY_MARGIN`).

    c.  **Create Candidate:** A temporary `BoxVolume` object is created with this new `x, y, z` position.
    d.  **Validation:** This candidate position is checked using `is_valid_placement()`, which performs three critical checks:
    - **Inside Fuselage (`is_box_inside_fuselage`):** Checks if all surface points (e.g., 8 corners) of the box are inside the fuselage's `SAFETY_MARGIN`
    - **Collision (`boxes_collide`):** Checks if the new box overlaps with any of the *already placed* boxes, using the `COLLISION_MARGIN`.
    - **Distance (`respects_distance_constraints`):** Checks if the box satisfies all `distance_constraints` defined in the JSON file (e.g., "must be 2.0m away from volume 5").

    e.  **On Success:** If all checks pass, the box is added to the `placed` list, and the Attempt Loop is broken.
    f.  **On Failure:** If the `MAX_PLACEMENT_ATTEMPTS` limit is reached without a valid placement, the box is considered `skipped`, and the function moves to the next box. (this is ver unlikely)

## Pros and Cons

* **Pros:**
    * **Handles Complex Constraints:** Can validate placement against the fuselage skin, other boxes, and arbitrary distance rules simultaneously.
    * **Flexible:** Can accommodate any number of boxes and constraints defined in the JSON file.
* **Cons:**
    * **Stochastic (Random):** The algorithm is not deterministic. A failed placement (`skipped > 0`) does **not** mean the configuration is impossible; it only means the *random placer* failed to find a valid spot in the given number of attempts. (very unlikely to fail)
    * **Computationally Expensive:** This process is slow, as it involves many random attempts and geometric checks per box.
    * **Local Optima:** The "largest-first" sorting is a heuristic and may not lead to the most compact global packing solution.