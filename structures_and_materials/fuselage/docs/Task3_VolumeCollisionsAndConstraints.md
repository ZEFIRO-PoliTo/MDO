# Task 3 – Volume Collisions and Constraints

## Preface

This update continues the collaborative work from the previous tasks. I have built upon the existing fuselage generation and volume placement logic developed by *Michele*, extending it to include support for **cuboid/parallelepiped volumes**, **batch configuration generation**, and **enhanced collision and constraint checking**. The implementation remains **modular and readable**, ensuring that all of us can easily integrate, review, and further extend the new scripts in the main code.

## Description 

This task integrates previous fuselage generation and volume placement scripts to enable realistic placement of internal components (volumes) inside a fuselage. The code now supports both ellipsoids and cuboids/parallelepipeds, batch generation of multiple configurations, and advanced collision and constraint checking.

## Inputs

* **Fuselage parameters:**

  * Length, width, height (can be single values or ranges for batch mode)
* **Volumes:**

  * List of 10 volumes, each with:

    * Shape type (`ellipsoid` or `cuboid`)
    * Dimensions (length, width, height)
    * Position (x, y, z)
    * Optional constraints (e.g., minimum distance from other volumes)

## Outputs

* `.vsp3` files for each valid fuselage + volume configuration
* CSV log of volume positions and dimensions for each configuration
* CSV log of constraint checks and results
* JSON metadata for each batch (volume data, fuselage parameters, constraint status)

## Algorithm / Procedure

1. **Volume Definition:**

   * Volumes are defined using a dataclass supporting both ellipsoid and cuboid shapes.
2. **Placement:**

   * Volumes are placed inside the fuselage, ensuring they fit within the fuselage bounds.
3. **Collision Checking:**

   * Checks for collisions between volumes (ellipsoid or cuboid logic).
   * Checks that volumes do not intersect fuselage walls.
4. **Constraint Enforcement:**

   * Enforces minimum distance constraints between specified volume pairs.
5. **Batch Generation:**

   * Supports batch creation of multiple configurations using parameter ranges.
   * Each configuration is validated and exported if valid.
6. **Export:**

   * Valid configurations are exported to `.vsp3` files and logged in CSV/JSON files.

## Examples

* **Single Configuration:**

  * Fuselage: length=12m, width=3m, height=3m
  * Volumes: 10 cuboids with specified positions and constraints
  * Output: `fuselage_config1.vsp3`, `volumes_positions.csv`, `distance_constraints.csv`
* **Batch Mode:**

  * Generates 10+ configurations with varying fuselage and volume parameters
  * Outputs: Multiple `.vsp3` files and logs

## Limitations / Notes

* Cuboid collision logic uses axis-aligned bounding box (AABB) checks.
* Placement may require multiple attempts for dense configurations.
* Only basic shapes (ellipsoid, cuboid) are supported; more complex geometries may require further development.
* Future improvements: visualization, support for more volume shapes, automated OpenVSP validation.