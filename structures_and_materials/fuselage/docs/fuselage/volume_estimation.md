# Sub-Module: Fuselage Volume Estimation

## Purpose

This module calculates the total internal (wetted) volume of a given fuselage geometry. This value is used as part of the final combined score to penalize overly large designs.

## Inputs

* A `FuselageConfiguration` object (containing `length`, `width`, `height` in meters).
* OpenVSP API.

## Outputs

* A single float value for the total theoretical fuselage volume (`fuselage_volume`) in **cubic meters**.

## Logic/Methodology

The process is defined in the `compute_fuselage_volume_compgeom()` [line 342] function:

1.  **Model Creation:** A fuselage is created in OpenVSP using the dimensions in **meters** (`create_fuselage_from_dimensions`). This is different from the CD calculation, which uses feet (as said in the cd_calculation volume this is a requirement from openVsp).
2.  **Run CompGeom:** The `vsp.ComputeCompGeom()` function is called. This is OpenVSP's standard tool for calculating geometric properties.
3.  **Find Results:** The function finds the results ID for the `Comp_Geom` analysis.
4.  **Extract Volume:** It searches the results for the geometry named "FuselageGeom" and extracts the `Theo_Vol` (Theoretical Volume) value associated with it.
5.  **Return Value:** This volume (in m³) is returned.

## Pros and Cons

* **Pros:**
    * **Accurate:** Uses OpenVSP's core geometry engine for a precise volume calculation of the lofted shape, which is more accurate than a simple elliptical cylinder formula.
* **Cons:**
    * **None:** This is a straightforward and effective part of the process.