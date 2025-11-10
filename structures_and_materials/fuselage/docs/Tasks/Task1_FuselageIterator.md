# Task 1 – Fuselage Iterator

## Description

This task implements a Python script that generates fuselage geometries using the OpenVSP Python API. The script is designed to support parametric studies for Multi-Disciplinary Optimization (MDO) by iterating over ranges of fuselage parameters (length, width, height) and exporting each geometry for further analysis.

## Inputs

- **Fuselage parameters:**
  - Length (x)
  - Width (y)
  - Height (z)
- **Parameter ranges:**
  - Each parameter can be specified as a range (e.g., length ∈ [10, 15] m).
- **Step size or number of samples:**
  - Optional step size for grid sampling, or number of random samples.
- **Command-line arguments:**
  - Single fuselage: `python create_fuselage.py <length> <width> <height> [output_file]`
  - Batch random: `python create_fuselage.py --batch`
  - Batch grid: `python create_fuselage.py --batch --steps <length_step> <width_step> <height_step>`

## Outputs

- **VSP3 files:**  
  Each generated fuselage is exported as a `.vsp3` file, compatible with the OpenVSP GUI.
- **CSV log:**  
  Parameters for each fuselage are logged in `fuselage_batch_log.csv` for traceability and validation.

## Algorithm / Procedure

1. **Parameter Parsing:**  
   Script reads parameters and ranges from command-line arguments.
2. **Batch Generation:**  
   - If step size is provided, grid sampling is used.
   - Otherwise, random sampling within ranges.
3. **Fuselage Creation:**  
   For each parameter set, a fuselage is generated using OpenVSP API.
4. **Volume Placement & Constraints:**  
   Optionally, volumes are placed inside the fuselage and constraints (collision, separation) are checked.
5. **Optimization:**  
   Fuselage is resized to the smallest possible size that contains all volumes, with a margin.
6. **Export:**  
   Geometry is saved as `.vsp3` and parameters are logged in a CSV file.

## Examples

# Single Fuselage
```bash
python create_fuselage.py 12.5 3.0 3.0 fuselage_example.vsp3
```

# Batch Generation (Random)
```bash
python create_fuselage.py --batch
```

# Batch Generation (Grid)
```bash
python create_fuselage.py --batch --steps 1.0 0.5 0.5
```

# Example CSV Log

| filename       | length | width | height |
|----------------|--------|-------|--------|
| fuselage_1.vsp3 | 10.5   | 2.3   | 2.8    |
| fuselage_2.vsp3 | 12.0   | 3.1   | 3.5    |
| ...            | ...    | ...   | ...    |


# Limitations / Notes

- **OpenVSP API**: Requires OpenVSP and its Python API to be installed and available.  
- **Validation**: All generated `.vsp3` files should be opened in OpenVSP to verify geometry.  
- **Future Improvements**:  
  - Add more advanced constraints and optimization features.  
  - Integrate with downstream MDO pipeline tasks.  
  - Add visualization and reporting features.  