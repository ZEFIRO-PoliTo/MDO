**I will write proper documentation when we have some type of main code**

Testing process :  
input_fuse.vsp3 -> Volumes_Optimization.py -> analyze_fuselage_volumes.py

`analyze_fuselage_volumes.py`  
Script to analyze fuselage configurations from JSON files and OpenVSP models:
- compute_fuselage_volume_from_vsp: Compute exact fuselage volume using OpenVSP's `ComputeCompGeom()` and `FindLatestResultsID()`
- find_vsp_file: Search for a `.vsp3` file in a configuration directory
- analyze_configuration: Compute fuselage, cubes, and remaining volume for one configuration
- analyze_all_configurations: Recursively process all configurations in a directory and subdirectories
- save_top20_csv: Save top 20 configurations sorted by remaining volume
- plot_volume_distributions: Visualize distributions of fuselage, cubes, and remaining volumes
- main: Run analysis for all configurations, save CSV, and show plots
