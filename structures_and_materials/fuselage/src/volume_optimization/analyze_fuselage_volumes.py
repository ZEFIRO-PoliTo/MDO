import os
import json
import math
from typing import List, Dict
import pandas as pd
import matplotlib.pyplot as plt

import openvsp as vsp


# ============================================================
#  Compute fuselage volume directly from OpenVSP (CompGeom)
# ============================================================
def compute_fuselage_volume_from_vsp(vsp_file: str, component_name="FuselageGeom") -> float:
    """
    Use OpenVSP's ComputeCompGeom() to compute the true volume of a component.
    Returns the 'Theo_Vol' of the specified geometry (e.g., FuselageGeom).
    """
    if not os.path.exists(vsp_file):
        print(f"[Error] File not found: {vsp_file}")
        return 0.0

    # Reset model
    vsp.ClearVSPModel()

    # Load .vsp3 file
    vsp.ReadVSPFile(vsp_file)

    # Use ComputeCompGeom to mesh and compute geometry
    mesh_id = vsp.ComputeCompGeom(vsp.SET_ALL, False, 0)
    
    # Find the results ID from CompGeom analysis
    result_id = vsp.FindLatestResultsID("Comp_Geom")
    
    if not result_id:
        print(f"[Error] Could not find Comp_Geom results")
        return 0.0

    # Extract results using correct field names
    geom_names = vsp.GetStringResults(result_id, "Comp_Name")
    theo_vols = vsp.GetDoubleResults(result_id, "Theo_Vol")

    # Find the fuselage (or matching component)
    for name, vol in zip(geom_names, theo_vols):
        if component_name.lower() in name.lower():
            return vol

    print(f"[Warning] Component '{component_name}' not found in {vsp_file}")
    return 0.0


# ============================================================
#  Find VSP file in configuration directory
# ============================================================
def find_vsp_file(conf_path: str) -> str:
    """
    Search for any .vsp3 file in the configuration directory.
    Returns the first .vsp3 file found, or empty string if none found.
    """
    for file in os.listdir(conf_path):
        if file.endswith('.vsp3'):
            return os.path.join(conf_path, file)
    return ""


# ============================================================
#  Analyze a single configuration
# ============================================================
def analyze_configuration(json_file: str) -> dict:
    """
    Reads a configuration JSON, computes:
    - fuselage volume via OpenVSP
    - total cubes volume
    - fuselage minus cubes
    """
    with open(json_file, "r") as f:
        config = json.load(f)

    # Get the vsp file path (search for any .vsp3 file in directory)
    conf_path = os.path.dirname(json_file)
    vsp_file = find_vsp_file(conf_path)

    # Compute fuselage volume via OpenVSP
    fuselage_volume = 0.0
    if vsp_file:
        fuselage_volume = compute_fuselage_volume_from_vsp(vsp_file)
    else:
        print(f"[Warning] No .vsp3 file found in {conf_path}")

    # Compute total cube volume
    volumes = config.get("volumes", [])
    total_cubes_volume = sum([v["length"] * v["width"] * v["height"] for v in volumes])
    remaining_volume = fuselage_volume - total_cubes_volume

    return {
        "configuration": config.get("configuration", "unknown"),
        "fuselage_volume": fuselage_volume,
        "cubes_volume": total_cubes_volume,
        "remaining_volume": remaining_volume,
        "json_file": json_file
    }


# ============================================================
#  Analyze all configurations
# ============================================================
def analyze_all_configurations(configs_dir: str) -> List[dict]:
    """
    Scan all conf*/volumes_data.json in configs_dir and analyze them
    """
    results = []
    for conf in os.listdir(configs_dir):
        conf_path = os.path.join(configs_dir, conf)
        if os.path.isdir(conf_path):
            json_file = os.path.join(conf_path, "volumes_data.json")
            if os.path.exists(json_file):
                res = analyze_configuration(json_file)
                results.append(res)
                print(f"[OK] Processed: {conf}")
    return results


# ============================================================
#  Save and plot results
# ============================================================
def save_top20_csv(results: List[dict], output_file: str):
    """
    Save top 20 fuselage configurations sorted by remaining volume (ascending)
    """
    df = pd.DataFrame(results)
    df_sorted = df.sort_values("remaining_volume")
    top20 = df_sorted.head(20)
    top20.to_csv(output_file, index=False)
    print(f"Top 20 configurations saved to {output_file}")


def plot_volume_distributions(results: List[dict], top_n: int = 20):
    """
    Plot histograms and scatter plots for fuselage and cubes volumes.
    Highlights the top N configurations.
    """
    # Data extraction
    fuselage_volumes = [r["fuselage_volume"] for r in results]
    cubes_volumes = [r["cubes_volume"] for r in results]
    remaining_volumes = [r["remaining_volume"] for r in results]

    # Find top N
    sorted_results = sorted(results, key=lambda r: r["remaining_volume"])
    top_results = sorted_results[:top_n]
    top_remaining = [r["remaining_volume"] for r in top_results]

    # Histogram of remaining volumes
    plt.figure(figsize=(10, 5))
    plt.hist(remaining_volumes, bins='auto', alpha=0.7, color="skyblue", label="All configurations")
    plt.hist(top_remaining, bins=20, alpha=0.9, color="orange", label=f"Top {top_n}")
    plt.xlabel("Fuselage Volume minus Cubes (m³)")
    plt.ylabel("Count")
    plt.title("Distribution of Remaining Fuselage Volumes")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

    # Scatter plot fuselage vs cubes volume
    plt.figure(figsize=(8, 6))
    plt.scatter(fuselage_volumes, cubes_volumes, alpha=0.6, label="All configs")
    plt.scatter([r["fuselage_volume"] for r in top_results],
                [r["cubes_volume"] for r in top_results],
                color="red", label=f"Top {top_n}", edgecolor="k", s=80)
    plt.xlabel("Fuselage Volume (m³)")
    plt.ylabel("Total Cubes Volume (m³)")
    plt.title("Fuselage Volume vs Cubes Volume")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


# ============================================================
#  Main entry point
# ============================================================
def main():
    configs_dir = "configs"  # directory with configs
    results = analyze_all_configurations(configs_dir)
    
    if not results:
        print("  No configurations found in", configs_dir)
        return

    print(f"\n Found {len(results)} configurations\n")
    
    # Print summary
    for r in results:
        print(f"Config {r['configuration']}: fuselage {r['fuselage_volume']:.2f} m³, "
              f"cubes {r['cubes_volume']:.3f} m³, remaining {r['remaining_volume']:.2f} m³")

    # Save top 20 CSV
    save_top20_csv(results, os.path.join(configs_dir, "top20_fuselage_volumes.csv"))
    
    # Call plots
    plot_volume_distributions(results, top_n=20)


if __name__ == "__main__":
    main()