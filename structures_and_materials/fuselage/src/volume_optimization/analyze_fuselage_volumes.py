import os
import json
import math
from typing import List, Tuple
import pandas as pd 
import matplotlib.pyplot as plt

def compute_ellipsoid_volume(cross_sections: List[dict]) -> float:
    """
    Compute fuselage volume using ellipsoidal approximation from cross-section data
    """
    if len(cross_sections) < 2:
        return 0.0  # fallback, should never happen if JSON corretto

    volume = 0.0
    for i in range(len(cross_sections) - 1):
        cs1 = cross_sections[i]
        cs2 = cross_sections[i + 1]
        dx = cs2["x"] - cs1["x"]
        ry = (cs1["radius_y"] + cs2["radius_y"]) / 2
        rz = (cs1["radius_z"] + cs2["radius_z"]) / 2
        volume += math.pi * ry * rz * dx
    return volume

def analyze_configuration(json_file: str) -> dict:
    """
    Reads a configuration JSON, computes:
    - fuselage volume
    - total cubes volume
    - fuselage minus cubes
    """
    with open(json_file, "r") as f:
        config = json.load(f)

    cross_sections = config.get("cross_sections_real", [])
    volumes = config.get("volumes", [])

    fuselage_volume = compute_ellipsoid_volume(cross_sections)
    total_cubes_volume = sum([v["length"] * v["width"] * v["height"] for v in volumes])
    remaining_volume = fuselage_volume - total_cubes_volume

    return {
        "configuration": config.get("configuration", "unknown"),
        "fuselage_volume": fuselage_volume,
        "cubes_volume": total_cubes_volume,
        "remaining_volume": remaining_volume,
        "json_file": json_file
    }

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
    return results

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

    # Istogramma dei volumi residui
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


def main():
    configs_dir = "configs"  # directory with configs
    results = analyze_all_configurations(configs_dir)
    if not results:
        print("No configurations found in", configs_dir)
        return

    print(f"Found {len(results)} configurations")
    # print summary
    for r in results:
        print(f"Config {r['configuration']}: fuselage {r['fuselage_volume']:.2f} m³, "
              f"cubes {r['cubes_volume']:.3f} m³, remaining {r['remaining_volume']:.2f} m³")

    # Save top 20 CSV
    save_top20_csv(results, os.path.join(configs_dir, "top20_fuselage_volumes.csv"))
    #Call plots
    plot_volume_distributions(results, top_n=20)

if __name__ == "__main__":
    main()
