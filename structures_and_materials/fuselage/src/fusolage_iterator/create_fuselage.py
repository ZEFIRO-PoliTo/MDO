from openvsp import vsp
import random
import csv
import sys
from typing import Tuple, List


def create_fuselage(length: float, width: float, height: float, filename: str = "fuselage.vsp3") -> None:
    """
    Create a fuselage geometry with given length, width, and height using OpenVSP,
    and export it as a VSP3 file.
    """
    vsp.ClearVSPModel()
    fid = vsp.AddGeom("FUSELAGE", "")

    # Set fuselage parameters
    vsp.SetParmValUpdate(fid, "Length", "Design", length)
    vsp.SetParmValUpdate(fid, "Diameter", "Design", width)   # fuselage width ≈ diameter
    vsp.SetParmValUpdate(fid, "Height", "Design", height)    # if fuselage supports height

    vsp.Update()
    vsp.WriteVSPFile(filename, vsp.SET_ALL)
    print(f"Fuselage saved to {filename}")


def batch_generate_fuselages(
    length_range: Tuple[float, float],
    width_range: Tuple[float, float],
    height_range: Tuple[float, float],
    num_samples: int = 10,
    output_prefix: str = "fuselage",
    log_csv: str = "fuselage_batch_log.csv",
    step_size: Tuple[float, float, float] = None
) -> None:
    """
    Generate multiple fuselages by sampling parameters within given ranges.
    Save each fuselage as a VSP3 file and log parameters in a CSV file.

    - If step_size is provided, use grid sampling.
    - Otherwise, use random sampling.
    """
    records: List[List[float]] = []

    if step_size:
        # Grid sampling
        lengths = [length_range[0] + i * step_size[0]
                   for i in range(int((length_range[1] - length_range[0]) / step_size[0]) + 1)]
        widths = [width_range[0] + i * step_size[1]
                  for i in range(int((width_range[1] - width_range[0]) / step_size[1]) + 1)]
        heights = [height_range[0] + i * step_size[2]
                   for i in range(int((height_range[1] - height_range[0]) / step_size[2]) + 1)]

        idx = 1
        for l in lengths:
            for w in widths:
                for h in heights:
                    filename = f"{output_prefix}_{idx}.vsp3"
                    print(f"\nGenerating fuselage {idx}: Length={l:.2f}, Width={w:.2f}, Height={h:.2f}")
                    create_fuselage(l, w, h, filename)
                    records.append([filename, l, w, h])
                    idx += 1
    else:
        # Random sampling
        for i in range(num_samples):
            length = random.uniform(*length_range)
            width = random.uniform(*width_range)
            height = random.uniform(*height_range)
            filename = f"{output_prefix}_{i+1}.vsp3"
            print(f"\nGenerating fuselage {i+1}: Length={length:.2f}, Width={width:.2f}, Height={height:.2f}")
            create_fuselage(length, width, height, filename)
            records.append([filename, length, width, height])

    # Write CSV log
    with open(log_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["filename", "length", "width", "height"])
        writer.writerows(records)
    print(f"\nBatch log written to {log_csv}")


if __name__ == "__main__":
    # Example usage:
    # python create_fuselage.py <length> <width> <height> [output_file]
    # python create_fuselage.py --batch
    # python create_fuselage.py --batch --steps <length_step> <width_step> <height_step>

    if len(sys.argv) >= 2 and sys.argv[1] == "--batch":
        length_range = (10.0, 15.0)
        width_range = (2.0, 4.0)
        height_range = (2.0, 4.0)
        num_samples = 10
        step_size = None

        if "--steps" in sys.argv:
            idx = sys.argv.index("--steps")
            step_size = (float(sys.argv[idx + 1]), float(sys.argv[idx + 2]), float(sys.argv[idx + 3]))
            batch_generate_fuselages(
                length_range, width_range, height_range,
                output_prefix="fuselage",
                log_csv="fuselage_batch_log.csv",
                step_size=step_size
            )
        else:
            batch_generate_fuselages(
                length_range, width_range, height_range,
                num_samples=num_samples,
                output_prefix="fuselage",
                log_csv="fuselage_batch_log.csv"
            )

    elif len(sys.argv) < 4:
        print("Usage: python create_fuselage.py <length> <width> <height> [output_file]")
        print("   or: python create_fuselage.py --batch")
        print("   or: python create_fuselage.py --batch --steps <length_step> <width_step> <height_step>")
        sys.exit(1)
    else:
        length = float(sys.argv[1])
        width = float(sys.argv[2])
        height = float(sys.argv[3])
        filename = sys.argv[4] if len(sys.argv) > 4 else "fuselage.vsp3"
        create_fuselage(length, width, height, filename)