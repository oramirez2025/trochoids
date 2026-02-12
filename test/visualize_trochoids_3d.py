import argparse
import os
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 - registers 3D projection


def parse_args():
    parser = argparse.ArgumentParser(description="Visualize 3D trochoid test CSV outputs.")
    parser.add_argument(
        "--csv-dir",
        type=str,
        default=os.environ.get("TROCHOIDS_3D_CSV_DIR", "csv_files/3d"),
        help="Directory containing CSV files with columns: x,y,z,psi",
    )
    parser.add_argument(
        "--fig-dir",
        type=str,
        default="figures/3d",
        help="Directory where figures are saved",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show figures interactively",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    csv_dir = Path(args.csv_dir)
    fig_dir = Path(args.fig_dir)
    fig_dir.mkdir(parents=True, exist_ok=True)

    if not csv_dir.exists():
        raise FileNotFoundError(f"CSV directory does not exist: {csv_dir}")

    csv_files = sorted([p for p in csv_dir.iterdir() if p.suffix == ".csv"])
    if not csv_files:
        raise FileNotFoundError(f"No CSV files found in: {csv_dir}")

    print(f"Found {len(csv_files)} CSV file(s) in {csv_dir}")

    for csv_file in csv_files:
        df = pd.read_csv(csv_file, header=None)
        if df.shape[1] < 3:
            print(f"Skipping {csv_file.name}: expected at least 3 columns (x,y,z)")
            continue

        x = df[0]
        y = df[1]
        z = df[2]
        name = csv_file.stem

        # XY projection
        fig_xy, ax_xy = plt.subplots(figsize=(7, 6))
        scatter_xy = ax_xy.scatter(x, y, c=z, s=8, cmap="viridis")
        ax_xy.plot(x, y, linewidth=1.0, alpha=0.6, color="black")
        ax_xy.set_aspect("equal", adjustable="box")
        ax_xy.set_title(f"{name} (XY)")
        ax_xy.set_xlabel("X")
        ax_xy.set_ylabel("Y")
        ax_xy.grid(True, alpha=0.3)
        fig_xy.colorbar(scatter_xy, ax=ax_xy, label="Altitude (z)")
        xy_path = fig_dir / f"{name}_xy.png"
        fig_xy.savefig(xy_path, dpi=180, bbox_inches="tight")
        print(f"Saved {xy_path}")

        # 3D view
        fig_3d = plt.figure(figsize=(8, 6))
        ax_3d = fig_3d.add_subplot(111, projection="3d")
        scatter_3d = ax_3d.scatter(x, y, z, c=z, s=8, cmap="viridis")
        ax_3d.plot(x, y, z, linewidth=1.0, alpha=0.6, color="black")
        ax_3d.set_title(f"{name} (3D)")
        ax_3d.set_xlabel("X")
        ax_3d.set_ylabel("Y")
        ax_3d.set_zlabel("Z")
        fig_3d.colorbar(scatter_3d, ax=ax_3d, label="Altitude (z)")
        fig_3d_path = fig_dir / f"{name}_3d.png"
        fig_3d.savefig(fig_3d_path, dpi=180, bbox_inches="tight")
        print(f"Saved {fig_3d_path}")

        if args.show:
            plt.show()
        else:
            plt.close(fig_xy)
            plt.close(fig_3d)


if __name__ == "__main__":
    main()
