"""
Export GMR robot motion pickle to OpenSim .mot (Gait 2392 column layout).

Uses the same header/column pattern as assets/opensim_samples/normal.mot.
With --mapping, fills coordinates from root pose + dof_pos via a JSON config.
"""

import argparse
import pickle

import numpy as np

from general_motion_retargeting.opensim import (
    BUNDLED_UNITREE_G1_GAIT2392_MAPPING,
    BUNDLED_UNITREE_G1_GAIT2392_WALK_MAPPING,
    GAIT2392_COORDINATE_COLUMNS,
    build_gait2392_coordinate_columns,
    load_mapping,
    write_mot_file,
)


def _diagnose_gmr_motion(motion: dict) -> None:
    """Print hints when pickle data cannot drive visible horizontal motion or one leg."""
    rp = np.asarray(motion.get("root_pos", []))
    if rp.ndim == 2 and rp.shape[0] > 1 and rp.shape[1] >= 2:
        spread_xy = float(np.max(np.ptp(rp[:, :2], axis=0)))
        if spread_xy < 1e-4:
            print(
                "GMR→OpenSim: root_pos X/Y range is ~0 m — pelvis_tx/tz from world_plane will stay flat; "
                "GMR often keeps the robot base fixed in XY. Use human trans in the pipeline or a clip with root motion."
            )
    dof = np.asarray(motion.get("dof_pos", []))
    if dof.ndim == 2 and dof.shape[0] > 1 and dof.shape[1] > 11:
        std_r = float(np.std(dof[:, 6:12], axis=0).max())
        std_l = float(np.std(dof[:, 0:6], axis=0).max())
        if std_r < 1e-9 and std_l > 1e-6:
            print(
                "GMR→OpenSim: right-leg dof (indices 6–11) are nearly constant while the left leg varies — "
                "check retargeting / pickle dof order, not only the .mot exporter."
            )
        elif std_l < 1e-9 and std_r > 1e-6:
            print(
                "GMR→OpenSim: left-leg dof (indices 0–5) are nearly constant while the right leg varies — "
                "check retargeting / pickle dof order."
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="GMR .pkl → OpenSim .mot (Gait 2392 layout)")
    parser.add_argument("--pkl", type=str, required=True, help="Path to GMR motion pickle")
    parser.add_argument("--out", type=str, required=True, help="Output .mot path")
    parser.add_argument(
        "--title",
        type=str,
        default="gmr_gait2392",
        help="First-line label in the .mot file (see normal.mot: normal_gait)",
    )
    parser.add_argument(
        "--mapping",
        type=str,
        default=None,
        help="JSON mapping (GMR → Gait2392 coords). Omit for placeholder (time only, rest 0).",
    )
    parser.add_argument(
        "--bundled-mapping-g1",
        action="store_true",
        help=f"Use bundled Unitree G1 mapping: {BUNDLED_UNITREE_G1_GAIT2392_MAPPING}",
    )
    parser.add_argument(
        "--bundled-mapping-g1-walk",
        action="store_true",
        help=f"Use bundled Unitree G1 walk mapping: {BUNDLED_UNITREE_G1_GAIT2392_WALK_MAPPING}",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress diagnostic messages about root_pos / dof ranges",
    )
    args = parser.parse_args()

    with open(args.pkl, "rb") as f:
        motion = pickle.load(f)

    if not args.quiet:
        _diagnose_gmr_motion(motion)

    fps = float(motion["fps"])
    n = motion["dof_pos"].shape[0]
    time_col = np.arange(n, dtype=np.float64) / fps

    n_coord = len(GAIT2392_COORDINATE_COLUMNS)
    data = np.zeros((n, n_coord), dtype=np.float64)
    data[:, 0] = time_col

    mapping_path = args.mapping
    if args.bundled_mapping_g1_walk:
        mapping_path = str(BUNDLED_UNITREE_G1_GAIT2392_WALK_MAPPING)
    elif args.bundled_mapping_g1:
        mapping_path = str(BUNDLED_UNITREE_G1_GAIT2392_MAPPING)
    if mapping_path is not None:
        mapping = load_mapping(mapping_path)
        data[:, 1:] = build_gait2392_coordinate_columns(motion, mapping)
        mode = f"mapped ({mapping_path})"
    else:
        mode = "placeholder (time only; other coords 0)"

    write_mot_file(
        args.out,
        data,
        GAIT2392_COORDINATE_COLUMNS,
        title=args.title,
        in_degrees=True,
    )
    print(f"Wrote {n} rows × {n_coord} columns to {args.out} ({mode})")


if __name__ == "__main__":
    main()
