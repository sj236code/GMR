"""
OpenSim .mot (kinematics) writer.

Header and column order for Gait 2392 match assets/opensim_samples/normal.mot.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np

# Tab-separated labels. Extends reference normal.mot (18 cols) with pelvis_tx and
# pelvis_tz so horizontal pelvis motion (e.g. side-step) is not stuck at model defaults.
GAIT2392_COORDINATE_COLUMNS: tuple[str, ...] = (
    "time",
    "pelvis_list",
    "pelvis_rotation",
    "pelvis_tilt",
    "pelvis_tx",
    "pelvis_ty",
    "pelvis_tz",
    "hip_flexion_r",
    "hip_adduction_r",
    "hip_rotation_r",
    "knee_angle_r",
    "ankle_angle_r",
    "hip_flexion_l",
    "hip_adduction_l",
    "hip_rotation_l",
    "knee_angle_l",
    "ankle_angle_l",
    "lumbar_bending",
    "lumbar_rotation",
    "lumbar_extension",
)


def write_mot_file(
    path: str | Path,
    data: np.ndarray,
    column_names: Sequence[str],
    *,
    title: str = "gmr_export",
    in_degrees: bool = True,
) -> None:
    """
    Write an OpenSim .mot file.

    Parameters
    ----------
    path
        Output path.
    data
        Array of shape (n_rows, n_columns), row-major, matching column_names.
    column_names
        Length must equal data.shape[1]. Use GAIT2392_COORDINATE_COLUMNS for Gait 2392.
    title
        First line of the file (metadata label).
    in_degrees
        Sets inDegrees=yes/no in the header. Angle columns should match this convention.
    """
    path = Path(path)
    data = np.asarray(data, dtype=np.float64)
    if data.ndim != 2:
        raise ValueError(f"data must be 2D, got shape {data.shape}")
    names = list(column_names)
    if data.shape[1] != len(names):
        raise ValueError(
            f"data has {data.shape[1]} columns but column_names has {len(names)} entries"
        )
    n_rows, n_columns = data.shape
    in_deg = "yes" if in_degrees else "no"
    lines = [
        title,
        "version=1",
        f"nRows={n_rows}",
        f"nColumns={n_columns}",
        f"inDegrees={in_deg}",
        "endheader",
        "\t".join(names),
    ]
    body = "\n".join(
        "\t".join(f"{v:.12g}" for v in row) for row in data
    )
    text = "\n".join(lines) + "\n" + body + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
