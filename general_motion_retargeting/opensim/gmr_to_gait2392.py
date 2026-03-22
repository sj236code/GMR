"""
Map GMR robot motion dict (pickle) to Gait 2392 .mot coordinate columns (no time).

Pelvis orientation uses scipy Euler angles from root quaternion (xyzw). Legs and
lumbar use dof_pos indices in Unitree G1 motor order. Signs/offsets are tuned in JSON.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from .mot_writer import GAIT2392_COORDINATE_COLUMNS

BUNDLED_UNITREE_G1_GAIT2392_MAPPING = (
    Path(__file__).resolve().parent / "mappings" / "unitree_g1_gait2392.json"
)
BUNDLED_UNITREE_G1_GAIT2392_WALK_MAPPING = (
    Path(__file__).resolve().parent / "mappings" / "unitree_g1_gait2392_walk.json"
)

COORD_NON_TIME = GAIT2392_COORDINATE_COLUMNS[1:]

RAD2DEG = 180.0 / np.pi


def load_mapping(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _fill_pelvis_euler(
    root_rot_xyzw: np.ndarray,
    cfg: dict[str, Any],
    arrays: dict[str, np.ndarray],
    n: int,
) -> None:
    seq = str(cfg.get("euler_seq", "zyx"))
    assignments = cfg["assignments"]
    probe = Rotation.from_quat(root_rot_xyzw[0]).as_euler(seq, degrees=True)
    euler_len = int(probe.shape[0])
    for spec in assignments.values():
        idx = int(spec["euler_index"])
        if idx < 0 or idx >= euler_len:
            raise ValueError(
                f"pelvis_orientation: euler_index {idx} invalid for seq {seq!r} (len {euler_len})"
            )
    for i in range(n):
        rot = Rotation.from_quat(root_rot_xyzw[i])
        euler = rot.as_euler(seq, degrees=True)
        for coord_name, spec in assignments.items():
            idx = int(spec["euler_index"])
            sign = float(spec.get("sign", 1.0))
            off = float(spec.get("offset_deg", 0.0))
            arrays[coord_name][i] = sign * float(euler[idx]) + off


def _fill_pelvis_horizontal_body_frame(
    root_pos: np.ndarray,
    root_rot_xyzw: np.ndarray,
    cfg: dict[str, Any],
    arrays: dict[str, np.ndarray],
    n: int,
) -> None:
    """
    Map horizontal root displacement into pelvis_tx / pelvis_tz using the pelvis facing at
    frame 0. Lateral = perpendicular to forward; sagittal = along forward. Setting
    sagittal_scale=0 drops forward/back drift that looks like the whole model sliding.
    """
    ax0 = int(cfg.get("horizontal_plane_axis0", 0))
    ax1 = int(cfg.get("horizontal_plane_axis1", 1))
    col = int(cfg.get("forward_body_matrix_column", 0))
    if col not in (0, 1, 2):
        raise ValueError("forward_body_matrix_column must be 0, 1, or 2")
    lat_to = str(cfg.get("lateral_to", "pelvis_tx"))
    sag_to = str(cfg.get("sagittal_to", "pelvis_tz"))
    lat_scale = float(cfg.get("lateral_scale", 1.0))
    sag_scale = float(cfg.get("sagittal_scale", 0.0))
    lat_off = float(cfg.get("lateral_offset_m", 0.0))
    sag_off = float(cfg.get("sagittal_offset_m", 0.0))

    R0 = Rotation.from_quat(root_rot_xyzw[0]).as_matrix()
    fwd3 = R0[:, col].astype(np.float64)
    fwd = np.array([fwd3[ax0], fwd3[ax1]], dtype=np.float64)
    fn = float(np.linalg.norm(fwd))
    if fn < 1e-8:
        fwd = np.array([1.0, 0.0])
    else:
        fwd /= fn
    right = np.array([-fwd[1], fwd[0]], dtype=np.float64)

    p0 = root_pos[0]
    d0 = np.stack([root_pos[:, ax0] - p0[ax0], root_pos[:, ax1] - p0[ax1]], axis=1)
    lateral = (d0 * right).sum(axis=1)
    sagittal = (d0 * fwd).sum(axis=1)

    arrays[lat_to][:] = lateral * lat_scale + lat_off
    arrays[sag_to][:] = sagittal * sag_scale + sag_off


def _fill_pelvis_world_plane(
    root_pos: np.ndarray, cfg: dict[str, Any], arrays: dict[str, np.ndarray]
) -> None:
    """Map MuJoCo root position axes directly to pelvis_tx / pelvis_tz (optionally relative to frame 0)."""
    iax = int(cfg.get("tx_source_axis", 0))
    zax = int(cfg.get("tz_source_axis", 1))
    if iax not in (0, 1, 2) or zax not in (0, 1, 2):
        raise ValueError("tx_source_axis and tz_source_axis must be 0, 1, or 2")
    tx_s = float(cfg.get("tx_scale", 1.0))
    tz_s = float(cfg.get("tz_scale", 1.0))
    tx_o = float(cfg.get("tx_offset_m", 0.0))
    tz_o = float(cfg.get("tz_offset_m", 0.0))
    sub = bool(cfg.get("subtract_initial", True))
    p0 = root_pos[0]
    bx = float(p0[iax]) if sub else 0.0
    bz = float(p0[zax]) if sub else 0.0
    arrays["pelvis_tx"][:] = (root_pos[:, iax] - bx) * tx_s + tx_o
    arrays["pelvis_tz"][:] = (root_pos[:, zax] - bz) * tz_s + tz_o


def _fill_root_pos_coord(
    root_pos: np.ndarray,
    coord_name: str,
    cfg: dict[str, Any],
    arrays: dict[str, np.ndarray],
) -> None:
    axis = int(cfg.get("axis", 0))
    if axis < 0 or axis > 2:
        raise ValueError(f"{coord_name}.axis must be 0, 1, or 2, got {axis}")
    scale = float(cfg.get("scale", 1.0))
    off = float(cfg.get("offset_m", 0.0))
    arrays[coord_name][:] = root_pos[:, axis] * scale + off


def _fill_dof(
    dof_pos: np.ndarray,
    dof_map: dict[str, Any],
    arrays: dict[str, np.ndarray],
    n_dof: int,
) -> None:
    for coord_name, spec in dof_map.items():
        if coord_name not in arrays:
            raise ValueError(f"dof_rad_to_deg: unknown coordinate {coord_name!r}")
        idx = spec.get("index")
        if idx is None:
            continue
        idx = int(idx)
        if idx < 0 or idx >= n_dof:
            raise ValueError(
                f"Coordinate {coord_name}: dof index {idx} out of range [0, {n_dof})"
            )
        sign = float(spec.get("sign", 1.0))
        off = float(spec.get("offset_deg", 0.0))
        scale = float(spec.get("rad_to_deg", RAD2DEG))
        arrays[coord_name][:] = sign * dof_pos[:, idx] * scale + off


def _apply_step_side_asymmetry(arrays: dict[str, np.ndarray], mapping: dict[str, Any]) -> None:
    """
    Pin the stance (opposite) leg's .mot columns to their frame-0 pose so only the
    stepping leg tracks retargeted motion — useful for side-step previews in OpenSim.
    """
    la = mapping.get("leg_asymmetry")
    if not isinstance(la, dict):
        return
    side = str(la.get("step_side", "")).lower()
    if side == "left":
        opp_coords = (
            "hip_flexion_r",
            "hip_adduction_r",
            "hip_rotation_r",
            "knee_angle_r",
            "ankle_angle_r",
        )
    elif side == "right":
        opp_coords = (
            "hip_flexion_l",
            "hip_adduction_l",
            "hip_rotation_l",
            "knee_angle_l",
            "ankle_angle_l",
        )
    else:
        return

    freeze = float(la.get("freeze_opposite_leg", 0.0))
    if freeze <= 0.0:
        return
    if freeze > 1.0:
        freeze = 1.0

    for name in opp_coords:
        if name not in arrays:
            continue
        col = arrays[name]
        base = float(col[0])
        col[:] = base + (1.0 - freeze) * (col - base)


def build_gait2392_coordinate_columns(
    motion: dict[str, Any], mapping: dict[str, Any]
) -> np.ndarray:
    """
    Build (n_frames, n_coords) array: columns matching GAIT2392_COORDINATE_COLUMNS[1:].

    Expects motion keys: dof_pos (n, d), root_pos (n, 3), root_rot (n, 4) xyzw.
    """
    dof_pos = np.asarray(motion["dof_pos"], dtype=np.float64)
    root_pos = np.asarray(motion["root_pos"], dtype=np.float64)
    root_rot = np.asarray(motion["root_rot"], dtype=np.float64)
    n = int(dof_pos.shape[0])
    if root_pos.shape != (n, 3):
        raise ValueError(f"root_pos must be (n, 3), got {root_pos.shape}")
    if root_rot.shape != (n, 4):
        raise ValueError(f"root_rot must be (n, 4) xyzw, got {root_rot.shape}")

    n_dof = dof_pos.shape[1]
    arrays: dict[str, np.ndarray] = {
        name: np.zeros(n, dtype=np.float64) for name in COORD_NON_TIME
    }

    # Optional: subtract frame-0 from selected root_pos axes (default [0,1]) so horizontal
    # motion is relative (avoids driving the pelvis with huge world coords / wrong drift).
    # Vertical axis (usually 2) stays absolute so pelvis_ty remains a plausible height.
    r_root = root_pos.astype(np.float64).copy()
    sub_axes = mapping.get("subtract_initial_root_axes", [0, 1])
    if isinstance(sub_axes, list) and sub_axes:
        for a in sub_axes:
            ai = int(a)
            if 0 <= ai <= 2:
                r_root[:, ai] -= root_pos[0, ai]

    pelvis_o = mapping.get("pelvis_orientation")
    if isinstance(pelvis_o, dict) and pelvis_o:
        _fill_pelvis_euler(root_rot, pelvis_o, arrays, n)

    ph = mapping.get("pelvis_horizontal")
    if isinstance(ph, dict):
        mode = str(ph.get("mode", "")).lower()
        if mode == "body_frame_lateral":
            _fill_pelvis_horizontal_body_frame(root_pos, root_rot, ph, arrays, n)
        elif mode == "world_plane":
            _fill_pelvis_world_plane(root_pos, ph, arrays)
        else:
            for pname in ("pelvis_tx", "pelvis_tz"):
                pcfg = mapping.get(pname)
                if isinstance(pcfg, dict) and pcfg:
                    src = r_root
                    if pcfg.get("use_absolute_root_pos", False):
                        src = root_pos
                    _fill_root_pos_coord(src, pname, pcfg, arrays)
    else:
        for pname in ("pelvis_tx", "pelvis_tz"):
            pcfg = mapping.get(pname)
            if isinstance(pcfg, dict) and pcfg:
                src = r_root
                if pcfg.get("use_absolute_root_pos", False):
                    src = root_pos
                _fill_root_pos_coord(src, pname, pcfg, arrays)

    pcfg_ty = mapping.get("pelvis_ty")
    if isinstance(pcfg_ty, dict) and pcfg_ty:
        src = root_pos if pcfg_ty.get("use_absolute_root_pos", False) else r_root
        _fill_root_pos_coord(src, "pelvis_ty", pcfg_ty, arrays)

    dof_map = mapping.get("dof_rad_to_deg")
    if isinstance(dof_map, dict) and dof_map:
        _fill_dof(dof_pos, dof_map, arrays, n_dof)

    _apply_step_side_asymmetry(arrays, mapping)

    return np.column_stack([arrays[name] for name in COORD_NON_TIME])
