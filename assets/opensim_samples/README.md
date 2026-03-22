# OpenSim reference samples

This folder holds a **target musculoskeletal model** and a **reference kinematics file** used to define the `.mot` format for the GMR → OpenSim export pipeline.

## Files

| File | Description |
|------|-------------|
| `gait2392_simbody.osim` | **Gait 2392** model (`3DGaitModel2392`): 23 DOF gait model (pelvis, legs, lumbar). See model credits inside the file and [OpenSim documentation](https://simtk-confluence.stanford.edu:8080/display/OpenSim/Gait+2392+and+2354+Models) for geometry dependencies (`.vtp` meshes, etc.). |
| `normal.mot` | Example walking motion: **canonical column order and header** for kinematics that load with this model. Exporter code should match this structure (`version=1`, `inDegrees=yes`, tab-separated columns). |

## Column template (`normal.mot`)

Reference `normal.mot` uses **18** tab-separated columns (pelvis translation is only **`pelvis_ty`**; horizontal `pelvis_tx` / `pelvis_tz` stay at model defaults).

GMR’s bundled export uses **20** columns: the same pelvis angles and leg/lumbar coordinates, but **`pelvis_tx`, `pelvis_ty`, `pelvis_tz` are grouped after `pelvis_tilt`** so root translation from MuJoCo can move the model in the horizontal plane (needed for motions like side-step).

## Code

- `general_motion_retargeting/opensim/mot_writer.py` — writes `.mot` files using the same header pattern; `nRows` / `nColumns` are derived from the data array.
- `general_motion_retargeting/opensim/gmr_to_gait2392.py` — builds non-time columns from a motion dict + JSON mapping (optional pelvis Euler; `root_pos` → `pelvis_tx/ty/tz`; legs/lumbar from `dof_pos`).
- `general_motion_retargeting/opensim/mappings/unitree_g1_gait2392.json` — bundled **starting-point** mapping for Unitree G1 (motor order in `g1_mocap_29dof.xml`); tune signs/offsets and pelvis Euler in OpenSim.
- `scripts/gmr_pkl_to_mot.py` — reads a GMR `.pkl` and writes a `.mot`. Use `--bundled-mapping-g1` (side-step preset), **`--bundled-mapping-g1-walk`** (walking / both legs + pelvis forward-back), or `--mapping <path.json>`; omit mapping flags for a time-only placeholder.
