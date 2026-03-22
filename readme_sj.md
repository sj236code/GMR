# GMR fork notes (OpenSim export)

Personal documentation for this fork: what upstream GMR does, what was added here, how it fits together, and how to run tests.

## 1. What GMR does

**General Motion Retargeting (GMR)** maps human motion to a humanoid robot using inverse kinematics (MuJoCo + mink). Typical inputs:

- **SMPL-X / AMASS** (`*_stageii.npz`): body pose, root orientation, translation, etc.
- Other pipelines (BVH, FBX, etc.) exist in the repo.

Outputs are **robot trajectories**: base position, base orientation (quaternion), and joint angles over time. The main entry script used here is `scripts/smplx_to_robot.py`, which can save a **pickle** (`.pkl`) with keys such as `root_pos`, `root_rot`, `dof_pos`, and `fps`.

See the upstream project README and `CLAUDE.md` in this repo for architecture and supported robots.

## 2. What was added in this fork

- **OpenSim kinematics export**: convert a GMR robot motion **pickle** into a **Gait 2392–style `.mot`** file for visualization or analysis in OpenSim.
- **`general_motion_retargeting/opensim/`**
  - `mot_writer.py` — `.mot` header and column layout (**20 columns**: includes `pelvis_tx`, `pelvis_ty`, `pelvis_tz` for horizontal pelvis motion, not only vertical).
  - `gmr_to_gait2392.py` — maps `root_pos` / `root_rot` / `dof_pos` into those columns using a JSON mapping (body-frame pelvis translation, optional `world_plane`, leg DOF signs, etc.).
  - `mappings/unitree_g1_gait2392.json` — bundled **side-step** preset (`--bundled-mapping-g1`).
  - `mappings/unitree_g1_gait2392_walk.json` — bundled **walk / turn / general locomotion** preset (`--bundled-mapping-g1-walk`).
- **`scripts/gmr_pkl_to_mot.py`** — CLI: read `.pkl`, write `.mot`; optional diagnostics unless `--quiet`.
- **`setup.py`** — `package_data` includes `opensim/mappings/*.json` so editable installs still find bundled mappings.
- **`assets/opensim_samples/`** — reference **Gait 2392** model (`gait2392_simbody.osim`), example **`normal.mot`**, and a short README describing the column template.
- **`general_motion_retargeting/utils/smpl.py`** — fixes/normalization for AMASS-style NPZ loading (e.g. stage I `poses`, framerate field names, betas shape) so `smplx_to_robot.py` works on common AMASS exports.

## 3. How it was added

1. **Column contract** — Match OpenSim Gait 2392 expectations: tab-separated, `inDegrees=yes`, coordinate names aligned with the model. The exporter builds the same column order as defined in `mot_writer.GAIT2392_COORDINATE_COLUMNS`.
2. **Mapping layer** — JSON files describe how Unitree G1 `dof_pos` indices and root pose map to OpenSim coordinate names (hip/knee/ankle/lumbar, pelvis translations). Tuning (signs, `sagittal_scale`, `lateral_scale`, optional `leg_asymmetry` for side-step previews) lives in JSON, not only in code.
3. **Pelvis motion** — Horizontal pelvis motion uses **`body_frame_lateral`** in the walk mapping (displacement projected onto forward vs lateral using the root orientation at frame 0), so progression matches the robot heading instead of raw world X/Y when that matters.
4. **Pipeline** — `smplx_to_robot.py` produces `.pkl` → `gmr_pkl_to_mot.py` produces `.mot` → load **`gait2392_simbody.osim`** in OpenSim and apply the `.mot` as kinematics (same idea as the bundled `normal.mot`).

## 4. How to test

### Prerequisites

- Conda/venv with this package installed (`pip install -e .`), SMPL-X body models under `assets/body_models/smplx/` (see main README; that directory is gitignored).
- AMASS **`*_stageii.npz`** files (not `male_stagei.npz`-style stage I-only exports without the fields GMR expects).
- **OpenSim** (separate install) to preview `.mot` on `assets/opensim_samples/gait2392_simbody.osim`.

### Shell variable (WSL / Bash)

Define `ACCAD` in the **same** shell session as the Python commands. If `ACCAD` is empty, paths like `"$ACCAD/file.npz"` resolve to invalid paths under `/`.

### Side-step clip + side-step mapping

```bash
ACCAD="/mnt/c/Users/admin/Downloads/ACCAD (1)/ACCAD/Male1Walking_c3d"
cd /mnt/c/Users/admin/github/GMR_Fork/GMR

python scripts/smplx_to_robot.py \
  --smplx_file "$ACCAD/Walk_B22_-_Side_step_left_stageii.npz" \
  --robot unitree_g1 \
  --save_path "$ACCAD/gmr_side_step_left.pkl" \
  --headless

python scripts/gmr_pkl_to_mot.py \
  --pkl "$ACCAD/gmr_side_step_left.pkl" \
  --out "$ACCAD/gmr_side_step_left.mot" \
  --bundled-mapping-g1
```

### Walk / backward walk + walk mapping

Use **`--bundled-mapping-g1-walk`** for straight walking, turns, stand-to-walk, etc. Example:

```bash
ACCAD="/mnt/c/Users/admin/Downloads/ACCAD (1)/ACCAD/Male1Walking_c3d"
cd /mnt/c/Users/admin/github/GMR_Fork/GMR

python scripts/smplx_to_robot.py \
  --smplx_file "$ACCAD/Walk_B4_-_Stand_to_Walk_Back_stageii.npz" \
  --robot unitree_g1 \
  --save_path "$ACCAD/gmr_stand_to_walk_back.pkl" \
  --headless

python scripts/gmr_pkl_to_mot.py \
  --pkl "$ACCAD/gmr_stand_to_walk_back.pkl" \
  --out "$ACCAD/gmr_stand_to_walk_back.mot" \
  --bundled-mapping-g1-walk
```

### Optional: preview the pickle in MuJoCo (no OpenSim)

```bash
python scripts/vis_robot_motion.py \
  --robot unitree_g1 \
  --robot_motion_path "$ACCAD/gmr_side_step_left.pkl"
```

### Mapping cheat sheet

| Motion style | `gmr_pkl_to_mot.py` flag |
|--------------|---------------------------|
| Side-step / stance-leg frozen preset | `--bundled-mapping-g1` |
| Walking, turns, transitions, both legs | `--bundled-mapping-g1-walk` |

Custom JSON: `--mapping /path/to/your_mapping.json` (copy and edit files under `general_motion_retargeting/opensim/mappings/`).

---

*This file is maintainer-specific documentation (`readme_sj.md`); the project’s main entry point for users remains the root `README.md`.*
