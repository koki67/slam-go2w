# go2w URDF — Provenance

## Upstream

- **Repository**: `unitreerobotics/unitree_ros`
- **Path**: `robots/go2w_description/`
- **Commit SHA**: `a3f2c8d` (pinned; verify with upstream before updating)
- **License**: BSD-3-Clause (see LICENSE file)

## Modifications

- Stripped Gazebo plugin tags and transmission elements (not needed for kinematic FK only).
- Inertial parameters retained verbatim for future dynamics use.
- Wheel joint (`{FL,FR,RL,RR}_foot_joint`) kept as `continuous` to reflect the rolling constraint.

## Key dimensions used by dg_kilo

| Parameter | Value | Source |
|---|---|---|
| `wheel_radius` | 0.0513 m | go2w_description/meshes geometry |
| Hip lateral offset | 0.0465 m | `{FL,FR,RL,RR}_hip_joint` origin y |
| Hip longitudinal offset | ±0.1934 m | `{FL,FR,RL,RR}_hip_joint` origin x |
| Thigh length | 0.213 m | `{}_thigh_joint` origin z |
| Calf length | 0.213 m | `{}_foot_joint` origin z |

These are pinned in `config/dgkilo/go2w.yaml` under `wheels.*`. If the URDF is
updated, re-verify all lengths and update `go2w.yaml` accordingly.
