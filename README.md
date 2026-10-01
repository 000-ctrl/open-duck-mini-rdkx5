# Open Duck Mini RDK-X5 Runtime And Sim-To-Real Diagnostics

Private working repository for the Open Duck Mini RDK-X5 board runtime, robot-specific calibration evidence, and non-invasive sim-to-real diagnostic tooling.

This repository exists because the live board runtime at `/home/sunrise/project/Open_Duck_Mini_Runtime-2_RDK_X5` is an RDK-X5 fork and is not a git repository. The goal is to preserve the exact board-side runtime and make diagnostics reproducible before changing training, gains, offsets, IMU remaps, friction, phase timing, or policy behavior.

The repository and its documentation are part of the robot's working state. Keep them current when the board runtime, config, evidence, analysis results, or next diagnostic gate changes.

## Current Next Step

The first evidence and suspended replay phases have shifted the leading root
cause from "unknown deployed contract mismatch" to "dynamic actuator mismatch."
The current work is offline:

```text
manual CUDA/Colab candidate run
  -> import the CUDA artifact bundle locally
  -> review x=0.0 and x=0.08 candidate sim gates
  -> only if gates pass, package a candidate ONNX under a new name
  -> only then request suspended robot validation
```

Local CPU pilots validate the training/export/package plumbing but repeatedly
learn near-standing policies. Local `7900 XTX` ROCm can run tiny host-loop
closed-loop smoke probes, but it is too slow for full-horizon eval or training.
Use `tools/print_cuda_colab_cell.py --run-candidate` in a manually
authenticated CUDA notebook/session for the next candidate attempt.

Do not tune hardware gains, patch IMU remaps, edit offsets, change action
scale, change phase timing, overwrite `BEST_WALK_ONNX_2.onnx`, or run grounded
walking yet.

Primary docs:

- [Project goal](PROJECT_GOAL.md)
- [Roadmap](ROADMAP.md)
- [Safety rules](docs/SAFETY_RULES.md)
- [Evidence flow](docs/EVIDENCE_FLOW.md)
- [Issue backlog](docs/ISSUE_BACKLOG.md)
- [Roboticist playbook](docs/ROBOTICIST_PLAYBOOK.md)
- [Deploy instrumentation](docs/DEPLOY_INSTRUMENTATION.md)
- [Diagnostic runbook](docs/run_sim2real_diagnostics.md)
- [Training actuator wrapper workflow](docs/TRAINING_ACTUATOR_WRAPPER_WORKFLOW.md)
- [Candidate policy validation gates](docs/CANDIDATE_POLICY_VALIDATION_GATES.md)
- [CUDA backend training runbook](docs/CUDA_BACKEND_TRAINING_RUNBOOK.md)
- [CUDA / Colab single cell](docs/CUDA_COLAB_SINGLE_CELL.md)
- [Diagnostic thresholds](docs/DIAGNOSTIC_THRESHOLDS.md)
- [Power manager handoff](docs/POWER_MANAGER_HANDOFF.md)
- [Agent instructions](AGENTS.md)

## Current Board State

- Board: D-Robotics RDK-X5, hostname `ubuntu`
- Current SSH target: `sunrise@192.168.1.50`
- Runtime on board: `/home/sunrise/project/Open_Duck_Mini_Runtime-2_RDK_X5`
- Python env on board: `/home/sunrise/duck_env/bin/python`
- Policy: `/home/sunrise/BEST_WALK_ONNX_2.onnx`
- Policy SHA256: `3c606f9381a1710cc8fecdb7442787dcbfce3ee9bc02a6f1224774ab2b3a1067`
- `imu_upside_down`: `true`
- `start_paused`: `true`
- IMU calibration pickle: not found in the snapshot search paths

The exact live config snapshot is in `evidence/20260621T180046Z_rdkx5_config_snapshot.json`.

## Repository Layout

```text
runtime/                 Board RDK-X5 runtime copied from the robot
instrumentation/         Additive diagnostic files to deploy into runtime/
tools/                   Workstation analysis and snapshot utilities
docs/                    Audit, runbook, telemetry schema, joint map, test plan
evidence/                Captured board config and calibration evidence
policy/                  Matching BEST_WALK_ONNX_2 baseline policy
```

## Safety Rules

- Do not run walking tests until home pose, IMU tilt, foot contact, joint identity, and suspended replay checks pass.
- Do not run suspended replay unless `v2_rl_walk_mujoco.py` exposes opt-in telemetry support and the replay command writes JSONL telemetry.
- Do not tune gains, offsets, action scale, friction, phase timing, IMU remaps, or policy code during evidence collection.
- Any moving diagnostic must be run with the robot physically supported unless the test explicitly says grounded.
- Keep fingers clear and be ready to cut power.
- Moving diagnostics require an explicit safety flag in the command.

## Diagnostic Gate Order

Run this order. Stop immediately if a gate fails.

```text
1. snapshot_robot_config
2. home_pose_log_test
3. analyze home pose log
4. imu_tilt_test
5. analyze IMU tilt log
6. foot_contact_test
7. joint_identity_test
8. suspended_policy_replay, x = 0.0
9. suspended_policy_replay, x = 0.08 only if x = 0.0 looks sane
10. push test
11. grounded_policy_replay
12. actuator_sine_sweep
```

The first strict gate is upright acceleration:

```text
obs[3:6] should be roughly [small, small, +9.5]
```

Expected ONNX accel means:

```text
obs[3] accel_x ~= -0.037894
obs[4] accel_y ~= -0.079717
obs[5] accel_z ~=  9.490142
```

Suspended replay evidence must include both JSONL telemetry and terminal log
capture so servo CRC/read warnings and control-budget warnings are visible in
the gate summary before increasing command speed.

CRC/read retry warnings are not an automatic stop. The suspended replay gate
holds only when they correlate with control damage such as dt spikes, action
saturation or jumps, post-startup tracking spikes, write failures, visible
twitching, or a read-error burst. Otherwise the gate reports
`WARN_PROCEED_WITH_CAUTION` and the next low-risk suspended command can be
considered.

## Key Live Calibration Facts

The live `duck_config.json` offsets are recorded in `evidence/duck_config_20260621T180046Z.json` and `docs/joint_map_template.yaml`.

Notable offsets:

```text
left_knee  -1.4880 rad
right_knee  0.0798 rad
```

This asymmetry may be legitimate after mechanical repair, but it must be verified by home-pose telemetry and `joint_identity_test`. Do not change it without evidence.

## Board Runtime Differences

The board runtime is not identical to the local upstream reference:

- `raw_imu.py` uses an RDK-compatible `I2CCompat` layer.
- `feet_contacts.py` uses `Hobot.GPIO` on RDK-X5.
- `rustypot_position_hwi.py` retries servo bus operations.
- `joints_dir` is all `+1.0`; no software sign flip is intended.

See `docs/SIM2REAL_AUDIT.md` for the full map.

## Deployment

Read `docs/DEPLOY_INSTRUMENTATION.md` before copying instrumentation to the board. The instrumentation is designed to be additive and disabled by default, but board runtime files should still be backed up before any overwrite.

Check the runtime telemetry contract locally before deploying:

```bash
python3 tools/check_runtime_telemetry_contract.py
```

Dry-run first:

```bash
bash scripts/deploy_instrumentation_to_duck.sh --dry-run
```

Apply only after the plan is reviewed:

```bash
bash scripts/deploy_instrumentation_to_duck.sh --apply
```

## Analysis

Use the workstation tools for analysis. The board Python env has `onnxruntime` but not `onnx`, so ONNX normalization extraction should usually run off-board.

```bash
python3 tools/analyze_telemetry_obs.py path/to/home_pose_log_test.jsonl \
  --onnx-model policy/BEST_WALK_ONNX_2.onnx
```

## What Not To Do Yet

- Do not retrain.
- Do not tune offsets.
- Do not patch IMU axes or offsets.
- Do not change action scale.
- Do not change gains.
- Do not change phase timing.
- Do not debug friction/TPU/contact until sensor/action/joint truth tables pass.
