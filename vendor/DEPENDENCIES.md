# External dependencies (forks) — not committed; reproduce from here

Three upstream projects are cloned into the workspace at runtime but are **not** part of
this git repo (they are large and are their own git repositories):

- `crazyflie-firmware/` — the cf2 SITL firmware + Gazebo bridge (CrazySim)
- `cflib-src/` — Bitcraze `cflib`, installed editable so crazyswarm2 can talk over UDP
- `src/crazyswarm2/` — the **llanesc fork** (UDP backend for CrazySim). **`deps.repos`
  points at upstream `IMRCLab/crazyswarm2@main` as a base, but you must use the llanesc
  `crazysim` fork below** — IMRClab main has no UDP backend and won't reach the SITL firmware.

They are `.gitignore`d. Our
**local edits** to them are small and intentional, so they are preserved here as patches +
copied files. To rebuild the environment on a fresh machine, clone each at its pinned
commit and apply the patch(es) below.

> These patches were captured by `git -C <fork> diff`; the forks on disk are unchanged.

---

## 1. crazyflie-firmware

```bash
git clone https://github.com/llanesc/crazyflie-firmware.git
cd crazyflie-firmware
git checkout crazysim          # pinned commit: aa6571dc
git submodule update --init --recursive

# (a) firmware edits: enable BVC collision avoidance in the SITL build
git apply /path/to/vendor/crazyflie-firmware/sitl-bvc.patch

# (b) new docker files (copy in; .dockerignore is the renamed copy)
cp /path/to/vendor/crazyflie-firmware/Dockerfile.cf2-sitl ./Dockerfile.cf2-sitl
cp /path/to/vendor/crazyflie-firmware/dockerignore.txt    ./.dockerignore

# (c) sim-launch scripts live in the tools/crazyflie-simulation submodule
cd tools/crazyflie-simulation
git apply /path/to/vendor/crazyflie-firmware/sitl-docker-launch.patch
cd ../..
```

Then build the SITL container per `../CRAZYSIM_MIGRATION.md` ("One-time setup").

## 2. cflib-src

```bash
git clone https://github.com/bitcraze/crazyflie-lib-python.git cflib-src
cd cflib-src
git checkout master            # pinned commit: c8bf364
git apply /path/to/vendor/cflib/extpos-packed.patch
# then editable install per CRAZYSIM_MIGRATION.md "One-time setup" step 5
```

## 3. crazyswarm2 (use this fork, NOT the one in deps.repos)

```bash
# Replaces the IMRCLab/crazyswarm2 entry in deps.repos.
rm -rf src/crazyswarm2
git clone https://github.com/llanesc/crazyswarm2.git src/crazyswarm2
cd src/crazyswarm2
git checkout crazysim          # pinned commit: 94df3be
git submodule update --init --recursive
# local edit: server-side BVC peer-position broadcast (the orchestrator of BVC)
git apply /path/to/vendor/crazyswarm2/bvc-peer-broadcast.patch
cd ../..
```

---

## What each edit does (and is it sim, hardware, or both?)

Background: **BVC (Buffered Voronoi Cell) collision avoidance is firmware that runs on the
drone.** On **real hardware it was already enabled** in the stock firmware. The upstream
**sim build had it disabled** (there was a literal `// TODO: Add collision avoidance to
SITL`). These edits bring the simulator up to parity so the safety net can be tested in
Gazebo before flying. See `../CRAZYSIM_MIGRATION.md §1.7`.

| Patch / file | Sim or HW | What & why |
|---|---|---|
| `sitl-docker-launch.patch` + `Dockerfile.cf2-sitl` + `.dockerignore` | sim infra | run the cf2 firmware inside an Ubuntu-22.04 docker container (the CrazySim migration; cf2 won't run on a 24.04 host) |
| `sitl-bvc.patch` → `sitl_make/CMakeLists.txt` | **sim only** | add `collision_avoidance.c` to the SITL build sources (the hardware build already compiled it) |
| `sitl-bvc.patch` → `src/modules/src/stabilizer.c` | **sim only** | remove the `#ifndef CONFIG_PLATFORM_SITL` guard so `collisionAvoidanceUpdateSetpoint()` runs in sim too (it was hardware-only) |
| `extpos-packed.patch` → `cflib/crazyflie/localization.py` | **both** | adds `send_extpos_packed()` + the `EXT_POSITION_PACKED` channel: packs each drone's `(id, x, y, z)` into one CRTP packet so every drone learns its **neighbors'** positions — the data BVC consumes. Used in sim and on hardware alike (transport-agnostic: travels over sim UDP or real radio) |
| `bvc-peer-broadcast.patch` → `crazyswarm2 crazyflie_server.py` | **both** | the orchestrator: the server tracks each drone's latest pose and, at `peer_broadcast_hz` (set under `all:` in the crazyflies yaml), calls `send_extpos_packed()` to give each drone ONLY its neighbors (distinct ids 1..N, never the firmware `my_id`). Without it the firmware sees `nOthers=0` and BVC does nothing |

## Runtime Python dependency — transforms3d ≥ 0.4.2 (NumPy 2.0 compat)

Not a fork, but a host pip constraint the server depends on. `crazyflie_server.py` imports
`tf_transformations`, which imports `transforms3d`.

**Symptom:** the cflib server node dies immediately on launch with
`AttributeError: np.maximum_sctype was removed in the NumPy 2.0 release`
(traceback through `transforms3d/quaternions.py` → `tf_transformations` →
`crazyflie_server.py`); the server never reaches `[cf1] is connected!`.

**Cause:** the apt `python3-transforms3d` (0.4.1, in `/usr/lib/python3/dist-packages`) calls
`np.maximum_sctype`, which was removed in NumPy 2.0. This box runs NumPy 2.5.0 from
`~/.local`, so the old transforms3d breaks at import.

**Fix:** install transforms3d 0.4.2 (the first NumPy-2.0-compatible release) into the user
site so it shadows the apt 0.4.1:

```bash
pip install --user --break-system-packages transforms3d==0.4.2
# verify: python3 -c "import tf_transformations; print('ok')"
# revert:  pip uninstall --user transforms3d   (falls back to apt 0.4.1)
```

`--break-system-packages` is needed because Ubuntu marks the system Python
externally-managed (PEP 668); the install stays confined to `~/.local`, matching how
NumPy 2.5.0 already got there.

## Runtime Python dependency — rowan (only for `gui:=True`)

**Symptom:** with `gui:=True` on the server launch, the `gui.py` node dies with
`ModuleNotFoundError: No module named 'rowan'`.

**Cause:** crazyswarm2's 2D position viewer (`gui.py`) imports `rowan` (a quaternion lib)
that isn't installed here. **Non-blocking:** only the `gui.py` node dies — the
`crazyflie_server`, teleop, and the flight itself are unaffected (the launch keeps the
other nodes running). The 3D flight view comes from the Gazebo GUI, not `gui.py`.

**Fix (only if you actually want the 2D viewer):**

```bash
pip install --user --break-system-packages rowan
```

Same `--break-system-packages` / `~/.local` rationale as transforms3d above. Skip it
entirely by launching with `gui:=false`.

## Pinned versions

| Fork | Upstream | Branch | Commit |
|---|---|---|---|
| crazyflie-firmware | `github.com/llanesc/crazyflie-firmware` | `crazysim` | `aa6571dc` |
| cflib-src | `github.com/bitcraze/crazyflie-lib-python` | `master` | `c8bf364` |
| src/crazyswarm2 | `github.com/llanesc/crazyswarm2` | `crazysim` | `94df3be` |
