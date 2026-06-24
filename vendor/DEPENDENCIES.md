# External dependencies (forks) — not committed; reproduce from here

Two upstream projects are cloned into the workspace at runtime but are **not** part of
this git repo (they are large and are their own git repositories):

- `crazyflie-firmware/` — the cf2 SITL firmware + Gazebo bridge (CrazySim)
- `cflib-src/` — Bitcraze `cflib`, installed editable so crazyswarm2 can talk over UDP

They are `.gitignore`d (same as `src/crazyswarm2/` and `src/crazyflie-simulation/`). Our
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

## Pinned versions

| Fork | Upstream | Branch | Commit |
|---|---|---|---|
| crazyflie-firmware | `github.com/llanesc/crazyflie-firmware` | `crazysim` | `aa6571dc` |
| cflib-src | `github.com/bitcraze/crazyflie-lib-python` | `master` | `c8bf364` |
