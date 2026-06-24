# cs2_ws — drone swarm research workspace

This is a multi-project research workspace. Infrastructure (CrazySim, crazyswarm2,
hardware bringup) is shared; individual research projects (federated coverage,
thermal mapping, payload coupling, RL, multinash bridge) sit on top.

## Doc structure (read in this order)

1. **`CRAZYSIM_MIGRATION.md`** — workspace + infrastructure. Architecture diagram,
   sim/HW invariants, hardware bringup checklist, conventions, migration history.
   **Start with §0 "How to use these docs"** for the doc protocol.
2. **`docs/architecture_map.md`** — code index: every module → real file path →
   `ros2 run` name → topics in/out. Use this to answer "where is the code for X".
3. **`docs/<project>.md`** — per-project. Goal, packages owned, internal contracts,
   status, next steps, how to run. Project index lives in `CRAZYSIM_MIGRATION.md §5`.

## When working in this workspace

| You're doing | Update |
|---|---|
| Starting a new project | Create `docs/<project>.md` (template in `CRAZYSIM_MIGRATION.md §0`). Add a row to project index in §5. List any new packages in the workspace tree in §1.6. |
| Finishing a project feature | Flip that feature's row in the project doc's "Status" table. Update its "Next steps". |
| Adding a new package to an existing project | Update that project doc's "Packages" + workspace tree in `CRAZYSIM_MIGRATION.md §1.6`. |
| Adding/changing infrastructure (sim/HW invariant, hardware step, convention) | Update `CRAZYSIM_MIGRATION.md` only. Don't push into project docs. |
| Tuning project parameters | Project doc only. |

**Rule:** never duplicate infrastructure docs in project docs — link to the relevant
section of `CRAZYSIM_MIGRATION.md` instead.

Full protocol + project-doc template lives in `CRAZYSIM_MIGRATION.md §0`.

## How Claude should behave in this workspace

You (Claude) are the keeper of the doc system. The user shouldn't have to remember to
say "we're starting a new project" or "update the doc". You proactively detect and
prompt.

**At the start of any chat in cs2_ws**, read `CRAZYSIM_MIGRATION.md §5` (project index)
so you know what projects exist and which files belong to each.

**When the user describes work**, classify it:

| User intent | Your action |
|---|---|
| Work mentions an existing project (e.g., "let's fix something in the coverage demo") | Open `docs/<project>.md`. Treat it as the working context. Update its Status/Next steps when work completes. |
| Work doesn't map to any existing project doc, and isn't pure infra | Ask: *"This looks like a new project — want me to create `docs/<name>.md` and add it to the project index?"* Wait for confirmation. |
| Work is pure infrastructure (sim/HW invariant, hardware step, convention) | Update `CRAZYSIM_MIGRATION.md` only — no project doc needed. |
| User adds a new package to `src/` (you ran `mkdir src/foo_pkg` or similar) | Prompt: *"Add foo_pkg to the workspace tree in CRAZYSIM_MIGRATION.md §1.6 and to docs/<project>.md packages list?"* |
| User finishes a feature for an existing project | After code is committed/tested, prompt: *"Update Status in docs/<project>.md?"* |
| User adds a new project that uses a package already owned by another project | Count cross-project references. If this is the 3rd project to use that package, prompt: *"`<pkg>` is now used by 3 projects — promote to root infrastructure in CRAZYSIM_MIGRATION.md?"* User decides yes/no. |

The user can always override or skip these prompts — they're nudges, not gates.

**Anti-pattern:** writing or modifying code without knowing which project you're in.
If unclear, ask first.
