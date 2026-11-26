---
paths:
  - "blueboat_sss_sim/blueboat_sss_sim/**"
  - "blueboat_sss_sim/config/**"
  - "blueboat_sss_sim/launch/**"
  - "blueboat_sss_sim/setup.py"
  - "blueboat_sss_sim/docs/**"
---
# The simulator is frozen (2026-11-18), with one world-generation version since (v1.1)

No `*.py` of the package and no `config/*.yaml` may change: code digest `cc29e08780aac4cd` (v1.0) or
`ed13b523d1be4f07` (v1.1, pending the owner's signature) (`python3 -m blueboat_sss_sim.core.provenance --check`).
A change is a new simulator version, by owner decision only. v1.1 changes world generation only: the World Builder's
Mission-like mode (`builder/`) and `BuilderState.raster_resolution_m`, a finer seabed raster for one large world
(`builder/state.py`, `worldgen/world_folder.py`). Engine digest (`sonar/`, `worldgen/`, `core/`) `ffacf236faa57010`
(v1.0), `40764206ab14c647` (v1.1: only `worldgen/world_folder.py` differs in the engine set); default behaviour
bit-identical. The population model lives
outside the package (`../blueboat_harbour_population/`, its own version and digests) and is not frozen with it.
`blueboat_sss_sim/docs/` is installed by `setup.py`: do not add, remove or edit files there. Simulated results follow
the superproject's `SIM_VALIDITY_CONDITIONS.md`. Module guidance: `.claude/CLAUDE.md` of this repository.
