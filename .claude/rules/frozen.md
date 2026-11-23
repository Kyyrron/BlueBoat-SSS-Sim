---
paths:
  - "blueboat_sss_sim/blueboat_sss_sim/**"
  - "blueboat_sss_sim/config/**"
  - "blueboat_sss_sim/launch/**"
  - "blueboat_sss_sim/setup.py"
  - "blueboat_sss_sim/docs/**"
---
# The simulator is frozen (2026-11-18), with one world-generation version since (v1.1, 2026-11-21)

No `*.py` of the package and no `config/*.yaml` may change: code digest `cc29e08780aac4cd` (v1.0) or
`bbd4112928645e13` (v1.1: the World Builder's Mission-like mode, `builder/` only; pending the owner's signature)
(`python3 -m blueboat_sss_sim.core.provenance --check`). A change is a new simulator version, by owner decision only.
The engine digest `ffacf236faa57010` (`sonar/`, `worldgen/`, `core/`) is the same in both. The population model lives
outside the package (`../blueboat_harbour_population/`, its own version and digests) and is not frozen with it.
`blueboat_sss_sim/docs/` is installed by `setup.py`: do not add, remove or edit files there. Simulated results follow
the superproject's `SIM_VALIDITY_CONDITIONS.md`. Module guidance: `.claude/CLAUDE.md` of this repository.
