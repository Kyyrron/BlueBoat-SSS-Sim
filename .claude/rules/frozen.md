---
paths:
  - "blueboat_sss_sim/blueboat_sss_sim/**"
  - "blueboat_sss_sim/config/**"
  - "blueboat_sss_sim/launch/**"
  - "blueboat_sss_sim/setup.py"
  - "blueboat_sss_sim/docs/**"
---
# The simulator is frozen (2026-11-18)

No `*.py` of the package and no `config/*.yaml` may change: code digest `cc29e08780aac4cd`
(`python3 -m blueboat_sss_sim.core.provenance --check`). A change is a new simulator version, by owner decision only.
`blueboat_sss_sim/docs/` is installed by `setup.py`: do not add, remove or edit files there. Simulated results follow
the superproject's `SIM_VALIDITY_CONDITIONS.md`. Module guidance: `.claude/CLAUDE.md` of this repository.
