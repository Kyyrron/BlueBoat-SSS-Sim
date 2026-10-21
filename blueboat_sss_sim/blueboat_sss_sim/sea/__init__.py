"""Sea state for the simulation: ocean current + wind waves, ROS-free.

The model that makes the simulated BlueBoat *feel* the water. It computes,
per tick, the two quantities Gazebo cannot produce on its own:

* a world-frame **water velocity** (a mean current with smooth, bounded,
  seeded variation in time and space + wave orbital velocity) that the boat's existing ``gz::sim::systems::Hydrodynamics``
  plugin turns into drag through its relative-velocity damping term
  (published to gz ``/ocean_current`` by the ROS shell);
* a **wave excitation wrench** -- the hydrostatic (Froude-Krylov) buoyancy
  of the hull box under the actual wave surface minus the buoyancy under
  the flat plane Gazebo's ``Buoyancy`` system already applies -- so the
  total force on the hull equals buoyancy under the wave surface and the
  boat heaves, pitches and rolls with it (applied through gz-sim's
  ``ApplyLinkWrench`` system).

Layering rule of the package (see ``.claude/CLAUDE.md`` §2): nothing in
here imports ROS or Qt, so the offline smoke test exercises every formula.
The physics and its references are documented in ``docs/sea_state_model.md``.
"""

from .current import (CurrentField, CurrentProcess, EddyField,
                      UnitGaussMarkov)
from .hull import Box, HullBox, HullBoxes, WaveExcitation
from .model import SeaModel, SeaSample
from .motion import (DampingSpec, HighRatePose, damping_wrench,
                     hull_damping_from_config, predict_pose, simulate_3dof)
from .presets import (CurrentSpec, SeaPresets, SeaSchedule, SeaState,
                      WaveSpec, douglas_from_hs, find_default_presets_yaml,
                      load_presets, parse_command, resolve_state,
                      state_from_dict, yaw_moment_from)
from .spectrum import dispersion, jonswap, tp_from_hs_dnv
from .wavefield import WaveField
from .yaw_moment import YawMomentField, YawMomentSpec

__all__ = [
    "Box", "CurrentField", "CurrentProcess", "CurrentSpec", "EddyField",
    "UnitGaussMarkov", "douglas_from_hs", "state_from_dict", "HullBox", "HullBoxes", "WaveExcitation", "SeaModel", "SeaPresets",
    "SeaSample", "SeaSchedule", "SeaState", "WaveField", "WaveSpec",
    "DampingSpec", "HighRatePose", "damping_wrench", "hull_damping_from_config",
    "predict_pose", "simulate_3dof",
    "dispersion", "find_default_presets_yaml", "jonswap", "load_presets",
    "parse_command", "resolve_state", "tp_from_hs_dnv",
    "YawMomentField", "YawMomentSpec", "yaw_moment_from",
]
