"""Field-calibration scaffolding (ROS-free).

The lifecycle this package supports, end to end:

    controlled field experiment -> immutable recordings + session manifest
    (``manifest``) -> quality checks (``quality``) -> identifiability of the
    planned fit (``identifiability``) -> fit (NOT here yet: it needs the
    private field data) -> versioned calibration artifact (``artifact``,
    checked against the parameter registry, ``registry``) -> held-out
    validation -> an effective, self-contained sonar profile for
    reproducible synthetic data (``artifact.effective_config``).

Nothing here changes a calibrated constant: the shipped profile stays the
default, and an artifact is applied only when a run asks for it.
Entry point: ``python3 -m blueboat_sss_sim.calibration --help``
(``sss_calibration``). Design: docs/engineering/.
"""

from .artifact import (ARTIFACT_FORMAT, ArtifactError, EffectiveConfig,
                       effective_config, load_artifact, validate_artifact,
                       write_effective_profile)
from .manifest import MANIFEST_FORMAT, load_manifest, validate_manifest
from .registry import (REGISTRY_FORMAT, Parameter, Registry, drift_report,
                       load_registry)

__all__ = [
    "ARTIFACT_FORMAT", "ArtifactError", "EffectiveConfig",
    "effective_config", "load_artifact", "validate_artifact",
    "write_effective_profile", "MANIFEST_FORMAT", "load_manifest",
    "validate_manifest", "REGISTRY_FORMAT", "Parameter", "Registry",
    "drift_report", "load_registry",
]
