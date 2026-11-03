"""The ray-cast sensor (``renderer: raycast``): a second, independent
:class:`~blueboat_sss_sim.sonar.renderer.SonarRenderer`.

A full 3-D, path-resolved, coherent side-scan renderer over a triangle scene
derived from the same :class:`~blueboat_sss_sim.worldgen.scene.SceneModel`
the geometric renderer reads. Design: ``docs/engineering/raycast_design.md``;
plan and gates: ``docs/engineering/raycast_plan.md``; literature:
``docs/literature/raycast_sonar/``.

ROS-free and Qt-free like the rest of ``sonar/``. Nothing here is imported by
the geometric renderer, the node's default path or the smoke test; a GPU
backend, when one exists, is imported only by the module that uses it.
"""
