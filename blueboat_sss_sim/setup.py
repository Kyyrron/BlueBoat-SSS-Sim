import importlib.util
import json
import os
from glob import glob

from setuptools import find_packages, setup
from setuptools.command.build_py import build_py as _build_py

package_name = "blueboat_sss_sim"
VERSION = "1.0.0"


class _StampedBuildPy(_build_py):
    """Write ``blueboat_sss_sim/_build_stamp.json`` into the BUILD (never the
    source tree): when this copy was built, from which source root, at which
    git state and code digest (``core/provenance.py``). The sonar node
    publishes it and warns when the installed copy is older than the source
    tree -- the package installs by copy, and a stale install once produced a
    session that did not show the current code (review of 2026-11-11)."""

    def run(self):
        super().run()
        here = os.path.dirname(os.path.abspath(__file__))
        try:
            spec = importlib.util.spec_from_file_location(
                "_blueboat_sss_sim_provenance",
                os.path.join(here, package_name, "core", "provenance.py"))
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            out = os.path.join(self.build_lib, package_name, mod.STAMP_NAME)
            os.makedirs(os.path.dirname(out), exist_ok=True)
            with open(out, "w", encoding="utf-8") as f:
                json.dump(mod.make_stamp(here, version=VERSION), f, indent=1)
        except Exception as e:  # noqa: BLE001 -- a stamp must never break a build
            print(f"[{package_name}] build stamp not written: {e}")


setup(
    name=package_name,
    version=VERSION,
    cmdclass={"build_py": _StampedBuildPy},
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/docs", glob("docs/*.md")),
        ("share/" + package_name + "/docs/engineering",
         glob("docs/engineering/*.md")),
        ("share/" + package_name + "/msg_reference", glob("msg_reference/*.msg")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Killian BERTRAND",
    maintainer_email="killian.bertrand2002@gmail.com",
    description="Synthetic Side Scan Sonar platform for the BlueBoat simulator",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            # ROS nodes
            "sss_sim_node = blueboat_sss_sim.ros.sss_sim_node:main",
            "mavros_shim_node = blueboat_sss_sim.ros.mavros_shim_node:main",
            "sea_state_node = blueboat_sss_sim.ros.sea_state_node:main",
            # World builder GUI (PySide6; import-guarded)
            "world_builder = blueboat_sss_sim.builder.app:main",
            # Offline tools
            "generate_world = blueboat_sss_sim.worldgen.generate:main",
            "export_scene_maps = blueboat_sss_sim.worldgen.export_maps:main",
            "mission_metrics = blueboat_sss_sim.analysis.cli:main",
            "sss_calibration_report = blueboat_sss_sim.analysis.calibration_cli:main",
            "sss_svlog_compare = blueboat_sss_sim.analysis.svlog_compare:main",
            "sss_sea_validation = blueboat_sss_sim.analysis.sea_validation:main",
            "sss_calibration = blueboat_sss_sim.calibration.__main__:main",
        ],
    },
)
