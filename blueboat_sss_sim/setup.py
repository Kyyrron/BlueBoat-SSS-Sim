from glob import glob

from setuptools import find_packages, setup

package_name = "blueboat_sss_sim"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/docs", glob("docs/*.md")),
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
        ],
    },
)
