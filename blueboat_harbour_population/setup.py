from glob import glob

from setuptools import find_packages, setup

PACKAGE = "blueboat_harbour_population"
# The model version: bump it whenever a plan can change for the same inputs
# (test/test_golden.py holds the plan digests that guard it).
VERSION = "1.0.0"

setup(
    name=PACKAGE,
    version=VERSION,
    packages=find_packages(exclude=["test", "test.*"]),
    package_data={PACKAGE: ["data/*.yaml"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
        (f"share/{PACKAGE}/examples", glob("examples/*.yaml")),
    ],
    install_requires=["setuptools", "numpy", "scipy", "PyYAML"],
    zip_safe=True,
    maintainer="Killian BERTRAND",
    maintainer_email="killian.bertrand2002@gmail.com",
    description="Evidence-based harbour seabed object population (mission-like placement)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            f"harbour_population = {PACKAGE}.__main__:main",
        ],
    },
)
