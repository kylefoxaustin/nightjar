from setuptools import setup

package_name = "drone_pilot"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kyle Fox",
    description="Pilot behavior model node.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pilot_node = drone_pilot.pilot_node:main",
        ],
    },
)
