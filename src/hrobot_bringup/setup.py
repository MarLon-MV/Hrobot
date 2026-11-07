from setuptools import find_packages, setup

PACKAGE_NAME = "hrobot_bringup"
RESOURCE_DIRS = ("launch", "config", "rviz", "maps")

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        *[("share/" + PACKAGE_NAME + "/" + d, [d + "/.gitkeep"]) for d in RESOURCE_DIRS],
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hrobot",
    maintainer_email="dev@example.com",
    description="Launch, configuración, RViz y mapas del robot AMR hospitalario.",
    license="Apache-2.0",
    entry_points={"console_scripts": []},
)
