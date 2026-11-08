from setuptools import find_packages, setup

PACKAGE_NAME = "hrobot_core"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hrobot",
    maintainer_email="dev@example.com",
    description="Dominio, servicios y adaptadores del robot AMR hospitalario.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ld19_node = hrobot_core.adapters.ld19_node:main",
            "supervisor_node = hrobot_core.adapters.supervisor_node:main",
        ],
    },
)
