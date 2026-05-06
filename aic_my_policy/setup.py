from pathlib import Path

from setuptools import find_packages, setup

package_name = "aic_my_policy"
policy_files = [
    str(path)
    for path in Path("policy").rglob("*")
    if path.is_file()
]

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *[
            (
                "share/" + package_name + "/" + str(Path(path).parent),
                [path],
            )
            for path in policy_files
        ],
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AIC Team",
    maintainer_email="sawansingsihag@gmail.com",
    description="Vision + impedance base policy with residual-RL fine-tuning for AIC.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
)
