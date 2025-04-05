from setuptools import setup

package_name = "calibration_tools"

setup(
    name=package_name,
    version="0.1.0",
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel San José Pro",
    maintainer_email="42489409+danielsanjosepro@users.noreply.github.com",
    description="Contains a few scripts to record joint states for calibration",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            f"record_joint_states_dataset = {package_name}.record_joint_states_dataset:main",
        ],
    },
)
