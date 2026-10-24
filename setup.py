from glob import glob
from setuptools import setup

pkg = "ekf_fusion"
setup(
    name=pkg,
    version="0.1.0",
    packages=[pkg],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{pkg}"]),
        (f"share/{pkg}", ["package.xml"]),
        (f"share/{pkg}/launch", glob("launch/*.py")),
        (f"share/{pkg}/config", glob("config/*.yaml")),
        (f"share/{pkg}/urdf", glob("urdf/*")),
        (f"share/{pkg}/worlds", glob("worlds/*")),
        (f"share/{pkg}/rviz", glob("rviz/*")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Shanmukh Narayana Chaganti",
    description="IMU + wheel odometry fusion with robot_localization",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wheel_odometry_node = ekf_fusion.wheel_odometry_node:main",
            "imu_relay_node = ekf_fusion.imu_relay_node:main",
            "route_driver_node = ekf_fusion.route_driver_node:main",
            "logger_node = ekf_fusion.logger_node:main",
            "path_viz_node = ekf_fusion.path_viz_node:main",
        ],
    },
)
