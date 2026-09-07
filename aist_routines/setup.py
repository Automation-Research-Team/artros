from setuptools import setup, find_packages
from glob import glob

package_name = "aist_routines"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config", glob("config/*.rviz")),
        ("share/" + package_name + "/config/inc", glob("config/inc/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Toshio Ueshiba",
    maintainer_email="t.ueshiba@aist.go.jp",
    description="Package with basic routines for moving robots with MoveIt",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'base                = ' + package_name + '.interactive:base',
            'assembly            = ' + package_name + '.interactive:assembly',
            'kitting             = ' + package_name + '.interactive:kitting',
            'hmi_demo            = ' + package_name + '.interactive:hmi_demo',
            'handeye_calibration = ' + package_name + '.interactive:handeye_calibration',
        ],
    },
)
