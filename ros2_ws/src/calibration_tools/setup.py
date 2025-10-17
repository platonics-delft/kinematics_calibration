from setuptools import setup, find_packages

package_name = 'calibration_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='franzesegiovanni',
    maintainer_email='franzesegiovanni@outlook.com',
    description='Calibration tools for kinematic calibration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_joint_states_dataset = calibration_tools.record_joint_states_dataset:main',
        ],
    },
)