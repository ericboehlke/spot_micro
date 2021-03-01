from setuptools import setup

package_name = 'spot_joints'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eric Boehlke',
    maintainer_email='eric.p.boehlke@gmail.com',
    description='A package to control the individual servos on Spot Micro',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_controller = spot_joints.joint_controller:main',
            'joint_publisher = spot_joints.joint_publisher:main'
        ],
    },
)
