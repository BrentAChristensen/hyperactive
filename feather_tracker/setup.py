from setuptools import setup

package_name = 'feather_tracker'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Brent Christensen',
    maintainer_email='brent@christensencap.com',
    description='Package for the DetectionToPosNode.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_to_pose_node = feather_tracker.detection_to_pose_node:main',
            'follow_feather = feather_tracker.follow_feather:main'
        ],
    },
)