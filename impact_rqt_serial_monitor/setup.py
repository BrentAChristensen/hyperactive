from setuptools import find_packages, setup

package_name = 'impact_rqt_serial_monitor'

setup(
    name=package_name,
    version='0.0.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='my.address@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)