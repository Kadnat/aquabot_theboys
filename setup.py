from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aquabot_theboys'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch/'),
         glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nathanael',
    maintainer_email='blavonathanael@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensors = aquabot_theboys.sensors:main',
            'cmd_motors = aquabot_theboys.cmd_motors:main',
            'filters = aquabot_theboys.filters:main',
            'hub = aquabot_theboys.hub:main',
            'object_detection_node = aquabot_theboys.object_detection_node:main',
        ],
    },
)
