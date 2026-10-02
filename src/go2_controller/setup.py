from setuptools import setup
from glob import glob
import os

package_name = 'go2_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.core'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'controller_node = go2_controller.controller_node:main',
        ],
    },
)
