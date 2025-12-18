from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'eureka_imuwt901'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kolomiyets',
    maintainer_email='rslvklmc@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wt901_calibration_node = eureka_imuwt901.wt901_calibration:main',
            'wt901_node = eureka_imuwt901.wt901_node:main',
            'solution = eureka_imuwt901.solution:main'
        ],
    },
)