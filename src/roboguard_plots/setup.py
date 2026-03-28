from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roboguard_plots'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        ('share/' + package_name + "/config", glob(os.path.join('config', "*"))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='capra',
    maintainer_email='capra@ens.etsmtl.ca',
    description='Package for plotting values of roboguard',
    license='MIT',
    extras_require={
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
