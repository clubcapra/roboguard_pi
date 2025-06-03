from glob import glob
from setuptools import find_packages, setup

package_name = 'roboguard_actions'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Capra',
    maintainer_email='capra@ens.etsmtl.ca',
    description='Roboguard action adapter',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actions = roboguard_actions.actions_node:main',
            'test_actions = roboguard_actions.test_actions_node:main',
        ],
    },
)
