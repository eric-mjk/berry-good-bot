from setuptools import find_packages, setup
import os
from glob import glob           # ✅ 함수만 import

package_name = 'berry_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),  glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),    glob('config/*.yaml')),
        (os.path.join('share', package_name, 'model'),    glob('model/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wossas',
    maintainer_email='swpants05@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node   = berry_perception.perception_node:main',
            'debug_cli         = berry_perception.debug_cli:main',
        ],
    },
)
