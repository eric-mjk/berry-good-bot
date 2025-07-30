from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'berrybot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ⬇︎ 우리가 만든 리소스들을 설치 경로에 복사
        (os.path.join('share', package_name, 'launch'),  glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'),    glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'),  glob('meshes/*')),
        (os.path.join('share', package_name, 'rviz'),    glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swpants05',
    maintainer_email='swpants05@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
