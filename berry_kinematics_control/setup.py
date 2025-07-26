from setuptools import find_packages, setup

package_name = 'berry_kinematics_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일 설치
        ('share/' + package_name + '/launch', [
            'launch/control.launch.py',
        ]),
        # (선택) config 파일 설치
        ('share/' + package_name + '/config/sample_robot', [
            'config/sample_robot/named_poses.yaml',
        ]),
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
            # launch에서 실행되는 노드 등록
            'control_node = berry_kinematics_control.control_node:main',
            'debug_cli = berry_kinematics_control.debug_cli:main',
        ],
    },
)
