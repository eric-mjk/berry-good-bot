from setuptools import find_packages, setup

package_name = 'berry_good_bot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'control_node = berry_good_bot_control.control_node:main',
            'debug_node = berry_good_bot_control.debug_node:main',
        ],
    },
)
