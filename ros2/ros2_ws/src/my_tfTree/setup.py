from setuptools import find_packages, setup
import glob
package_name = 'my_tfTree'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch'), glob.glob('urdf/*')
        ('share/' + package_name + '/urdf'), glob.glob('urdf/*')
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yc-dlan',
    maintainer_email='1940275781@qq.com',
    description='build tf tree',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broadcaster_node = my_tfTree.broadcaster_node:main',
            'listener_node = my_tfTree.listener_node:main',
            'radar_node = my_tfTree.radar_node:main',
            'depth_camera_node = my_tfTree.depth_camera_node:main',
        ],
        'ros2_launch': [
        ]
    },
)
