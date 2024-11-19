from setuptools import find_packages, setup
import glob

package_name = 'my_tf_tree'  # 确保包名符合规范

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),  # 修正launch文件的路径
        ('share/' + package_name + '/urdf', glob.glob('urdf/*')),  # 修正urdf文件的路径
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
            'broadcaster_node = my_tf_tree.broadcaster_node:main',  # 修正包名和节点名
            'listener_node = my_tf_tree.listener_node:main',  # 修正包名和节点名
            'radar_node = my_tf_tree.radar_node:main',  # 修正包名和节点名
            'depth_camera_node = my_tf_tree.depth_camera_node:main',  # 修正包名和节点名
        ],
    },
)