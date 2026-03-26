from setuptools import setup

package_name = 'robot_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu.launch.py']),  # 添加这行
    ],
    install_requires=['setuptools', 'pyserial'],  # 添加 pyserial
    zip_safe=True,
    maintainer='zero',
    maintainer_email='zeroeverything001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hfi_a9_imu = robot_imu.hfi_a9_ros:main',
        ],
    },
)
