from setuptools import find_packages, setup

package_name = 'my_tfTree'

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
    maintainer='yc-dlan',
    maintainer_email='1940275781@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broadcaster_node = my_tfTree.broadcaster_node:main',
            'listener_node = my_tfTree.listener_node:main',
        ],
    },
)
