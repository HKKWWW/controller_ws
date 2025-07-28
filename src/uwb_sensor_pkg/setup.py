from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uwb_sensor_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # 添加launch文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  # 添加配置文件
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='synapath',
    maintainer_email='hkkwww@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uwb_node = uwb_sensor_pkg.uwb_node:main',
            'euler_listener_node = uwb_sensor_pkg.euler_listener:main'
        ],
    },
)
