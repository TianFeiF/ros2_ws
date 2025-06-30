from setuptools import setup
from glob import glob

package_name = 'rrml_63_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/meshes/rm_63_arm', glob('meshes/rm_63_arm/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros-industrial',
    maintainer_email='olmer@gmail.com',
    description='RML63 servo control package for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 注释掉错误的entry_points，因为相应的Python模块不存在
            # 'keyboard_control = rml_63_servo.keyboard_control:main',
            # 'simple_keyboard_control = rml_63_servo.simple_keyboard_control:main',
            # 'moveit_planner = rml_63_servo.moveit_planner:main',
            # 'test_target_sender = rml_63_servo.test_target_sender:main',
        ],
    },
)
