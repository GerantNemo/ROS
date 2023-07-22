import os
from glob import glob
from setuptools import setup

package_name = 'sub_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aymeric',
    maintainer_email='aymeric@todo.todo',
    description='Teleop from the keyboard.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_twist_keyboard_vf = sub_teleop.teleop_twist_keyboard_vf:main',
            'teleop_twist_keyboard_vf2 = sub_teleop.teleop_twist_keyboard_vf2:main',
            'teleop_twist_keyboard_vf3 = sub_teleop.teleop_twist_keyboard_vf3:main',
            'teleop_twist_keyboard_vf_gazebo = sub_teleop.teleop_twist_keyboard_vf_gazebo:main',
        ],
    },
)
