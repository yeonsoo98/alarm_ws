from setuptools import setup
import os
from glob import glob

package_name = 'sound_alarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ys',
    maintainer_email='yeonso981@naver.com',
    description='Sound/Alarm ROS 2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sound_alarm_node = sound_alarm.sound_alarm_node:main',
            'alarm_publisher = sound_alarm.alarm_publisher:main',
        ],
    },
)
