from glob import glob

from setuptools import find_packages, setup

package_name = 'kingfisher_goals'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/scenarios', glob('config/scenarios/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Felipe Wolf Batista',
    maintainer_email='luis.batista@gatech.edu',
    description='Goal streams for the Kingfisher controllers',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detection_goal = kingfisher_goals.detection_goal:main',
            'goal_publisher = kingfisher_goals.goal_publisher:main',
            'virtual_detections = kingfisher_goals.virtual_detections:main',
            'waypoint_goal = kingfisher_goals.waypoint_goal:main',
        ],
    },
)
