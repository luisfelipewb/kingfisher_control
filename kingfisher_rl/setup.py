from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'kingfisher_rl'

# models/<task>_<actuation>/<tag>.onnx: the tracked samples, and any others present
models = [(os.path.join('share', package_name, os.path.dirname(f)), [f])
          for f in sorted(glob('models/**/*.onnx', recursive=True))]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        *models,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Felipe Wolf Batista',
    maintainer_email='luis.batista@gatech.edu',
    description='Reinforcement learning agents for the Kingfisher ASV, run with onnxruntime',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'passthrough_thrusters = kingfisher_rl.passthrough_thrusters:main',
        ],
    },
)
