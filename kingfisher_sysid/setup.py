from glob import glob

from setuptools import find_packages, setup

package_name = 'kingfisher_sysid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Felipe Wolf Batista',
    maintainer_email='luis.batista@gatech.edu',
    description='Open-loop thrust sequences for system identification',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sequence_runner = kingfisher_sysid.sequence_runner:main',
        ],
    },
)
