from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'model_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'templates/urdf'),
         glob(os.path.join('templates/urdf', '*.template'))),
        (os.path.join('share', package_name, 'templates/xml'),
         glob(os.path.join('templates/xml', '*.template'))),
        (os.path.join('share', package_name, 'rviz'),
         glob(os.path.join('rviz', '*.rviz'))),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lizhen',
    maintainer_email='179547526@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'model_generator = model_generator.model_generator:main',
        ],
    },
)
