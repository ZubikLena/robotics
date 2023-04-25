from setuptools import setup
from os import path
from glob import glob

package_name = 'lab1_ereszka'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ereszka',
    maintainer_email='01149421@pw.edu.pl',
    description='Package for ANRO lab1',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lab1 = lab1_ereszka.lab1:main',
            'publisher = lab1_ereszka.publisher:main',
        ]
    },
)
