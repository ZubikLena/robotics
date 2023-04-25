import os
from glob import glob
from setuptools import setup

package_name = 'py_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py'))
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='lena',
    maintainer_email='01149451@pw.edu.pl',
    description='Python motion generator',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'py_pub = py_motion.py_pub:main'
        ],
    },
)
