from setuptools import setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'urdf_lab5'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('urdf_lab5/*.yaml')),
        (os.path.join('share', package_name), glob('urdf_lab5/*.json'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lena',
    maintainer_email='01149451@pw.edu.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'state_publisher = urdf_lab5.state_publisher:main',
        'jint = urdf_lab5.jint:main',
        'oint = urdf_lab5.oint:main',
        'jint_client = urdf_lab5.jint_client:main',
        'oint_client = urdf_lab5.oint_client:main'
        ],
    },
)
