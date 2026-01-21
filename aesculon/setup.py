from setuptools import setup
import os
from glob import glob

package_name = 'aesculon'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'action'), glob('action/*.action')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sourish Senapati',
    maintainer_email='sourishs.chem.ug@jadavpuruniversity.in',
    description='Thermodynamic Safety Agent',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aesculon_core = aesculon.core_node:main',
            'aesculon_console = aesculon.console_node:main',
        ],
    },
)
