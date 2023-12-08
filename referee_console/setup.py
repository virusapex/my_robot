from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'referee_console'

data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml'])
   ]

setup(
 name=package_name,
 version='0.0.1',
 packages=find_packages(exclude=['test']),
 data_files=data_files,
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Vitalii Kudinov',
 maintainer_email='v.kudinov@g.nsu.ru',
 description='Referee console for Autorace 2023',
 license='Apache 2.0',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'mission_autorace_2023_referee = referee_console.mission_autorace_2023_referee:main'
     ],
   },
)
