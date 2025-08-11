from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'midterm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaeyeolkim',
    maintainer_email='jaeyeolkim@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emergency_node = midterm.emergency:main',
            'wpdriving_node = midterm.wpdriving:main',
            
        ],
    },
)
