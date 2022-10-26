import os
from glob import glob
from setuptools import setup


package_name = 'sim_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch')),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('launch/bash_scripts/delay.sh')),
        (os.path.join('share', package_name), glob('launch/sublaunchers/*.launch')),
        (os.path.join('share', package_name), glob('models/rcCar_assembly/*.sdf')),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hp',
    maintainer_email='hp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'entity_spawner=sim_pkg.entity_model2:main'
        ],
    },
)
