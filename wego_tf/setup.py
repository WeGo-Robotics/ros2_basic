from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'wego_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yam]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wego',
    maintainer_email='changmin@wego-robotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_one_broadcast = wego_tf.turtle_one_broadcast:main',
            'turtle_two_broadcast = wego_tf.turtle_two_broadcast:main',
            'carrot_follow = wego_tf.carrot_follow:main',
        ],
    },
)
