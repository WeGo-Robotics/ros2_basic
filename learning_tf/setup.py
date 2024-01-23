from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'learning_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
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
            'turtle_one_broadcaster = learning_tf.turtle_one_broadcaster:main',
            'turtle_two_broadcaster = learning_tf.turtle_two_broadcaster:main',
            'carrot_follower = learning_tf.carrot_follower:main',
        ],
    },
)
