from setuptools import find_packages, setup

package_name = 'intermediate'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'qos_publisher = intermediate.qos_publisher:main',
            'qos_subscriber = intermediate.qos_subscriber:main',
            'executor_example = intermediate.executor_example:main',
            'service_server = intermediate.service_server:main',
            'service_client = intermediate.service_client:main',
            'fancy_action_server = intermediate.fancy_action_server:main',
            'fancy_action_client = intermediate.fancy_action_client:main',
        ],
    },
)
