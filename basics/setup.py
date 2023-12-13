from setuptools import find_packages, setup

package_name = 'basics'

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
    maintainer_email='wego@todo.todo',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = basics.topic_publisher:main',
            'listener = basics.topic_subscriber:main',
            'service_server = basics.service_server:main',
            'service_client = basics.service_client:main',
            'fancy_action_server = basics.action_server:main',
            'fancy_action_client = basics.action_client:main',
        ],
    },
)
