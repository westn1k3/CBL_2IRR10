
from setuptools import setup

package_name = 'farmbot_system'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='FarmBot system with mapping and autonomous task execution',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator_node = farmbot_system.navigator_node:main',
            'seedingbot_node = farmbot_system.seedingbot_node:main',
            'waterbot_node = farmbot_system.waterbot_node:main',
            'spraybot_node = farmbot_system.spraybot_node:main',
        ],
    },
)
