from setuptools import setup

package_name = 'polishing_removal'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eunseop',
    maintainer_email='your_email@example.com',
    description='ROS 2 node for polishing removal analysis and waypoint regeneration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'polishing_removal_node = polishing_removal.polishing_removal_node:main',
        ],
    },
)