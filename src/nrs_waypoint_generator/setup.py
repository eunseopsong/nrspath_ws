from setuptools import setup

package_name = 'nrs_waypoint_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eunseop',
    maintainer_email='eunseop@example.com',
    description='Generate offset waypoints from mesh and publish selected region to /clicked_point',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_generator = nrs_waypoint_generator.waypoint_generation:main',
        ],
    },
)